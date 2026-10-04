"""The whole pipeline (design section 10): setup, global selection, regret-order
commit, negotiated rip-up, refinement, realisation with DRC feedback, and via
insertion for connections that no single layer can complete (section 12)."""
import copy
import math
import time
from dataclasses import dataclass, field

from weaveengine.board import Board, Pad
from weaveengine.plan import candidates as cand_mod
from weaveengine.plan import vias as via_mod
from weaveengine.plan.commit import commit_all
from weaveengine.plan.context import Connection, Context, Layer, Options, decompose
from weaveengine.plan.ripup import legalise, negotiate, refine
from weaveengine.plan.select import select
from weaveengine.realize.relax import polyline_length, realize
from weaveengine.realize.teardrop import teardrops as make_teardrops
from weaveengine.realize.terminals import straighten
from weaveengine.topo.runs import path_from_steps
from weaveengine.topo import planar_map
from weaveengine.topo.costs import CostParams
from weaveengine.topo.planar_map import GATE, PlanarMap
from weaveengine.topo.state import TopoState


@dataclass
class Result:
    board: Board                 # working copy of the input, with the vias added as pads
    layers: list[Layer]
    connections: dict[int, Connection]
    polylines: dict[int, list[tuple[float, float]]]
    wire_net: dict[int, int]
    wire_layer: dict[int, int]
    violations: list
    unrouted: list[int]
    vias: list[Pad] = field(default_factory=list)
    teardrops: dict[int, list] = field(default_factory=dict)
    stats: dict = field(default_factory=dict)

    @property
    def complete(self) -> bool:
        return not self.unrouted and not self.violations

    @property
    def pmap(self) -> PlanarMap:
        return self.layers[0].pmap

    @property
    def state(self) -> TopoState:
        return self.layers[0].state


def route_board(board: Board, params: CostParams | None = None, options: Options | None = None, seed: int = 0,
                drc_rounds: int = 4, drop_violators: bool = True, max_via_rounds: int = 4) -> Result:
    t0 = time.perf_counter()
    options = options or Options()
    work = copy.copy(board)
    work.pads = list(board.pads)
    pairs = origins = None
    best = None
    via_rounds = 0
    while True:
        layers = [Layer(i, name, planar_map.build(work, i)) for i, name in enumerate(work.layers)]
        if pairs is None:
            pairs, buried = decompose(work, layers)
            origins = list(range(len(pairs)))
        run_params = copy.copy(params) if params else CostParams.for_map(layers[0].pmap)
        ctx = Context(work, layers, pairs, run_params, options, seed)
        lines, violations, wire_net = _route_once(ctx, drc_rounds, drop_violators)
        broken = {origins[w - 1] for w in ctx.unrouted}
        done = len(set(origins)) - len(broken)
        key = (done, -sum(1 for p in work.pads if p.is_via))
        if best is None or key > best[0]:
            best = (key, ctx, lines, violations, wire_net, copy.copy(work), list(work.pads), list(origins))
        if not ctx.unrouted or not options.vias or len(layers) < 2 or via_rounds >= max_via_rounds:
            break

        # Split every connection that is still open at a via site and route again.
        taken = [p.centre for p in work.pads if p.is_via]
        new_pairs, new_origins, added = [], [], 0
        for i, pair in enumerate(pairs):
            conn = ctx.conns[i + 1]
            at = via_mod.propose(ctx, conn, taken) if conn.wire_id in ctx.unrouted else None
            if at is None:
                new_pairs.append(pair)
                new_origins.append(origins[i])
                continue
            via = via_mod.add_via(work, conn.net_id, at)
            taken.append(at)
            new_pairs += [(conn.net_id, conn.src, via), (conn.net_id, via, conn.dst)]
            new_origins += [origins[i], origins[i]]
            added += 1
        if not added:
            break
        pairs, origins = new_pairs, new_origins
        via_rounds += 1

    key, ctx, lines, violations, wire_net, work, pads, origins = best
    work.pads = pads
    t_route = time.perf_counter()

    wire_layer = {w: c.layer for w, c in ctx.conns.items() if c.layer is not None}
    used = {p for w in wire_layer for p in (ctx.conns[w].src, ctx.conns[w].dst)}
    vias = [p for p in work.pads if p.is_via and p.pad_id in used]
    drops: dict[int, list] = {}
    if options.teardrops:
        for layer in ctx.layers:
            on_layer = {w: lines[w] for w, li in wire_layer.items() if li == layer.index}
            ends = {w: (ctx.conns[w].src, ctx.conns[w].dst) for w in on_layer}
            drops.update(make_teardrops(work, on_layer, wire_net, ends, layer.index))

    for layer in ctx.layers:
        assert layer.state.check_invariants(), "topological invariant broken"
    t_end = time.perf_counter()
    routed = sorted(wire_layer)
    air = sum(ctx.conns[w].air_len for w in routed)
    length = sum(polyline_length(lines[w]) for w in routed)
    total = len(set(origins))
    stats = {
        "connections": total,
        "routed": key[0],
        "completion": key[0] / total if total else 1.0,
        "wires": len(routed),
        "vias": len(vias),
        "length": length,
        "airwire_length": air,
        "length_ratio": length / air if air else math.nan,
        "ripup_rounds": ctx.rounds,
        "via_rounds": via_rounds,
        "buried_pads": buried,
        "wires_per_layer": {l.name: sum(1 for li in wire_layer.values() if li == l.index) for l in ctx.layers},
        "time_route": t_route - t0,
        "time_total": t_end - t0,
        "triangles": sum(l.pmap.num_triangles for l in ctx.layers),
        "edges": sum(l.pmap.num_edges for l in ctx.layers),
    }
    return Result(work, ctx.layers, ctx.conns, lines, wire_net, wire_layer, violations, sorted(ctx.unrouted), vias, drops, stats)


def _route_once(ctx: Context, drc_rounds: int, drop_violators: bool):
    """Phases 1 - 4 and realisation on a fixed set of pads and connections."""
    opts = ctx.options
    conns = list(ctx.conns.values())
    # Phase 1: candidates for every connection on the empty map, one chosen per connection.
    cands: dict[int, list] = {}
    selection: dict[int, int] = {}
    if opts.global_selection:
        cands = {c.wire_id: cand_mod.generate(ctx, c) for c in conns}
        selection = select(ctx, cands)
    # Phase 2: commit in regret order.
    commit_all(ctx, cands, selection)
    # Phase 3: negotiated rip-up and reroute.
    if opts.ripup:
        negotiate(ctx)
    legalise(ctx)
    # Phase 4: topology refinement.
    if opts.refine:
        refine(ctx)

    # Realisation, with DRC feedback into Phase 3 (13.2).
    lines, violations, wire_net = _realize(ctx)
    for _ in range(drc_rounds):
        if not violations:
            break
        if not _penalise(ctx, violations):
            break
        if opts.ripup:
            negotiate(ctx, max_rounds=ctx.rounds + 5)
        legalise(ctx)
        lines, violations, wire_net = _realize(ctx)
    # Whatever still violates is not a routed connection: drop it.
    while violations and drop_violators:
        dropped: set[int] = set()
        for v in violations:
            if not dropped.intersection(v.wires):
                dropped.add(max(v.wires))
        for w in dropped:
            ctx.rip(w)
        lines, violations, wire_net = _realize(ctx)
    return lines, violations, wire_net


def _realize(ctx: Context):
    lines, violations, wire_net = {}, [], {}
    for layer in ctx.layers:
        # Let each trace leave its pad through the edge it is heading for (M8).
        for w in straighten(layer.state, ctx.board):
            layer.paths[w] = path_from_steps(layer.pmap, layer.state.wire_path[w])
        l, v, n = realize(layer.state, ctx.board)
        lines.update(l)
        violations += v
        wire_net.update(n)
    return lines, violations, wire_net


def _penalise(ctx: Context, violations) -> bool:
    """Capacity was only an estimate: lower it on the gate nearest each
    violation and add history, so Phase 3 moves a wire away. Returns False if
    there was nothing to adjust."""
    changed = False
    for v in violations:
        layer = ctx.layers[v.layer]
        pmap, state = layer.pmap, layer.state
        paths = [layer.paths[w] for w in v.wires if w in layer.paths]
        if not paths:
            continue
        shared = set(paths[0].gates).intersection(*[p.gates for p in paths[1:]]) if len(paths) > 1 else set()
        pool = shared or {g for p in paths for g in p.gates}
        pool = [g for g in pool if pmap.edge_kind_list[g] == GATE and state.usage(g) > 0]
        if not pool:
            continue
        g = min(pool, key=lambda e: _dist_to_edge(pmap, e, v.at))
        state.cap[g] = min(state.cap[g], state.load[g] - 1.0)
        state.hist[g] += ctx.params.hist_inc
        changed = True
    return changed


def _dist_to_edge(pmap: PlanarMap, e: int, p) -> float:
    (ax, ay), (bx, by) = (pmap.vxy[v] for v in pmap.edge_v_list[e])
    dx, dy = bx - ax, by - ay
    t = ((p[0] - ax) * dx + (p[1] - ay) * dy) / (dx * dx + dy * dy)
    t = min(1.0, max(0.0, t))
    return math.hypot(p[0] - ax - t * dx, p[1] - ay - t * dy)
