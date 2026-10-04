"""The whole pipeline (design section 10): setup, global selection, regret-order
commit, negotiated rip-up, refinement, realisation with DRC feedback, and via
insertion for connections that no single layer can complete (section 12)."""
import copy
import math
import time
from dataclasses import dataclass, field

from weaveengine import parallel
from weaveengine.board import Board, Pad
from weaveengine.plan import candidates as cand_mod
from weaveengine.plan import vias as via_mod
from weaveengine.plan.commit import commit_all
from weaveengine.plan.context import Connection, Context, Layer, Options, decompose
from weaveengine.plan.ripup import legalise, negotiate, refine
from weaveengine.plan.select import select
from weaveengine.realize.relax import polyline_length, realize
from weaveengine.realize.smooth import smooth as smooth_corners
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
    wire_pads: dict[int, tuple[int, int]] = field(default_factory=dict)  # wire -> the two pads it joins

    def summary(self) -> "Result":
        """The same result without the maps and states: small enough to send
        between processes, and all a front end needs to draw and export."""
        open_pairs = {w: self.connections[w] for w in self.unrouted}
        return Result(self.board, [], open_pairs, self.polylines, self.wire_net, self.wire_layer,
                      self.violations, self.unrouted, self.vias, self.teardrops, self.stats,
                      {w: (c.src, c.dst) for w, c in self.connections.items() if w in self.polylines})

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
                drc_rounds: int = 4, drop_violators: bool = True, max_via_rounds: int = 4,
                workers: int | None = None, progress=None, events=None) -> Result:
    """Routes the board.

    ``workers``: processes to use (None = all cores, 1 = none besides this one);
    the result does not depend on it. ``progress``: optional callable
    (phase, done, total), called as the work advances; ``phase`` starts with
    "pass N: " for the N-th routing pass (a new pass starts whenever vias are added).
    ``events``: optional callable taking a dict, for live front ends: progress,
    snapshots of the routing as it grows, and pass results, from every raced
    variant (see ``weaveengine.session`` for the event types).
    """
    t0 = time.perf_counter()
    options = options or Options()
    work = copy.copy(board)
    work.pads = list(board.pads)
    pairs = origins = None
    best = None
    via_rounds = 0
    while True:
        layers = [Layer(i, name, pmap) for i, (name, pmap) in enumerate(zip(
            work.layers, parallel.run(_build_task, work, range(len(work.layers)), workers)))]
        if pairs is None:
            pairs, buried = decompose(work, layers)
            origins = list(range(len(pairs)))
        run_params = copy.copy(params) if params else CostParams.for_map(layers[0].pmap)
        label = f"pass {via_rounds + 1}: "
        ctx = Context(work, layers, pairs, run_params, options, seed, workers,
                      (lambda phase, done=0.0, total=1.0: progress(label + phase, done, total)) if progress else None,
                      events)
        ctx.pass_index = via_rounds + 1
        lines, violations, wire_net = _route_portfolio(ctx, drc_rounds, drop_violators)
        if events is not None:
            ctx.variant = -1  # the outcome of the pass, whichever variant it came from
            ctx.emit_snapshot(force=True, lines=lines)
            events({"type": "pass", "pass": via_rounds + 1, "open": len(ctx.unrouted), "connections": len(ctx.conns),
                    "vias": sum(1 for p in work.pads if p.is_via)})
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
        ctx.report("placing vias")
        open_ids = [i + 1 for i in range(len(pairs)) if i + 1 in ctx.unrouted]
        guess = dict(zip(open_ids, parallel.run(_propose_task, (ctx, list(taken)), open_ids, workers)))
        spacing = work.rules.via_diameter + work.rules.clearance + work.rules.pitch
        for i, pair in enumerate(pairs):
            conn = ctx.conns[i + 1]
            at = guess.get(conn.wire_id)
            if at is not None and any(math.dist(at, t) < spacing for t in taken):
                at = via_mod.propose(ctx, conn, taken)  # too close to a via placed just before: look again
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
    smoothed = (0, 0)
    if options.smooth:
        tasks = [({w: lines[w] for w, li in wire_layer.items() if li == layer.index}, layer.index) for layer in ctx.layers]
        for new, rounded, kept in parallel.run(_smooth_task, (work, wire_net), tasks, workers):
            lines.update(new)
            smoothed = (smoothed[0] + rounded, smoothed[1] + kept)
    drops: dict[int, list] = {}
    if options.teardrops:
        tasks = []
        for layer in ctx.layers:
            on_layer = {w: lines[w] for w, li in wire_layer.items() if li == layer.index}
            ends = {w: (ctx.conns[w].src, ctx.conns[w].dst) for w in on_layer}
            tasks.append((on_layer, ends, layer.index))
        limits = (options.teardrop_max_length, options.teardrop_max_width, options.teardrop_breathing)
        for found in parallel.run(_teardrop_task, (work, wire_net, limits), tasks, workers):
            drops.update(found)

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
        "corners_rounded": smoothed[0],
        "corners_left_sharp": smoothed[1],
        "via_rounds": via_rounds,
        "buried_pads": buried,
        "wires_per_layer": {l.name: sum(1 for li in wire_layer.values() if li == l.index) for l in ctx.layers},
        "time_route": t_route - t0,
        "time_total": t_end - t0,
        "triangles": sum(l.pmap.num_triangles for l in ctx.layers),
        "edges": sum(l.pmap.num_edges for l in ctx.layers),
    }
    return Result(work, ctx.layers, ctx.conns, lines, wire_net, wire_layer, violations, sorted(ctx.unrouted), vias, drops, stats)


# Search settings of the raced variants: (heuristic weight). Variant 0 is the plain configuration.
VARIANTS = (1.0, 1.5, 1.25, 1.75, 1.0, 2.0, 1.5, 1.25)
HEAT = 4


def _route_portfolio(ctx: Context, drc_rounds: int, drop_violators: bool):
    """One routing pass, raced over several variants (section 22).

    How a pass ends depends strongly on small differences early on, so the
    spare cores each run the whole pass with a different seed and search
    weighting. The first variant, in their fixed order, that connects
    everything is taken (the plain configuration is first, so nothing is lost
    when it succeeds); if none does, the best is kept: fewest open connections,
    then shortest. The choice does not depend on which finishes first.
    """
    count = ctx.options.portfolio
    if count <= 0:
        workers = parallel.cpu_count() if ctx.workers is None else ctx.workers
        count = max(1, min(len(VARIANTS), workers))
    if count == 1 or not parallel.can_fork() or parallel._inside:
        return _route_once(ctx, drc_rounds, drop_violators)
    # Raced in heats of four: more at once than that only slows each other
    # down (shared caches, efficiency cores), and the plain variant is in the
    # first heat, so an easy board is not held up by the rest.
    complete = lambda outcome: outcome[0][0] == 0 and outcome[0][1] == 0
    outcomes: list = []
    for start in range(0, count, HEAT):
        heat = range(start, min(count, start + HEAT))
        ctx.report("racing variants %d-%d of %d" % (heat[0] + 1, heat[-1] + 1, count))
        stop = parallel.stop_flag()
        found = parallel.first_accepted(_variant_task, (ctx, drc_rounds, drop_violators, stop), heat,
                                        lambda outcome: outcome is not None and complete(outcome), len(heat), stop)
        outcomes += [o for o in found if o is not None]
        if outcomes and complete(outcomes[-1]):
            break
    best = len(outcomes) - 1 if outcomes[-1][0][:2] == (0, 0) else min(range(len(outcomes)), key=lambda i: (outcomes[i][0], i))
    _, snaps, caps, hists, pres, flags, rounds, lines, violations, wire_net = outcomes[best]
    ctx.restore(snaps)
    for layer, cap, hist in zip(ctx.layers, caps, hists):
        layer.state.cap[:] = cap
        layer.state.hist[:] = hist
    ctx.params.pres_fac = pres
    ctx.rounds = rounds
    for w, (fails, dead) in flags.items():
        ctx.conns[w].fails, ctx.conns[w].dead = fails, dead
    ctx.report("variant %d of %d kept" % (best + 1, count), 1.0, 1.0)
    return lines, violations, wire_net


def _variant_task(shared, index: int):
    ctx, drc_rounds, drop_violators, stop = shared
    ctx.stop = stop
    ctx.seed += index
    ctx.variant = index
    ctx.params.h_weight = VARIANTS[index % len(VARIANTS)]
    if index:
        ctx.progress = None  # the console bar follows the plain variant; live events come from all of them
    try:
        lines, violations, wire_net = _route_once(ctx, drc_rounds, drop_violators)
    except parallel.Stopped:
        return None  # an earlier variant already connected everything
    key = (len(ctx.unrouted), len(violations), round(sum(polyline_length(l) for l in lines.values()), 6))
    return (key, ctx.snapshot(), [l.state.cap.copy() for l in ctx.layers], [l.state.hist.copy() for l in ctx.layers],
            ctx.params.pres_fac, {w: (c.fails, c.dead) for w, c in ctx.conns.items()}, ctx.rounds,
            lines, violations, wire_net)


def _route_once(ctx: Context, drc_rounds: int, drop_violators: bool):
    """Phases 1 - 4 and realisation on a fixed set of pads and connections."""
    opts = ctx.options
    conns = list(ctx.conns.values())
    # Phase 1: candidates for every connection on the empty map, one chosen per connection.
    cands: dict[int, list] = {}
    selection: dict[int, int] = {}
    ctx.report("candidates")
    if opts.global_selection:
        cands = cand_mod.generate_all(ctx)
        ctx.report("global selection")
        selection = select(ctx, cands)
    # Phase 2: commit in regret order.
    commit_all(ctx, cands, selection)
    # Phase 3: negotiated rip-up and reroute.
    if opts.ripup:
        negotiate(ctx)
    legalise(ctx)
    # Phase 4: topology refinement.
    if opts.refine:
        ctx.report("refinement")
        refine(ctx)

    # Realisation, with DRC feedback into Phase 3 (13.2).
    ctx.report("geometry")
    lines, violations, wire_net = _realize(ctx)
    ctx.emit_snapshot(force=True, lines=lines)
    for fix in range(drc_rounds):
        if not violations:
            break
        if not _penalise(ctx, violations):
            break
        ctx.report("design-rule repair", fix, drc_rounds)
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
    ctx.emit_snapshot(force=True, lines=lines)
    return lines, violations, wire_net


def _realize(ctx: Context):
    """Geometry of every layer. Layers are independent, so each gets a worker."""
    lines, violations, wire_net = {}, [], {}
    for layer, (changed, snap, l, v, n) in zip(ctx.layers, parallel.run(_realize_task, ctx, range(len(ctx.layers)), ctx.workers)):
        if snap is not None:  # computed in a worker: bring its straightened wire ends over
            layer.state.restore(snap)
        for w in changed:
            layer.paths[w] = path_from_steps(layer.pmap, layer.state.wire_path[w])
        lines.update(l)
        violations += v
        wire_net.update(n)
    return lines, violations, wire_net


def _realize_task(ctx: Context, index: int):
    layer = ctx.layers[index]
    # Let each trace leave its pad through the edge it is heading for (M8).
    changed = straighten(layer.state, ctx.board)
    lines, violations, wire_net = realize(layer.state, ctx.board)
    snap = layer.state.snapshot() if changed and parallel._inside else None
    return changed, snap, lines, violations, wire_net


def _build_task(board: Board, index: int) -> PlanarMap:
    return planar_map.build(board, index)


def _propose_task(shared, wire_id: int):
    ctx, taken = shared
    return via_mod.propose(ctx, ctx.conns[wire_id], taken)


def _smooth_task(shared, task):
    """Round the corners of one layer; keep the original if the check finds anything wrong with the result."""
    from weaveengine.realize import drc
    board, wire_net = shared
    on_layer, index = task
    new, rounded, kept = smooth_corners(board, on_layer, wire_net, index)
    if drc.check(board, new, wire_net, index):
        return on_layer, 0, rounded + kept
    return new, rounded, kept


def _teardrop_task(shared, task):
    board, wire_net, limits = shared
    on_layer, ends, index = task
    return make_teardrops(board, on_layer, wire_net, ends, index, *limits)


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
