"""The whole pipeline (design section 10): setup, global selection, regret-order
commit, negotiated rip-up, refinement, realisation with DRC feedback, and via
insertion for connections that no single layer can complete (section 12)."""
import copy
import math
import time
from dataclasses import dataclass, field, replace

from shapely.geometry import Polygon

from weaveengine import parallel
from weaveengine.board import Board, Pad
from weaveengine.plan import candidates as cand_mod
from weaveengine.plan.commit import commit_all
from weaveengine.plan.context import Connection, Context, Layer, Options, decompose
from weaveengine.plan import skeleton
from weaveengine.plan.fanout import join as join_pours
from weaveengine.plan.path import find
from weaveengine.plan.ripup import legalise, negotiate, settle
from weaveengine.plan.select import select
from weaveengine.plan.slide import slide
from weaveengine.realize.relax import copper_length, polyline_length, realize
from weaveengine.realize.smooth import smooth as smooth_corners
from weaveengine.realize.taper import WEDGE as TAPER_WEDGE, taper as make_tapers, with_wedges
from weaveengine.realize.teardrop import teardrops as make_teardrops
from weaveengine.realize.terminals import straighten
from weaveengine.topo.runs import path_from_steps
from weaveengine.topo import planar_map, sites
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
    widths: dict[int, list] = field(default_factory=dict)  # wire -> [(points, width)], for traces drawn in pieces of different widths (``realize.taper``)
    wedges: dict[int, list] = field(default_factory=dict)  # wire -> the wedges (polygons) where a tapered trace narrows (``realize.taper``)
    kept: dict | None = field(default=None, repr=False)  # the routing itself, to go on from (``route_board(resume=...)``)

    def summary(self) -> "Result":
        """The same result without the maps and states: small enough to send
        between processes, and all a front end needs to draw and export."""
        open_pairs = {w: self.connections[w] for w in self.unrouted}
        return Result(self.board, [], open_pairs, self.polylines, self.wire_net, self.wire_layer,
                      self.violations, self.unrouted, self.vias, self.teardrops, self.stats,
                      {w: (c.src, c.dst) for w, c in self.connections.items() if w in self.polylines}, self.widths, self.wedges, self.kept)

    @property
    def complete(self) -> bool:
        return not self.unrouted and not self.violations

    @property
    def pmap(self) -> PlanarMap:
        return self.layers[0].pmap

    @property
    def state(self) -> TopoState:
        return self.layers[0].state


def redo_teardrops(result: "Result", on: bool = True, max_length: float = Options.teardrop_max_length,
                   max_width: float = Options.teardrop_max_width, breathing: float = Options.teardrop_breathing) -> "Result":
    """``result`` with its teardrops made again, to other limits, or taken
    away (not ``on``), and nothing else changed: the routing is not touched.
    Teardrops are drawn last, round the finished traces, vias and pads
    (``realize.teardrop``), so they can be changed after routing. (The fan-out
    vias were placed clear of the teardrops of the routing; new ones keep
    clear of those vias and traces as any teardrop keeps clear of copper.)"""
    drops: dict[int, list] = {}
    pads = {**{w: (c.src, c.dst) for w, c in result.connections.items()}, **result.wire_pads}
    if on:
        for layer in sorted(set(result.wire_layer.values())):
            on_layer = {w: pts for w, pts in result.polylines.items() if result.wire_layer.get(w) == layer}
            drops.update(make_teardrops(result.board, on_layer, result.wire_net, pads, layer, max_length, max_width, breathing))
    return replace(result, teardrops=drops)


def route_board(board: Board, params: CostParams | None = None, options: Options | None = None, seed: int = 0,
                drc_rounds: int = 4, drop_violators: bool = True,
                workers: int | None = None, progress=None, events=None, resume: dict | None = None,
                deadline: float | None = None, checkpoint=None) -> Result:
    """Routes the board.

    ``resume``: the ``kept`` of an earlier result for this board with these
    rules. The routing goes on from there instead of from nothing: what is
    open is negotiated for again, everything is settled and drawn again, and
    the outcome replaces the earlier one only if it is better (fewer open,
    then fewer violations, then less trace); otherwise the earlier one is
    returned as it was. ValueError if the board or its rules are not the ones
    that routing was made for.

    ``workers``: processes to use (None = all cores, 1 = none besides this one);
    the result does not depend on it. ``progress``: optional callable
    (phase, done, total), called as the work advances.
    ``events``: optional callable taking a dict, for live front ends: progress,
    snapshots of the routing as it grows, and the result of the pass, from every
    raced variant (see ``weaveengine.session`` for the event types).

    ``deadline``: a time on the clock (``time.time()``) after which nothing more
    is done to better the routing (``Context.late``): rip-up stops, no further
    heat of variants is started, and the best routing so far is settled, drawn
    and returned. What is left to do then (making it legal, the geometry and its
    check) still takes a while. None: no limit. With one, the result depends on
    how fast the machine is.
    ``checkpoint``: optional callable (key, result), called in this process
    whenever a pass or a raced variant has finished, with how good its routing
    is (``_key``: lower is better) and a ``Result`` of it that ``write_ses``
    can write (traces and vias, without rounded corners, teardrops or the vias
    down to the planes): to keep the best so far, should the run not finish.
    """
    t0 = time.perf_counter()
    options = options or Options()
    layers = _layers(board, workers)
    pairs, buried = decompose(board, layers)
    narrowed: dict[int, float] = {}
    full_rules, fine = board.rules, set(buried)
    if options.narrow and buried:
        board, layers, narrowed = reach(board, layers, buried, workers)
        if narrowed:
            pairs, buried = decompose(board, layers)
    run_params = copy.copy(params) if params else CostParams.for_map(layers[0].pmap)
    drawn_board = board  # (with the rules the traces are drawn and checked with: the skeletons' widths are the routing's only)
    skeletons = skeleton.plan(board, layers) if options.skeleton and board.pours else []
    if skeletons:
        board = copy.copy(board)
        board.rules = copy.copy(board.rules)
        board.rules.net_width = dict(board.rules.net_width)
        nets = {net for net, _, _, _ in skeletons}
        for net in nets:  # (a net with a skeleton is planned at the skeleton's width, all along)
            drawn_board.rules.net_full.pop(net, None)
            drawn_board.rules.fine.pop(net, None)
        board.rules.net_width.update({net: skeleton.width(board, net) for net in nets})
        skeleton.hold(board, layers, nets)
    ctx = Context(board, layers, pairs, run_params, options, seed, workers,
                  (lambda phase, done=0.0, total=1.0: progress("pass 1: " + phase, done, total)) if progress else None, events,
                  skeletons)
    ctx.deadline = deadline
    # (As built: before any via is put into them. With the areas no via may touch, which are not in them.)
    maps = [_print(layer.pmap) for layer in layers] + [area.wkt for area in board.no_via] + (["skeletons"] if skeletons else [])
    if resume is not None:
        if resume["maps"] != maps:
            raise ValueError("the board or its rules have changed since that routing was made: it cannot be gone on from (route it from nothing: Restart)")
        _adopt(ctx, resume)
    lines, violations, wire_net = _route_portfolio(ctx, drc_rounds, drop_violators, resume is not None, checkpoint)
    better = None
    if resume is not None:
        better = _key(ctx, lines, violations) < resume["key"]
        if not better:  # no gain: the earlier routing, exactly
            _adopt(ctx, resume)
            lines, violations, wire_net = dict(resume["lines"]), list(resume["violations"]), dict(resume["wire_net"])
    routing = _keep(ctx, lines, violations, wire_net, maps)  # (before corners are rounded: that is drawing, and done again)
    work = ctx.board_with_vias()  # the input board is not modified: the vias are pads of a copy
    loops = _redundant(ctx, {w: line for w, line in lines.items() if not ctx.conns[w].skeleton}, {p.pad_id for p in work.pads if p.is_via})
    lines, violations = _taut(ctx, lines, violations, loops)
    work.rules = drawn_board.rules
    lines, violations, wire_layer, vias = _drawn(ctx, work, lines, violations, loops)
    if events is not None:
        ctx.variant = -1  # the outcome of the pass, whichever variant it came from
        ctx.emit_snapshot(force=True, lines=lines)
        events({"type": "pass", "pass": 1, "open": len(ctx.open), "connections": len(pairs),
                "vias": sum(1 for p in work.pads if p.is_via)})
    t_route = time.perf_counter()

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
    pieces: dict[int, list] = {}
    wedges: dict[int, list] = {}
    if options.taper and narrowed:
        pieces, wedges = make_tapers(work, full_rules, lines, wire_net, wire_layer,
                                     {w: (ctx.conns[w].src, ctx.conns[w].dst) for w in wire_layer}, narrowed, fine)
    planed, unplaned, reliefs, kinds = [], [], {}, {}
    if options.fanout and work.pours:  # pads and pour pieces joined to their net, by the cheapest way (``plan.fanout``)
        # (the traces as drawn: tapered ones in their pieces, with their wedges)
        drawn = with_wedges(drops, wedges)
        shapes = [(Polygon(poly[:5]).buffer(0), wire_net[w], wire_layer[w]) for w, polys in drawn.items() if w in wire_layer for poly in polys]
        done = join_pours(work, lines, wire_net, wire_layer, shapes, drawn, pieces, options.reliefs, options.narrow_reliefs)
        planed, unplaned, reliefs, kinds = done.vias, done.left, done.reliefs, done.kinds
        work.pads = [replace(p, **reliefs[p.pad_id]) if p.pad_id in reliefs else p for p in work.pads] + planed
        vias = vias + planed
        w = max(list(ctx.conns) + list(lines), default=-1) + 1
        for net, layer_index, points in done.traces:
            lines[w], wire_net[w], wire_layer[w] = points, net, layer_index
            w += 1

    for layer in ctx.layers:
        assert layer.state.check_invariants(), "topological invariant broken"
    t_end = time.perf_counter()
    routed = sorted(w for w in wire_layer if w in ctx.conns)
    air = sum(ctx.conns[w].air_len for w in routed)
    length = sum(polyline_length(lines[w]) for w in routed)
    # A pad no trace can reach under the rules (packed closer than the clearance
    # allows, outside the board, inside a keepout) leaves its connections open too.
    unreachable = _unreachable(board, buried)
    total = len(pairs) + unreachable
    done = len(pairs) - len(ctx.open)
    stats = {
        "connections": total,
        "routed": done,
        "completion": done / total if total else 1.0,
        "wires": len(routed),
        "vias": len(vias),
        "length": length,
        "copper": sum(copper_length([lines[w] for w in routed if wire_layer[w] == layer.index]) for layer in ctx.layers),  # shared trunks once
        "airwire_length": air,
        "length_ratio": length / air if air else math.nan,
        "ripup_rounds": ctx.rounds,
        "corners_rounded": smoothed[0],
        "corners_left_sharp": smoothed[1],
        "via_sites": len(ctx.layers[0].pmap.sites),
        "buried_pads": buried,
        "narrowed": narrowed,  # net -> the width its traces were narrowed to, to reach its pads (``reach``)
        # Traces of those nets drawn at their own width but near the fine-pitch pads (``realize.taper``), and drawn narrow all along.
        "tapered": sum(1 for w in routed if wire_net.get(w) in narrowed and w in pieces),
        "narrow_all_along": sum(1 for w in routed if wire_net.get(w) in narrowed and w not in pieces),
        "fanout": len(planed),  # vias placed to join pads to their net's plane (``plan.fanout``)
        "plane_left": [p.pad_id for p in unplaned],  # pads of a plane net still not joined to the rest of it
        "joins": kinds,  # kind of join (``plan.fanout``) -> how many
        "reliefs": reliefs,  # pad id -> its thermal relief as changed ({Pad field: value}; ``Options.reliefs``, ``narrow_reliefs``)
        "unreachable": unreachable,  # connections of those pads, counted as open
        "skeletons": len(skeletons),  # connections of the pours' skeletons (``plan.skeleton``)
        "skeletons_routed": len(skeletons) - ctx.skeletons_open(),
        "wires_per_layer": {l.name: sum(1 for li in wire_layer.values() if li == l.index) for l in ctx.layers},
        "time_route": t_route - t0,
        "time_total": t_end - t0,
        "triangles": sum(l.pmap.num_triangles for l in ctx.layers),
        "edges": sum(l.pmap.num_edges for l in ctx.layers),
        "improved": better,  # None: routed from nothing; else whether going on from the earlier routing gained anything
    }
    return Result(work, ctx.layers, ctx.conns, lines, wire_net, wire_layer, violations, sorted(ctx.open), vias, drops, stats, widths=pieces, wedges=wedges, kept=routing)


def _drawn(ctx: Context, work: Board, lines: dict, violations: list, loops: set[int] | None = None) -> tuple[dict, list, dict, list[Pad]]:
    """What of a routing is drawn: the traces less those that join nothing
    new (``_redundant``, unless given as ``loops``) and pieces of no length,
    the violations left among them, each trace's layer, and the vias a trace
    ends on. ``work``: the board with the vias as pads
    (``Context.board_with_vias``). The pours' skeletons are not drawn: they
    are the fill's (``plan.skeleton``)."""
    lines = {w: line for w, line in lines.items() if not ctx.conns[w].skeleton}
    if loops is None:
        loops = _redundant(ctx, lines, {p.pad_id for p in work.pads if p.is_via})
    lines = {w: line for w, line in lines.items() if w not in loops and polyline_length(line) > 1e-9}  # (not a piece of no length)
    violations = [v for v in violations if not loops.intersection(v.wires) and not any(w in ctx.conns and ctx.conns[w].skeleton for w in v.wires)]
    wire_layer = {w: c.layer for w, c in ctx.conns.items() if c.layer is not None and w in lines}
    used = {p for w in wire_layer for p in (ctx.conns[w].src, ctx.conns[w].dst)}
    vias = [p for p in work.pads if p.is_via and p.pad_id in used]
    return lines, violations, wire_layer, vias


def _taut(ctx: Context, lines: dict, violations: list, loops: set[int]) -> tuple[dict, list]:
    """The traces pulled once more without the wires that will not be drawn
    (``loops``, from ``_redundant``): a trace is held away from a wire only if
    that wire is copper on the board (13.1). A layer keeps the new traces only
    if the check finds no more on it than before; the state is left as it was."""
    if not loops:
        return lines, violations
    board = ctx.board_with_vias()
    lines, violations = dict(lines), list(violations)
    skeleton = lambda v: any(w in ctx.conns and ctx.conns[w].skeleton for w in v.wires)
    for layer in ctx.layers:
        state = layer.state
        gone = [w for w in loops if w in state.wire_path]
        if not gone:
            continue
        snap = state.snapshot()
        for w in gone:
            state.remove(w)
        new, found, _ = realize(state, board)
        state.restore(snap)
        found = [v for v in found if not skeleton(v)]
        before = [v for v in violations if v.layer == layer.index and not loops.intersection(v.wires) and not skeleton(v)]
        if len(found) <= len(before):
            lines.update(new)
            violations = [v for v in violations if v.layer != layer.index or skeleton(v)] + found
    return lines, violations


def _draft(ctx: Context, lines: dict, violations: list, wire_net: dict) -> Result:
    """A routing as a ``Result`` that ``write_ses`` can write, as it comes out
    of a pass: no corners rounded, no teardrops, no vias down to the planes
    (those come after the race). For ``route_board``'s ``checkpoint``."""
    work = ctx.board_with_vias()
    lines, violations, wire_layer, vias = _drawn(ctx, work, lines, violations)
    total = sum(1 for c in ctx.conns.values() if c.parent is None and not c.skeleton)
    return Result(ctx.board, [], {}, lines, {w: wire_net[w] for w in lines if w in wire_net}, wire_layer, violations,
                  sorted(ctx.open), vias, stats={"connections": total, "routed": total - len(ctx.open), "vias": len(vias)})


def _layers(board: Board, workers: int | None) -> list[Layer]:
    return [Layer(i, name, pmap) for i, (name, pmap) in enumerate(zip(
        board.layers, parallel.run(_build_task, board, range(len(board.layers)), workers)))]


def reach(board: Board, layers: list[Layer], buried: list[int], workers: int | None) -> tuple[Board, list[Layer], dict[int, float]]:
    """Narrower traces for the nets of pads no trace reaches at the net's width
    (design 4.1): a fine-pitch part whose pins are closer than its class's
    width and clearance allow, which a designer routes by hand with thinner
    traces. Such a net is narrowed halfway down to the narrowest trace the
    board's rules allow (``Rules.min_width``; 0, not known: nothing is
    narrowed) if that reaches all its pads, else down to it, else routed at
    that less half the safety margin and drawn at it (where the pins leave
    exactly the clearance, the margin is all that is in the way). A net none
    of that helps keeps its width. Returns the board (a copy with rules of its
    own, if any net was narrowed), its layers and the nets narrowed, with the
    width they are drawn at."""
    rules = board.rules
    floor = rules.min_width
    if not floor:
        return board, layers, {}
    net_of = {p.pad_id: p.net_id for p in board.pads}
    left = {net_of[p] for p in buried if net_of[p] >= 0}
    steps = [lambda n: ((rules.width(n) + floor) / 2.0, None), lambda n: (floor, None),
             lambda n: (floor - rules.margin, floor)]
    trial = copy.copy(board)
    trial.rules = copy.copy(rules)
    trial.rules.net_width, trial.rules.net_drawn = dict(rules.net_width), dict(rules.net_drawn)
    trial.rules.net_full, trial.rules.fine = dict(rules.net_full), dict(rules.fine)
    pads = {p.pad_id: p for p in board.pads}
    done: dict[int, tuple[float, float | None]] = {}
    built = None
    for step in steps:
        tried = {n: step(n) for n in left if step(n)[0] < rules.width(n) - 1e-9 and step(n)[0] > 0}
        if not tried:
            continue
        for n, (width, drawn) in tried.items():
            trial.rules.net_width[n] = width
            if drawn is not None:
                trial.rules.net_drawn[n] = drawn
            # Planned at its own width but near the pads it is narrowed for: as far as
            # ``realize.taper`` draws it narrow there (the pad, a stretch and a wedge).
            wide, narrow = rules.width(n), drawn or width
            trial.rules.net_full[n] = wide
            trial.rules.fine[n] = [(*pads[p].centre, _pad_reach(pads[p]) + wide + max(TAPER_WEDGE * (wide - narrow), wide))
                                   for p in buried if net_of[p] == n]
        built = _layers(trial, workers)
        still = {net_of[p] for p in decompose(trial, built)[1]}
        done.update({n: w for n, w in tried.items() if n not in still})
        left -= set(done)
        if not left:
            break
    if not done:
        return board, layers, {}
    if left:  # narrowed in vain: as they were, and the maps again without them
        for n in left:
            trial.rules.net_width.pop(n, None)
            trial.rules.net_drawn.pop(n, None)
            trial.rules.net_full.pop(n, None)
            trial.rules.fine.pop(n, None)
            if n in rules.net_width:
                trial.rules.net_width[n] = rules.net_width[n]
            if n in rules.net_drawn:
                trial.rules.net_drawn[n] = rules.net_drawn[n]
        built = _layers(trial, workers)
    return trial, built, {n: trial.rules.drawn(n) for n in done}


def _pad_reach(pad: Pad) -> float:
    """How far a pad reaches from its centre (as ``realize.taper`` measures it)."""
    return max(math.dist(pad.centre, p) for p in pad.shape.convex_hull.exterior.coords)


def _unreachable(board: Board, buried: list[int]) -> int:
    """How many of the connections a net needs (one fewer than its pads) are
    lost with the pads ``buried``: those can be made only among the others."""
    lost = set(buried)
    count = 0
    for pads in board.nets().values():
        gone = sum(1 for p in pads if p in lost)
        if gone:
            count += (len(pads) - 1) - max(len(pads) - gone - 1, 0)
    return count


def _redundant(ctx: Context, lines: dict, vias: set[int]) -> set[int]:
    """Wires that join nothing their net's other wires do not join already (section 11).

    A connection may go through pads and vias its net has, and beside its
    net's copper it costs next to nothing, so a piece of it can run from pad
    to pad beside a wire that joins those two already, or a whole connection
    can join two pads other connections have joined. In the state each is a
    wire of its own (nothing depends on anything); on the board it is a
    second trace. Per net, the shortest wires that join its pads are kept (a
    spanning forest of pads and vias, wires by drawn length); the rest, and
    wires to a via nothing else uses, are redundant."""
    by_net: dict[int, list[int]] = {}
    for w, c in ctx.conns.items():
        if c.layer is not None and w in lines:
            by_net.setdefault(c.net_id, []).append(w)
    out: set[int] = set()
    for wires in by_net.values():
        parent: dict[int, int] = {}

        def root(p: int) -> int:
            while parent.setdefault(p, p) != p:
                parent[p] = p = parent[parent[p]]
            return p

        kept = []
        for w in sorted(wires, key=lambda w: (polyline_length(lines[w]), w)):
            a, b = root(ctx.conns[w].src), root(ctx.conns[w].dst)
            if a == b:
                out.add(w)
            else:
                parent[a] = b
                kept.append(w)
        while True:  # a via one wire ends on is that wire's dead end
            ends: dict[int, list[int]] = {}
            for w in kept:
                for p in (ctx.conns[w].src, ctx.conns[w].dst):
                    ends.setdefault(p, []).append(w)
            stubs = {ws[0] for p, ws in ends.items() if p in vias and len(ws) == 1}
            if not stubs:
                break
            out |= stubs
            kept = [w for w in kept if w not in stubs]
    return out


def _print(pmap: PlanarMap) -> str:
    """Names a map as it was built: a routing can only be gone on from on the same one."""
    import hashlib
    digest = hashlib.sha1()
    for table in (pmap.vx, pmap.vy, pmap.tri_v, pmap.edge_kind):
        digest.update(table.tobytes())
    return digest.hexdigest()


def _key(ctx: Context, lines: dict, violations: list) -> tuple:
    """How good a routing is, least best: connections open, violations, the pours' skeletons
    open, copper (a shared trunk once; the skeletons' is not copper)."""
    copper = sum(copper_length([line for w, line in lines.items() if ctx.conns[w].layer == layer.index and not ctx.conns[w].skeleton])
                 for layer in ctx.layers)
    return (len(ctx.open), len(violations), ctx.skeletons_open(), round(copper, 3))


def _keep(ctx: Context, lines: dict, violations: list, wire_net: dict, maps: list | None = None) -> dict:
    """The routing as it stands, to be taken up again on another copy of the
    maps (``_adopt``): by the parent of a raced variant, or by a later run."""
    return {"key": _key(ctx, lines, violations), "snapshot": ctx.snapshot(),
            "caps": [layer.state.cap.copy() for layer in ctx.layers], "hists": [layer.state.hist.copy() for layer in ctx.layers],
            "pres": ctx.params.pres_fac, "flags": {w: (c.fails, c.dead) for w, c in ctx.conns.items()}, "rounds": ctx.rounds,
            "lines": lines, "violations": violations, "wire_net": wire_net,
            "logs": [list(sites.log(layer.pmap)) for layer in ctx.layers], "maps": maps}


def _adopt(ctx: Context, kept: dict) -> None:
    """Takes up a routing kept by ``_keep``: its via sites are made again on this copy of the maps."""
    ctx.restore(kept["snapshot"], kept["logs"])
    for layer, cap, hist in zip(ctx.layers, kept["caps"], kept["hists"]):
        layer.state.cap, layer.state.hist = cap.copy(), hist.copy()
    ctx.params.pres_fac = kept["pres"]
    ctx.rounds = kept["rounds"]
    for w, (fails, dead) in kept["flags"].items():
        if w in ctx.conns:
            ctx.conns[w].fails, ctx.conns[w].dead = fails, dead


# Settings of the raced variants: (search weighting, growth of the price of an over-full gate per round).
# Variant 0 is the plain configuration. No two are alike: the seed by itself changes nothing
# (measured, 22), so a variant that repeats another's settings repeats its routing.
VARIANTS = ((1.0, 1.5), (1.5, 1.5), (1.25, 1.5), (1.75, 1.5), (1.0, 1.8), (2.0, 1.5), (1.5, 1.8), (1.25, 1.8))
HEAT = 4
SETTLE_AGAIN = 3  # times the routing is settled after repair before it is left as repair had it


def _route_portfolio(ctx: Context, drc_rounds: int, drop_violators: bool, resumed: bool = False, checkpoint=None):
    """One routing pass, raced over several variants (section 22).

    How a pass ends depends strongly on small differences early on, so the
    spare cores each run the whole pass with a different search weighting
    and price growth. The first variant, in their fixed order, that connects
    everything is taken (the plain configuration is first, so nothing is lost
    when it succeeds); if none does, the best is kept: fewest open connections,
    then shortest. The choice does not depend on which finishes first.
    Past the time limit (``Context.late``) no further heat is started.
    ``checkpoint``: as ``route_board``'s, called as each variant finishes.
    """
    count = ctx.options.portfolio
    if count <= 0:
        workers = parallel.cpu_count() if ctx.workers is None else ctx.workers
        count = max(1, min(len(VARIANTS), workers))
    if count == 1 or parallel._inside:
        lines, violations, wire_net = _route_once(ctx, drc_rounds, drop_violators, resumed)
        if checkpoint is not None:
            checkpoint(_key(ctx, lines, violations), _draft(ctx, lines, violations, wire_net))
        return lines, violations, wire_net
    given = ctx
    if not parallel.can_fork():
        # Each variant gets a copy (``parallel.first_accepted``): of what reports
        # progress, only what writes to a pipe still reaches anyone from there.
        given = copy.copy(ctx)
        given.progress = None
        given.events = ctx.events if parallel.travels(ctx.events) else None
    # Raced in heats of four: more at once than that only slows each other
    # down (shared caches, efficiency cores), and the plain variant is in the
    # first heat, so an easy board is not held up by the rest.
    complete = lambda outcome: outcome[0][0] == 0 and outcome[0][1] == 0
    outcomes: list = []
    # Each variant's outcome is handed to ``checkpoint`` here, as it finishes, whichever finishes first.
    arrived = (lambda outcome: outcome is not None and checkpoint(outcome[0], outcome[2])) if checkpoint is not None else None
    for start in range(0, count, HEAT):
        if outcomes and ctx.late():
            break  # the time limit is up: the best of the variants that have run
        heat = range(start, min(count, start + HEAT))
        ctx.report("racing variants %d-%d of %d" % (heat[0] + 1, heat[-1] + 1, count))
        stop = parallel.stop_flag()
        found = parallel.first_accepted(_variant_task, (given, drc_rounds, drop_violators, stop, resumed, checkpoint is not None), heat,
                                        lambda outcome: outcome is not None and complete(outcome), len(heat), stop, arrived)
        outcomes += [o for o in found if o is not None]
        if outcomes and complete(outcomes[-1]):
            break
    best = len(outcomes) - 1 if outcomes[-1][0][:2] == (0, 0) else min(range(len(outcomes)), key=lambda i: (outcomes[i][0], i))
    kept = outcomes[best][1]
    _adopt(ctx, kept)
    ctx.report("variant %d of %d kept" % (best + 1, count), 1.0, 1.0)
    return kept["lines"], kept["violations"], kept["wire_net"]


def _variant_task(shared, index: int):
    ctx, drc_rounds, drop_violators, stop, resumed, draft = shared
    ctx.stop = stop
    ctx.seed += index
    ctx.variant = index
    ctx.params.h_weight, ctx.params.pres_growth = VARIANTS[index % len(VARIANTS)]
    if index:
        ctx.progress = None  # the console bar follows the plain variant; live events come from all of them
    try:
        lines, violations, wire_net = _route_once(ctx, drc_rounds, drop_violators, resumed)
    except parallel.Stopped:
        return None  # an earlier variant already connected everything
    kept = _keep(ctx, lines, violations, wire_net)
    return (kept["key"], kept, _draft(ctx, lines, violations, wire_net) if draft else None)


def _route_once(ctx: Context, drc_rounds: int, drop_violators: bool, resumed: bool = False):
    """Phases 1 - 4 and realisation on a fixed set of pads and connections.
    ``resumed``: the context holds a routing already (``_adopt``); Phases 1
    and 2 are not run, and everything from Phase 3 on goes on from it."""
    opts = ctx.options
    if not resumed:
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
        if ctx.later:  # (the skeletons wait till the traces are settled: ``ripup.settle``)
            for conn in ctx.conns.values():
                conn.dead = conn.dead or conn.skeleton
    else:
        for conn in ctx.conns.values():  # what could not be done then may be possible now: try everything again
            conn.fails, conn.dead = 0, False
    # Phase 3: negotiated rip-up and reroute. (Going on from a routing: as many rounds again, at the most.)
    if opts.ripup:
        negotiate(ctx, ctx.rounds + ctx.params.max_rounds if resumed else None)
        if not ctx.late() and _narrow_again(ctx):
            negotiate(ctx, ctx.rounds + ctx.params.max_rounds)
    # Phase 4: topology refinement, what still fits, and vias where a trace would do.
    settle(ctx, vias=True)

    # Realisation, with DRC feedback into Phase 3 (13.2).
    ctx.report("geometry")
    lines, violations, wire_net = _realize(ctx)
    for _ in range(2):  # skeletons beside a violation give way first: often that is all it needs, without repair
        if not violations or not _crowding(ctx, violations):
            break
        lines, violations, wire_net = _realize(ctx)
    ctx.emit_snapshot(force=True, lines=lines)
    disturbed = False  # whether anything has moved since the routing was settled
    for fix in range(drc_rounds):
        if not violations or ctx.late():  # (past the time limit what violates is dropped, below, without repair)
            break
        if not _penalise(ctx, violations):
            break
        ctx.report("design-rule repair", fix, drc_rounds)
        disturbed = True
        if opts.refine and not ctx.late():  # the skeletons wait out the repair: settling puts them back (``ripup.settle``)
            for conn in [c for c in ctx.conns.values() if c.skeleton and c.parent is None]:
                if conn.routed:
                    ctx.rip(conn.wire_id)
                conn.dead = True
            ctx.rebuild_barrier()
        if opts.ripup and not ctx.late():
            negotiate(ctx, max_rounds=ctx.rounds + 5)
        legalise(ctx)
        lines, violations, wire_net = _realize(ctx)
    # Whatever still violates is not a routed connection: drop it.
    while violations and drop_violators:
        dropped: set[int] = set()
        for v in violations:
            if not dropped.intersection(v.wires):
                dropped.add(max(v.wires, key=lambda w: (w in ctx.conns and ctx.conns[w].skeleton, w)))  # (a pour's skeleton gives way first)
        for w in dropped:
            if w in ctx.conns:  # not a piece of a connection ripped up a moment ago
                ctx.rip(w)
        disturbed = True
        lines, violations, wire_net = _realize(ctx)
    if opts.refine and disturbed and not ctx.late():
        # Repair has moved things since the routing was settled: it may have
        # left room for a connection that is open, or a via where a trace
        # would now do. Settle once more, and keep that if the check agrees.
        before, hold = ctx.snapshot(), set()
        for _ in range(SETTLE_AGAIN):
            settle(ctx, hold, vias=True)
            again = _realize(ctx)
            if len(again[1]) <= len(violations):
                lines, violations, wire_net = again
                break
            # The check objects: once more from where it stood, leaving alone
            # the connections whose traces it names.
            hold |= {ctx.conns[w].parent or w for v in again[1] for w in v.wires if w in ctx.conns}
            ctx.restore(before)
    if opts.vias and opts.slide and not ctx.late():
        ctx.report("straightening vias")
        lines, violations, wire_net = slide(ctx, lines, violations, wire_net, _realize)
    ctx.emit_snapshot(force=True, lines=lines)
    ctx.report("finished", complete=not ctx.open and not violations)
    return lines, violations, wire_net


def _narrow_again(ctx: Context) -> bool:
    """A net narrowed to reach fine-pitch pads is planned at its own width away
    from them (``reach``). Where that leaves a connection open, the nets planned
    so that are its own or in its way are planned narrow all along again, as
    they were before (``Context.demote``), and every open connection may be
    tried again. Returns whether any was."""
    wide = {n for layer in ctx.layers for n, z in layer.state.zone.items() if z[0] is not None}
    if not wide or not ctx.open:
        return False
    nets = set()
    for w in sorted(ctx.open):
        conn = ctx.conns[w]
        if conn.net_id in wide:
            nets.add(conn.net_id)
            continue
        r = find(ctx, conn, mode="relaxed")
        if r is not None:
            nets |= {ctx.conns[x].net_id for x in r.blocking if x in ctx.conns} & wide
    if not nets:
        return False
    ctx.demote(nets)
    for w in ctx.open:
        ctx.conns[w].fails, ctx.conns[w].dead = 0, False
    return True


def _realize(ctx: Context):
    """Geometry of every layer. Layers are independent, so each gets a worker."""
    lines, violations, wire_net = {}, [], {}
    for layer, (changed, snap, l, v, n) in zip(ctx.layers, parallel.run(_realize_task, ctx, range(len(ctx.layers)), ctx.workers)):
        if snap is not None:  # computed in a worker: bring its straightened wire ends over
            layer.state.restore(snap)
        for w in changed:
            layer.paths[w] = path_from_steps(layer.pmap, layer.state.wire_path[w])
        lines.update(l)
        # (a pour's skeleton is not copper: what comes too near it is not a violation, the fill there is only narrower)
        violations += [x for x in v if not any(w in ctx.conns and ctx.conns[w].skeleton for w in x.wires)]
        wire_net.update(n)
    return lines, violations, wire_net


def _realize_task(ctx: Context, index: int):
    layer = ctx.layers[index]
    board = ctx.board_with_vias()
    # Let each trace leave its pad through the edge it is heading for (M8).
    changed = straighten(layer.state, board)
    lines, violations, wire_net = realize(layer.state, board)
    snap = layer.state.snapshot() if changed and parallel._inside else None
    return changed, snap, lines, violations, wire_net


def _build_task(board: Board, index: int) -> PlanarMap:
    return planar_map.build(board, index)


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


def _crowding(ctx: Context, violations) -> bool:
    """Takes out the pours' skeletons on a gate that a trace in a violation
    passes (``plan.skeleton``): a skeleton is wider than a trace, and where the
    room estimated for it is not there in the geometry, the traces beside it
    are pushed into each other or into a pad. Returns whether any was."""
    out = set()
    for v in violations:
        layer = ctx.layers[v.layer]
        for w in v.wires:
            if w in layer.paths:
                for g in layer.paths[w].gates:
                    out |= {ctx.conns[x].parent or x for x in layer.state.gate_order[g] if ctx.conns[x].skeleton}
    for w in sorted(out):
        ctx.rip(w)
    if out:
        ctx.rebuild_barrier()
    return bool(out)


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
