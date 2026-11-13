"""Pads and pour pieces joined to the rest of their net, by the cheapest way (design 11.2, 11.4).

A net with a copper plane (a pour, in KiCad a zone) is not routed as traces:
the plane joins its pads when the CAD tool fills it. A pad the plane does not
reach stays unconnected: the plane is on another layer (an inner ground plane,
the ground pour on the back of a two-layer board) or stops short of it, or the
routing has walled off the piece of the pour it is in, or starved its thermal
spokes. Once the board is routed, each group of such pads (with the pieces of
pour they are in) is joined to the rest of its net by the cheapest of these
that the fill estimate (``pours``) finds joins it and that leaves every other
pad as joined as it was:

- its thermal relief changed (solid, or the spokes turned), or its relief gap
  and spokes narrowed: no copper added, but it departs from the zone's rules,
  so only when asked for (``reliefs``, ``narrow``);
- a via in the pad, or in the cut-off piece (stitching it to the net's fill on
  another layer), or beside the pad joined by a short trace;
- a trace from the pad, or from the piece over a gap, into the net's fill, or
  to a pad or via of the net that is joined;
- a jumper: a via in the piece, a trace on another layer, and a via (or the
  fill there) at its other end.

Each is priced by its length, its vias, the room its copper takes with its
clearance, and how far its end reaches into the fill (an end at the fill's
edge is less sure to be reached by KiCad's fill). The cheapest are checked
against the estimate, made again where the join changes it (the net's own fill
with the new copper in it; other nets' pours near it filled again), and the
first that joins the group and cuts nothing off is kept. Done after routing, it
takes no room a trace needed; what is left is named. Fixed copper of the net
counts as the net's: a pad a locked trace already joins to a locked via in the
plane needs nothing, and a locked via joined to the plane may end a trace.
"""
import copy
import dataclasses
import math
from dataclasses import dataclass, field

import numpy as np
import shapely
import shapely.ops
from shapely.geometry import LineString, Point, Polygon

from weaveengine.board import Board, Pad, circle_polygon

DIRECTIONS = 32    # rays round a pad, or a via in a cut-off piece, a join is looked for along
FAR = 4.0          # mm: the longest trace of a join (beyond the pad)
STEP = 0.05        # mm between the places tried along a ray
GAP = 0.05         # mm between a pad and a via beside it, and a trace's end into the fill past its half width
HOLES = 0.25       # mm a via keeps from a through-hole pad or fixed via of its own net (drill to drill: KiCad's default minimum)
LANDING = 0.02     # mm a via may stand nearer the edge of the fill than its radius (its pad is no bigger than its copper, and KiCad's fill gives it a little room)
FILL = 0.25        # mm: a pour's fill is not narrower than this (KiCad's default zone minimum width)
# What a join costs, in millimetres of trace:
VIA = 2.0          # a via
IN_PAD = 1.0       # more, for a via in a pad (solder runs down it)
RELIEF = 0.5       # a pad's thermal relief changed (no copper, but not as the zone asks)
NARROWED = 1.0     # a pad's relief gap and spokes narrowed
ROOM = 0.5         # each mm2 of a layer its copper takes, with the clearance round it
SURE = 0.25        # mm: an end this far into the fill (past a via's radius, or past where a trace may end) is sure to be reached
UNSURE = 4.0       # each mm an end falls short of that
TRIES = 8          # joins checked against the estimate for one group, cheapest first
SAMPLES = 12       # places in a cut-off piece a via (a jumper's, or a bridge's start) is tried at
KEPT = 6           # joins of each kind to each group kept from one place, the cheapest, ending ``APART``
APART = 1.0        # mm
STARVED = 4.0      # mm round a join within which a pad's spokes may change, or fill round it that met nothing else (``_starved``)
INSET = 0.03       # mm a place is taken inside a region where copper may stand (its edge is drawn with polygons: the check is exact)


@dataclass
class Join:
    cost: float
    kind: str                                      # "relief", "narrowed", "via in pad", "stitch", "via", "trace", "jumper", "spoke"
    vias: list = field(default_factory=list)       # (x, y) of each via it adds
    traces: list = field(default_factory=list)     # (layer, [(x, y), (x, y)]) of each trace it adds
    relief: dict = field(default_factory=dict)     # pad id -> {Pad field: value}: its thermal relief as changed


@dataclass
class Joined:
    vias: list = field(default_factory=list)       # Pads: the vias added (``Pad.fanout``)
    traces: list = field(default_factory=list)     # (net, layer, points) of each trace added
    left: list = field(default_factory=list)       # Pads: those still not joined to the rest of their net
    reliefs: dict = field(default_factory=dict)    # pad id -> {Pad field: value}: thermal reliefs changed (``reliefs``, ``narrow``)
    kinds: dict = field(default_factory=dict)      # kind of join -> how many were made


def covered(board: Board, pad: Pad, cut=frozenset()) -> bool:
    """Whether a plane of the pad's net reaches the pad on a layer it has copper on
    (not one where it is in a piece ``cut`` off from the rest: (pad, layer) pairs;
    nor one whose zone does not join it, its connection being "none": ``pours.joined``)."""
    from weaveengine.pours import joined
    centre = Point(pad.centre)
    return any(net == pad.net_id and pad.on(layer) and (pad.pad_id, layer) not in cut and shape.contains(centre)
               and joined(pad, board.zone(net, layer)) != "none" for net, layer, shape in board.pours)


def unjoined(board: Board, lines: dict | None = None, wire_net: dict | None = None, wire_layer: dict | None = None,
             teardrops=None, widths: dict | None = None) -> list[Pad]:
    """The pads of plane nets not joined to the rest of their net (``pours.reach``), with the routing ``lines``."""
    from weaveengine import pours as estimates
    wires = estimates.copper_of(board, lines or {}, wire_net or {}, wire_layer or {}, teardrops, widths)
    fill_of, _ = estimates.fills(board, {}, {}, {}, wires=wires)
    return [p for net in sorted(board.plane_nets)
            for p in _reach(board, estimates.classify(board, _of(board, fill_of, {net}), wires), wires, net)[1]]


def fanout(board: Board, lines: dict, wire_net: dict, wire_layer: dict, shapes=(), teardrops=None,
           widths: dict | None = None) -> tuple[list[Pad], list[tuple[int, int, list]], list[Pad]]:
    """``join`` with the zones' rules as they are: the vias added, the traces as (net, layer, points), the pads left."""
    done = join(board, lines, wire_net, wire_layer, shapes, teardrops, widths)
    return done.vias, done.traces, done.left


def join(board: Board, lines: dict, wire_net: dict, wire_layer: dict, shapes=(), teardrops=None, widths: dict | None = None,
         reliefs: bool = False, narrow: bool = False) -> Joined:
    """For the routed ``board`` (its vias among its pads), its traces and other copper ``shapes`` (shape, net,
    layer: teardrops; and the same as ``teardrops``, wire -> polygons, for the fill): the pads of plane nets not
    joined to the rest of their net, joined by the cheapest way (see the module). ``widths``: wire -> [(points,
    width)] for traces drawn in pieces (``realize.taper``). ``reliefs``: a pad's thermal relief may be made solid or
    its spokes turned; ``narrow``: its relief gap narrowed to the zone's clearance and its spokes to the zone's
    least width."""
    from weaveengine import pours as estimates
    rules = board.rules
    out = Joined()
    nets = sorted(net for net in board.plane_nets if any(p.net_id == net and not p.is_via for p in board.pads))
    if not nets:
        return out
    work = copy.copy(board)
    work.pads = list(board.pads)
    wires = estimates.copper_of(board, lines, wire_net, wire_layer, teardrops, widths)
    fill_of, _ = estimates.fills(work, lines, wire_net, wire_layer, wires=wires)
    reach = {net: _reach(work, estimates.classify(work, _of(work, fill_of, {net}), wires), wires, net) for net in nets}
    if not any(lost for _, lost, _ in reach.values()):
        return out
    r = rules.via_diameter / 2.0
    keep = rules.apart(rules.via_clearance)  # what a via keeps from foreign copper
    edge = (rules.clearance if rules.edge_clearance is None else rules.edge_clearance)
    inside_via = board.outline.buffer(-(edge + r))
    margin = shapely.union_all([m.buffer(edge) for m in board.margin]) if board.margin else None  # (KiCad holds copper off these too)
    no_via = shapely.union_all(board.no_via) if board.no_via else None  # (areas no via may touch)
    nlayers = len(board.layers)
    every = nlayers + len(board.unrouted)  # (a via goes through the layers left out of the routing too, and lands in their planes)
    area = estimates.pour_area(board)
    # Everything a via or a trace must keep clear of, per layer: (shape, net, what it asks; -2 = keepout).
    things: list[list] = [[] for _ in range(every)]
    for p in board.pads:
        for i in range(every):
            if p.on(i):
                things[i].append((p.shape, p.net_id, keep if p.is_via else rules.apart(p.clearance)))
    for o in board.obstacles:
        for i in range(every):
            if o.on(i):
                # (copper drawn on a part is of no net to KiCad, even where it touches a pad of this one)
                things[i].append((o.shape, -1 if o.copper else o.net_id if o.net_id >= 0 else -2, rules.apart(o.clearance)))
    for w, line in lines.items():
        if len(line) >= 2 and w in wire_layer:
            net = wire_net[w]
            for piece, width in (widths or {}).get(w) or [(line, rules.drawn(net))]:
                if len(piece) >= 2:
                    things[wire_layer[w]].append((LineString(piece).buffer(width / 2.0, quad_segs=4), net, rules.clearance))
    for shape, net, layer in shapes:
        things[layer].append((shape, net, rules.clearance))
    through = {p.shape for p in board.pads if not p.is_via and (p.layers is None or len(p.layers) == nlayers > 1)}
    through |= {o.shape for o in board.obstacles if o.via}
    trees = [shapely.STRtree([t[0] for t in ts]) if ts else None for ts in things]
    widest = max([keep, rules.clearance] + [t[2] for ts in things for t in ts])
    within = board.outline.buffer(-edge)

    def clear(shape, net: int, layer: int, need: float, drilled: bool = False) -> bool:
        """Whether ``shape`` (copper of ``net``) keeps ``need`` from everything foreign on ``layer``,
        and, if ``drilled`` (a via), ``HOLES`` from every through-hole pad and fixed via."""
        tree = trees[layer]
        if tree is None:
            return True
        for i in tree.query(shape, predicate="dwithin", distance=max(widest + need, HOLES)).tolist():
            other, other_net, asks = things[layer][i]
            if other_net == net:
                if drilled and other in through and shape.distance(other) < HOLES - 1e-9:
                    return False  # (drill to drill, even within the net)
                continue
            if other_net == -2:
                if shape.intersects(other):
                    return False
            elif shape.distance(other) < max(need, asks) - 1e-9:
                return False
        return True

    def trace_ok(net: int, layer: int, a, b, pad=None) -> bool:
        """Whether a trace of ``net`` from ``a`` to ``b`` on ``layer`` keeps its clearances (from ``pad``, if it starts in one)."""
        stub = LineString([a, b]).buffer(rules.drawn(net) / 2.0, quad_segs=4)
        if margin is not None and (stub if pad is None else stub.difference(pad.shape)).intersects(margin):
            return False
        return within.contains(stub) and clear(stub, net, layer, rules.clearance)

    seen: dict = {}  # id of a via added -> (it, its centre, its net)

    def placed() -> list:
        """(centre, net) of each via added (its centre is measured once)."""
        for v in out.vias:
            if seen.get(id(v), (None,))[0] is not v:
                seen[id(v)] = (v, v.centre, v.net_id)
        return [seen[id(v)][1:] for v in out.vias]

    def via_ok(net: int, x: float, y: float) -> bool:
        """Whether a via of ``net`` may stand at (x, y) among everything there and the vias added."""
        if not inside_via.contains(Point(x, y)):
            return False
        disc = circle_polygon(x, y, r)
        if margin is not None and disc.intersects(margin):
            return False
        if no_via is not None and disc.intersects(no_via):
            return False
        # (two vias of the net may touch, but their holes keep ``HOLES`` apart: KiCad's hole to hole)
        if any(math.dist((x, y), at) < max(2 * r + (0.0 if other == net else keep), rules.via_drill + HOLES) - 1e-9 for at, other in placed()):
            return False
        return all(clear(disc, net, i, keep, drilled=True) for i in range(every))

    def free(net: int, window, grow: float, need: float, layers) -> object:
        """Where the middle of copper of ``net`` reaching ``grow`` round it keeps ``need`` (or what each thing asks, if
        more) from every foreign thing on ``layers``, within ``window`` (a keepout: not touched at all)."""
        taken = []
        for i in layers:
            if trees[i] is None:
                continue
            for k in trees[i].query(window, predicate="dwithin", distance=widest + need + grow).tolist():
                other, other_net, asks = things[i][k]
                if other_net != net:
                    key = (i, k, grow, need)
                    if key not in grown:
                        grown[key] = other.buffer(grow + (0.0 if other_net == -2 else max(need, asks)), quad_segs=4)
                    taken.append(grown[key])
        return window.difference(shapely.union_all(taken)) if taken else window

    grown: dict = {}  # (layer, thing, grow, need) -> the thing grown so (``free``; things are only added to)

    state = {"work": work, "fill_of": fill_of, "wires": wires, "reach": reach}

    def attempt(net: int, unit: list, deep: bool) -> float | None:
        """Joins the group of ``unit`` (pads of ``net``) to another group of its net that holds pads, or to its main
        group, by the cheapest way the estimate finds does so and splits nothing (``deep``: or to a group with fill
        and no pads, a plane joined to nothing yet, if a second join then takes that on to one of those, and the two
        cost less). Returns what the join cost, or None if none was made."""
        work, fill_of, wires, reach = state["work"], state["fill_of"], state["wires"], state["reach"]
        half = rules.drawn(net) / 2.0
        comp = reach[net][0]
        root = comp[("pad", unit[0].pad_id)]
        now = estimates.classify(work, _of(work, fill_of, {net}), wires)
        pieces = [(pour.layer, x.shape) for pour in now for x in pour.pieces if comp.get(estimates.piece_key(pour, x)) == root]
        # Everything near enough to matter: the group and its pieces, and as far round them as a join reaches.
        window = shapely.box(*shapely.union_all([p.shape for p in unit] + [s for _, s in pieces]).bounds).buffer(FAR + r + 1.0, join_style="mitre")
        target: dict = {}  # (layer, group) -> the fill of each other group of the net there (within the window)
        for pour in now:
            for x in pour.pieces:
                group = comp.get(estimates.piece_key(pour, x))
                if group is not None and group != root and x.shape.intersects(window):
                    key = (pour.layer, group)
                    target[key] = shapely.union_all([x.shape.intersection(window)] + ([target[key]] if key in target else []))
        ends = [(p, p.centre, comp[("pad", p.pad_id)]) for p in work.pads if p.net_id == net
                and comp.get(("pad", p.pad_id)) != root and p.shape.intersects(window)]
        ends += [(o, (o.shape.centroid.x, o.shape.centroid.y), comp[("fixed", id(o))]) for o in board.obstacles
                 if o.via and o.net_id == net and comp.get(("fixed", id(o))) not in (None, root) and o.shape.intersects(window)]
        tfree = {}
        for i in range(nlayers):
            g = free(net, window, half, rules.clearance, [i]).intersection(within.buffer(-half).intersection(window))
            tfree[i] = g.difference(margin.buffer(half)) if margin is not None else g
        vfree = free(net, window, r, keep, range(every)).intersection(inside_via.intersection(window))
        if no_via is not None:
            vfree = vfree.difference(no_via.buffer(r))
        if margin is not None:
            vfree = vfree.difference(margin.buffer(r))
        land = {key: t.buffer(-(r - LANDING)) for key, t in target.items()}
        into = {key: t.buffer(-(half + GAP)) for key, t in target.items() if key[0] < nlayers}
        ways = _candidates(work, net, unit, pieces, target, land, into, ends, tfree, vfree, r, keep, half, nlayers,
                           reliefs, narrow, trace_ok, via_ok)
        before = _groups(work, reach[net][0], net)
        direct, through = None, []  # the cheapest join that does it; cheaper ones to a plane joined to nothing
        for way in sorted(ways, key=lambda j: j.cost)[:TRIES]:
            done = _apply(work, way, net, fill_of, wires, reach, area, r, half, now, out.traces)
            if done is None:
                continue
            after = _groups(done[0], done[3][net][0], net)
            if not _coarser(before, after):
                continue
            # (fewer groups with pads; or this one now in the main group: a plane no pad was joined to is the main one)
            if len(set(after.values())) < len(set(before.values())) or after[unit[0].pad_id] == done[3][net][2]:
                direct = (way, done)
                break
            if deep and _grew(done[3][net][0], reach[net][0], root, after[unit[0].pad_id]):
                through.append((way, done))
        for way, done in through:  # (each, and then a second join from that plane, if the two cost less)
            kept = snapshot()
            make(net, way, done)
            more = attempt(net, unit, False)
            if more is not None and (direct is None or way.cost + more < direct[0].cost):
                return way.cost + more
            restore(kept)
        if direct is None:
            return None
        make(net, *direct)
        return direct[0].cost

    def snapshot():
        return (len(out.vias), len(out.traces), dict(out.reliefs), dict(out.kinds), [len(ts) for ts in things], trees, dict(state))

    def restore(kept) -> None:
        nonlocal trees
        del out.vias[kept[0]:], out.traces[kept[1]:]
        out.reliefs, out.kinds = kept[2], kept[3]
        for ts, n in zip(things, kept[4]):
            del ts[n:]
        trees = kept[5]
        state.update(kept[6])

    def make(net: int, way: Join, done) -> None:
        """Keeps ``way`` (made on ``done``, ``_apply``'s)."""
        nonlocal trees
        half = rules.drawn(net) / 2.0
        _keep(done[0], way, net, out)
        state.update(work=done[0], fill_of=done[1], wires=done[2], reach=done[3])
        for via in out.vias[len(out.vias) - len(way.vias):]:
            for i in range(every):
                things[i].append((via.shape, net, keep))
        for layer, pts in way.traces:
            things[layer].append((LineString(pts).buffer(half, quad_segs=4), net, rules.clearance))
        trees = [shapely.STRtree([t[0] for t in ts]) if ts else None for ts in things]
        out.kinds[way.kind] = out.kinds.get(way.kind, 0) + 1

    tried: set = set()
    for net in nets:
        while True:
            comp, lost, main = state["reach"][net]
            if main is None:
                break  # (no fill at all: nothing to join to)
            groups: dict = {}
            for p in lost:
                groups.setdefault(comp[("pad", p.pad_id)], []).append(p)
            todo = sorted((g for g in groups.values() if frozenset(p.pad_id for p in g) not in tried),
                          key=lambda g: (-len(g), min(p.pad_id for p in g)))
            if not todo:
                break
            tried.add(frozenset(p.pad_id for p in todo[0]))
            attempt(net, todo[0], True)
    def spoke(net: int, pad: Pad, layer: int, outline) -> None:
        """Gives ``pad``, which KiCad's check would find starved of thermal spokes in its pour on ``layer``
        (``pours.starved``), traces from inside it into the fill of the net's main group there, each a spoke that
        check counts, the cheapest first, until it is not starved; none if that cannot be done."""
        half = rules.drawn(net) / 2.0
        kept = snapshot()
        for _ in range(rules_spokes + 1):  # (the last round only finds whether the spokes made do)
            work, fill_of, wires, reach = state["work"], state["fill_of"], state["wires"], state["reach"]
            now = estimates.classify(work, _of(work, fill_of, {net}), wires)
            pour = next(x for x in now if x.layer == layer and x.outline is outline)
            if not estimates.starved(work, pour, pad, wires, [(at, pts) for other, at, pts in out.traces if other == net]):
                return
            comp, _, main = reach[net]
            window = pad.shape.buffer(FAR + 1.0).envelope
            fill = shapely.union_all([x.shape.intersection(window) for x in pour.pieces
                                      if comp.get(estimates.piece_key(pour, x)) == main and x.shape.intersects(window)])
            into = {(layer, main): fill.buffer(-(half + GAP))}
            room = free(net, window, half, rules.clearance, [layer]).intersection(within.buffer(-half).intersection(window))
            room = room.difference(margin.buffer(half)) if margin is not None else room
            s = Sizes(r, half, keep, rules.clearance, every)
            ways = _rays(net, pad.centre, layer, pad, {layer: room}, into, Polygon(), {}, [], s, None, None, trace_ok, via_ok)
            for way in sorted(ways, key=lambda j: j.cost)[:TRIES]:
                done = _apply(work, way, net, fill_of, wires, reach, area, r, half, now, out.traces)
                if done is not None and _coarser(_groups(work, reach[net][0], net), _groups(done[0], done[3][net][0], net)):
                    make(net, dataclasses.replace(way, kind="spoke"), done)
                    break
            else:
                break
        restore(kept)

    # Pads joined with fewer spokes than KiCad asks (a join may have joined them, through fill that met nothing else):
    # a trace into the fill counts as one.
    rules_spokes = max(1, board.min_spokes)
    for net in nets:
        for k, (other, layer, outline) in enumerate(board.pours):
            if other != net or layer >= nlayers or k not in state["fill_of"]:
                continue
            zone = board.zone(net, layer)
            for pad in [p for p in state["work"].pads if p.net_id == net and not p.is_via and p.on(layer)
                        and estimates.joined(p, zone) == "thermal" and p.shape.intersects(outline)]:
                spoke(net, pad, layer, outline)
    out.left = [p for net in nets for p in state["reach"][net][1]]
    return out


def _groups(board: Board, comp: dict, net: int) -> dict:
    """Pad id -> its group (``pours.reach``), for the pads (not vias) of ``net``."""
    return {p.pad_id: comp[("pad", p.pad_id)] for p in board.pads if p.net_id == net and not p.is_via}


def _coarser(before: dict, after: dict) -> bool:
    """Whether every two pads together ``before`` are together ``after`` too (pad id -> group)."""
    seen: dict = {}
    for pad, group in before.items():
        if seen.setdefault(group, after[pad]) != after[pad]:
            return False
    return True


def _grew(after: dict, before: dict, root, now) -> bool:
    """Whether the group ``root`` (before; ``now`` after) took in fill more than the copper of a join (a plane)."""
    fill = lambda comp, group: sum(key[-1] for key, g in comp.items() if key[0] == "piece" and g == group)
    return fill(after, now) > fill(before, root) + 2.0


def _keep(board: Board, way: Join, net: int, out: Joined) -> None:
    """``way``, made on ``board`` (its vias the last of its pads), into ``out``."""
    if way.vias:
        out.vias += board.pads[len(board.pads) - len(way.vias):]
    out.traces += [(net, layer, pts) for layer, pts in way.traces]
    for pad_id, change in way.relief.items():
        out.reliefs.setdefault(pad_id, {}).update(change)


def _reach(board: Board, pours: list, wires: list, net: int) -> tuple[dict, list, object]:
    """How the copper of ``net`` hangs together (``pours.reach``), as ``pours.classify`` found it for its ``pours``."""
    from weaveengine import pours as estimates
    return next((pour.reach for pour in pours if pour.net == net), None) or estimates.reach(board, pours, wires, net)


def _of(board: Board, fill_of: dict, nets: set) -> dict:
    return {k: f for k, f in fill_of.items() if board.pours[k][0] in nets}


def _apply(work: Board, way: Join, net: int, fill_of: dict, wires: list, reach: dict, area, r: float, half: float,
           now: list, traces: list):
    """The board, fills, routing copper and groups (``pours.reach``) with ``way`` made, if the estimate finds that it
    leaves every pad of every other net as joined as it was, no more islands in their pours, and no more pads starved
    of thermal spokes (``pours.starved``: KiCad finds a pad whose spokes reach only a piece that meets nothing else
    starved once that piece is joined, so a join may trade a pad left unconnected for one starved); else None.
    ``now``: the pours of ``net`` as they are (``pours.classify``); ``traces``: (net, layer, points) of the joins made."""
    from weaveengine import pours as estimates
    board = copy.copy(work)
    board.pads = list(work.pads)
    next_id = max((p.pad_id for p in board.pads), default=-1) + 1
    vias = []
    for x, y in way.vias:
        via = Pad.circle(next_id, x, y, r, net, name=f"via{next_id}", is_via=True)
        via.fanout = True
        next_id += 1
        vias.append(via)
    board.pads += vias
    for pad_id, change in way.relief.items():
        i = next(i for i, p in enumerate(board.pads) if p.pad_id == pad_id)
        board.pads[i] = dataclasses.replace(board.pads[i], **change)
    added = [(LineString(pts).buffer(half, quad_segs=4), net, layer) for layer, pts in way.traces]
    new_wires = wires + added
    if way.relief:
        pads = [p for p in board.pads if p.pad_id in way.relief]
        mine = estimates.refill(board, _of(board, fill_of, {net}), {net}, new_wires, shapely.union_all([p.shape for p in pads]), _reach_of(board, net))
    else:
        mine = estimates.with_copper(board, _of(board, fill_of, {net}),
                                     [(v.shape.intersection(area), None) for v in vias] + [(shape, layer) for shape, _, layer in added])
    # Other nets' pours the new copper comes near are filled again (it takes from them).
    # (only where it comes within their clearance and least width of their fill: else their fill is as it was)
    copper = [(v.shape, None) for v in vias] + [(shape, layer) for shape, _, layer in added]
    others = {other for k, (other, at, _) in enumerate(board.pours) if other != net and k in fill_of
              and any((layer is None or layer == at) and fill_of[k].distance(shape) < _reach_of(board, other) for shape, layer in copper)}
    theirs = estimates.refill(board, _of(board, fill_of, others), others, new_wires, shapely.union_all([c for c, _ in copper]),
                              max(_reach_of(board, other) for other in others)) if others else {}
    near = shapely.union_all([c for c, _ in copper] + [p.shape for p in board.pads if p.pad_id in way.relief])
    joins = traces + [(net, layer, pts) for layer, pts in way.traces]
    after = estimates.classify(board, mine, new_wires)
    if _starved(board, after, new_wires, joins, net, near) > _starved(work, now, wires, traces, net, near):
        return None
    changed = {net: _reach(board, after, new_wires, net)}
    for other in sorted(others):
        old = estimates.classify(work, _of(work, fill_of, {other}), wires)
        new = estimates.classify(board, _of(board, theirs, {other}), new_wires)
        if sum(pour.islands for pour in new) > sum(pour.islands for pour in old):
            return None
        if _starved(board, new, new_wires, joins, other, near) > _starved(work, old, wires, traces, other, near):
            return None
        if other in reach:
            changed[other] = _reach(board, new, new_wires, other)
            if not _coarser(_groups(work, reach[other][0], other), _groups(board, changed[other][0], other)):
                return None
    return board, {**fill_of, **mine, **theirs}, new_wires, {**reach, **changed}


def _starved(board: Board, pours: list, wires: list, traces: list, net: int, near) -> int:
    """How many (pad, pour) of ``net`` KiCad's check finds starved of thermal spokes (``pours.starved``) in ``pours``,
    of those a change at ``near`` (a shape) may touch: pads within ``STARVED`` of it (their spokes, or fill round them
    that met nothing else, ``pours.LONE``)."""
    from weaveengine import pours as estimates
    mine = [(layer, pts) for other, layer, pts in traces if other == net]
    by_id = {p.pad_id: p for p in board.pads if p.net_id == net}
    pads = [p for p in by_id.values() if not p.is_via and p.shape.distance(near) < STARVED]
    return sum(estimates.starved(board, pour, p, wires, mine, by_id) for pour in pours if pour.net == net
               for p in pads if p.on(pour.layer) and p.shape.intersects(pour.outline))


def _reach_of(board: Board, net: int) -> float:
    """How far a change can reach into the fill of ``net``'s pours: the most any of them keeps from other copper (its
    zone's clearance, or the board's), its least width, and a thermal relief's gap and spoke."""
    from weaveengine import pours as estimates
    far = 0.0
    for other, at, _ in board.pours:
        if other == net:
            zone = board.zone(other, at)
            gap, spoke = estimates.relief(Pad(-1, other, Polygon()), zone)
            far = max(far, board.pour_clearance(other, at) + board.rules.clearance + board.pour_width(other, at) + gap + spoke)
    return far + 0.1


def _depth(region, x: float, y: float) -> float:
    """How far (x, y) is inside ``region`` (0 outside)."""
    p = Point(x, y)
    return region.boundary.distance(p) if region.contains(p) else 0.0


def _candidates(board: Board, net: int, unit: list, pieces: list, target: dict, land: dict, into: dict, ends: list,
                tfree: dict, vfree, r: float, keep: float, half: float, nlayers: int, reliefs: bool, narrow: bool,
                trace_ok, via_ok) -> list[Join]:
    """The ways to join ``unit`` (pads of ``net`` in one group, with its ``pieces`` of fill: (layer, shape)) to another
    group of its net (``target``: (layer, group) -> its fill there; ``land``: that drawn in for a via's middle;
    ``into``: for a trace's end; ``ends``: the pads and vias of the other groups, and fixed vias, as (thing, centre,
    group)), each clear and priced, the cheapest of each kind to each group. ``tfree`` (layer -> where a trace's middle
    may run) and ``vfree`` (where a via's) find them; ``trace_ok`` and ``via_ok`` check them."""
    from weaveengine import pours as estimates
    rules = board.rules
    out: list[Join] = []
    s = Sizes(r, half, keep, rules.clearance, len(board.layers) + len(board.unrouted))
    vland = shapely.union_all([vfree.intersection(g) for g in land.values()]) if land else Polygon()

    def landed(x: float, y: float, skip=()):
        """The (layer, group) whose fill a via at (x, y) lands in, not on the layers ``skip``."""
        return next((key for key, g in land.items() if key[0] not in skip and g.contains(Point(x, y))), None)

    def via_cost(x: float, y: float, key) -> float:
        return VIA + ROOM * s.via_room + UNSURE * max(0.0, SURE - _depth(land[key], x, y))

    # Its thermal relief changed (no copper): only when asked for, and only for a pad its main fill comes near.
    for p in unit:
        for (at, _), t in target.items():
            zone = board.zone(net, at)
            if not p.on(at) or estimates.joined(p, zone) != "thermal":
                continue
            gap, spoke = estimates.relief(p, zone)
            least = board.pour_width(net, at)
            if p.shape.distance(t) > gap + spoke + least:
                continue
            if reliefs:
                out.append(Join(RELIEF, "relief", relief={p.pad_id: {"zone_connect": "solid"}}))
                angle = p.spoke_angle if p.spoke_angle is not None else 45.0 if p.radius is not None else 90.0
                out.append(Join(RELIEF, "relief", relief={p.pad_id: {"spoke_angle": 90.0 if angle % 90.0 else 45.0}}))
            if narrow:
                close = max(board.pour_clearance(net, at), least)
                if close < gap - 1e-9 or least < spoke - 1e-9:
                    out.append(Join(NARROWED, "narrowed", relief={p.pad_id: {"thermal_gap": min(gap, close), "spoke": min(spoke, least)}}))
            break
    # A via in a surface pad, landing in the fill on another layer.
    for p in unit:
        if p.layers is None:
            continue
        others = [g for key, g in land.items() if not p.on(key[0])]
        region = p.shape.buffer(-r).intersection(vfree).intersection(shapely.union_all(others)).buffer(-INSET) if others else Polygon()
        if region.is_empty:
            continue
        c = Point(p.centre)
        q = c if region.contains(c) else shapely.ops.nearest_points(region, c)[0]
        key = landed(q.x, q.y, skip=[at for at, _ in land if p.on(at)])
        if key is not None and via_ok(net, q.x, q.y):
            out.append(Join(IN_PAD + via_cost(q.x, q.y, key), "via in pad", vias=[(q.x, q.y)]))
    # A via in a cut-off piece of the net's pour, stitching it to the fill on another layer: at the place deepest in both.
    for layer, shape in pieces:
        region = shape.buffer(-(r - LANDING)).intersection(vland)
        polys = sorted((g for g in getattr(region, "geoms", [region]) if g.geom_type == "Polygon" and not g.is_empty),
                       key=lambda g: -g.area)[:3]
        for g in polys:
            x, y = shapely.maximum_inscribed_circle(g, 0.01).coords[0]
            key = landed(x, y, skip=[layer])
            if key is not None and via_ok(net, x, y):
                out.append(Join(via_cost(x, y, key), "stitch", vias=[(x, y)]))
    # From each pad, along rays on each routed layer it is on.
    for p in unit:
        for layer in range(nlayers):
            if p.on(layer):
                out += _rays(net, p.centre, layer, p, tfree, into, vland, land, ends, s, landed, via_cost, trace_ok, via_ok)
    # From a cut-off piece: a via in it and a trace on any layer (a jumper), or a trace over a gap on its own layer.
    aims = [g for g in [*land.values(), *into.values()] if not g.is_empty]
    for layer, shape in pieces:
        starts = shape.buffer(-(r - LANDING)).intersection(vfree).buffer(-INSET)
        if not starts.is_empty and aims:
            for x, y in _nearest(starts, aims, SAMPLES):
                if via_ok(net, x, y):
                    for other in range(nlayers):
                        out += _rays(net, (x, y), other, None, tfree, into, vland, land, ends, s, landed, via_cost,
                                     trace_ok, via_ok, (VIA + ROOM * s.via_room, layer))
        start = shape.buffer(-(half + GAP)).intersection(tfree.get(layer, Polygon())).buffer(-INSET) if layer < nlayers else Polygon()
        for key, aim in into.items():
            if key[0] != layer or start.is_empty or aim.is_empty:
                continue
            best = None
            for x, y in _nearest(start, [aim.buffer(-INSET)], SAMPLES):
                q = shapely.ops.nearest_points(aim, Point(x, y))[0]
                length = math.dist((x, y), (q.x, q.y))
                if length <= FAR and (best is None or length < best[0]) and trace_ok(net, layer, (x, y), (q.x, q.y)):
                    best = (length, [(x, y), (q.x, q.y)])
            if best is not None:
                out.append(Join(best[0] + ROOM * s.trace_room(best[0]), "trace", traces=[(layer, best[1])]))
    return out


@dataclass
class Sizes:
    r: float          # a via's radius
    half: float       # a trace's half width
    keep: float       # what a via keeps from foreign copper
    clearance: float  # what a trace keeps
    layers: int       # the layers a via goes through

    @property
    def via_room(self) -> float:
        return math.pi * (self.r + self.keep) ** 2 * self.layers

    def trace_room(self, length: float) -> float:
        return length * 2.0 * (self.half + self.clearance)


def _nearest(region, aims: list, count: int) -> list[tuple[float, float]]:
    """Up to ``count`` places in ``region``, by its edge, nearest any of ``aims`` (within ``FAR``), half a millimetre
    apart at least."""
    aims = [aim for aim in aims if not aim.is_empty]
    edge = shapely.segmentize(region.boundary, 0.25)
    pts = shapely.get_coordinates(edge)
    if not len(pts) or not aims:
        return []
    points = shapely.points(pts)
    d = np.min([shapely.distance(points, aim) for aim in aims], axis=0)
    out: list = []
    for i in np.argsort(d, kind="stable"):
        if d[i] > FAR:
            break
        x, y = pts[i]
        if all(math.dist((x, y), o) >= 0.5 for o in out):
            out.append((float(x), float(y)))
            if len(out) == count:
                break
    return out


def _rays(net: int, start, layer: int, pad, tfree: dict, into: dict, vland, land: dict, ends: list, s: Sizes, landed,
          via_cost, trace_ok, via_ok, via_start=None) -> list[Join]:
    """The joins by one straight trace on ``layer`` from ``start`` (a ``pad``'s centre, or, with ``via_start`` (its
    cost, and the layer of the piece it is in), a via in a cut-off piece): into another group's fill there, to its pad
    or via, or to a via that lands in its fill on another layer; the cheapest of each kind to each group, along
    ``DIRECTIONS`` rays."""
    free = tfree.get(layer)
    if free is None or free.is_empty:
        return []
    kind = "trace" if via_start is None else "jumper"
    base = 0.0 if via_start is None else via_start[0]
    first = [] if via_start is None else [start]
    x0, y0 = start
    t = np.arange(0.0, FAR + (_extent(pad) if pad is not None else 0.0) + 1e-9, STEP)
    a = 2.0 * np.pi * np.arange(DIRECTIONS) / DIRECTIONS
    xs, ys = x0 + np.outer(np.cos(a), t), y0 + np.outer(np.sin(a), t)
    clear = shapely.contains_xy(free, xs, ys)
    beyond = np.zeros_like(xs)  # (how far each place is beyond the pad: the trace's length that counts)
    if pad is not None:
        beyond = shapely.distance(pad.shape, shapely.points(xs, ys))
        clear |= beyond <= 0.0  # (inside the pad is its own copper)
    else:
        beyond = np.broadcast_to(t, xs.shape)
    stop = np.where((~clear).any(axis=1), (~clear).argmax(axis=1), len(t))  # (each ray, up to where it is first not clear)
    best: dict = {}

    def offer(join: Join, group) -> None:
        """Keeps the ``KEPT`` cheapest of a kind to a group, each ending apart from the others (if the cheapest is
        refused, by what it takes from another net's pour, say, one elsewhere may do)."""
        kept = best.setdefault((join.kind, group), [])
        end = join.traces[-1][1][-1] if join.traces else join.vias[-1]
        if any(math.dist(end, other.traces[-1][1][-1] if other.traces else other.vias[-1]) < APART for other in kept):
            same = [other for other in kept if math.dist(end, other.traces[-1][1][-1] if other.traces else other.vias[-1]) < APART]
            if all(join.cost < other.cost for other in same):
                kept[:] = [other for other in kept if other not in same] + [join]
        else:
            kept.append(join)
        kept.sort(key=lambda j: j.cost)
        del kept[KEPT:]

    def dead(kind: str, group, cost: float) -> bool:
        """Whether ``offer`` would keep nothing of a join of ``kind`` to ``group`` that costs ``cost`` or more (so it
        need not be checked)."""
        kept = best.get((kind, group))
        return kept is not None and len(kept) >= KEPT and cost >= kept[-1].cost

    for (at, group), aim in into.items():  # into another group's fill on this layer: a little past its edge, where the ray stays clear
        if at != layer:
            continue
        hit = shapely.contains_xy(aim, xs, ys)
        for k in range(len(a)):
            got = np.nonzero(hit[k, :stop[k]])[0]
            if len(got):
                i = min(got[0] + int(round(SURE / STEP)), stop[k] - 1)
                i = i if hit[k, i] else got[0]
                x, y, length = float(xs[k, i]), float(ys[k, i]), float(beyond[k, i])
                if not dead(kind, group, base + length + ROOM * s.trace_room(length)) and trace_ok(net, layer, start, (x, y), pad):
                    offer(Join(base + length + ROOM * s.trace_room(length) + UNSURE * max(0.0, SURE - _depth(aim, x, y)),
                               kind, vias=list(first), traces=[(layer, [start, (x, y)])]), group)
    # To a via that lands in the main fill on another layer: beside the pad, or clear of the jumper's first via.
    vhit = shapely.contains_xy(vland, xs, ys) & (beyond >= (s.r + GAP if pad is not None else 2.0 * s.r + HOLES))
    skip = [layer] + ([at for at, _ in land if pad.on(at)] if pad is not None else [])
    masks = {key: shapely.contains_xy(g, xs, ys) & vhit for key, g in land.items() if key[0] not in skip}
    least = np.minimum.accumulate(beyond[:, ::-1], axis=1)[:, ::-1]  # (the least length of a trace from here on, along each ray)
    vkind = "via" if via_start is None else kind
    for k in range(len(a)):
        for key, mask in masks.items():  # (along this ray, the first good place that lands in each group, and others further)
            last = -math.inf
            for i in np.nonzero(mask[k, :stop[k]])[0]:
                if dead(vkind, key[1], base + least[k, i] + ROOM * s.trace_room(least[k, i]) + VIA + ROOM * s.via_room):
                    break  # (nothing further along it can be kept)
                if t[i] - last < APART:
                    continue
                x, y = float(xs[k, i]), float(ys[k, i])
                if not via_ok(net, x, y) or not trace_ok(net, layer, start, (x, y), pad):
                    continue
                last = t[i]
                length = float(beyond[k, i])
                offer(Join(base + length + ROOM * s.trace_room(length) + via_cost(x, y, key), vkind,
                           vias=first + [(x, y)], traces=[(layer, [start, (x, y)])]), key[1])
    for thing, (x, y), group in ends:  # to a pad or via of another group
        if not thing.on(layer):
            continue
        length = pad.shape.distance(Point(x, y)) if pad is not None else math.dist(start, (x, y))
        tkind = kind + " to pad" if via_start is None else kind
        if 1e-9 < length <= FAR and not dead(tkind, group, base + length + ROOM * s.trace_room(length))                 and trace_ok(net, layer, start, (x, y), pad):
            offer(Join(base + length + ROOM * s.trace_room(length), tkind,
                       vias=list(first), traces=[(layer, [start, (x, y)])]), group)
    return [join for kept in best.values() for join in kept]


def _extent(pad) -> float:
    """How far the pad reaches from its centre."""
    cx, cy = pad.centre
    x0, y0, x1, y1 = pad.shape.bounds
    return max(math.dist((cx, cy), (x, y)) for x in (x0, x1) for y in (y0, y1))
