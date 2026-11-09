"""Vias that join pads to their net's plane (design 11.2).

A net with a copper plane (a pour, in KiCad a zone) is not routed as traces:
the plane joins its pads when the CAD tool fills it. A pad the plane does not
reach, because the plane is on another layer (an inner ground plane, the
ground pour on the back of a two-layer board) or does not extend to it, stays
unconnected unless something takes it there. Once the board is routed, each
such pad gets a via beside it where a via may stand among everything else
(pads, keepouts, the traces and vias of other nets, on every layer) and lands
inside a plane of its net, joined to the pad by a short straight trace that
keeps its clearances too; or it shares a via of its net placed close by.
Done after routing, it never takes room a trace needed; a pad with no such
place is left as it was, and named. Fixed copper of the net counts as the
net's: a pad a locked trace already joins to a locked via in the plane needs
nothing, and a locked via in the plane close by is shared like a new one.
"""
import math

import shapely
import shapely.ops
from shapely.geometry import LineString, Point, Polygon

from weaveengine.board import Board, Pad, circle_polygon

DIRECTIONS = 24    # rays round a pad a via is tried along
REACH = 2.0        # mm beyond the pad a via is looked for
STEP = 0.1         # mm between the places tried along a ray
SHARE = 1.0        # mm: a via of the net this near the pad is joined to instead of a new one
GAP = 0.05         # mm between a pad and its via
HOLES = 0.25       # mm a via keeps from a through-hole pad or fixed via of its own net (drill to drill: KiCad's default minimum)
LANDING = 0.02    # mm a via may stand nearer the edge of the fill than its radius (its pad is no bigger than its copper, and KiCad's fill gives it a little room)
FILL = 0.25        # mm: a pour's fill is not narrower than this (KiCad's default zone minimum width)


def covered(board: Board, pad: Pad, cut=frozenset()) -> bool:
    """Whether a plane of the pad's net reaches the pad on a layer it has copper on
    (not one where it is in a piece ``cut`` off from the rest: (pad, layer) pairs;
    nor one whose zone does not join it, its connection being "none": ``pours.joined``)."""
    from weaveengine.pours import joined
    centre = Point(pad.centre)
    return any(net == pad.net_id and pad.on(layer) and (pad.pad_id, layer) not in cut and shape.contains(centre)
               and joined(pad, board.zone(net, layer)) != "none" for net, layer, shape in board.pours)


def fanout(board: Board, lines: dict, wire_net: dict, wire_layer: dict, shapes=(), cut=frozenset(), teardrops=None) -> tuple[list[Pad], list[tuple[int, int, list]], list[Pad]]:
    """For the routed ``board`` (its vias among its pads), its traces and other
    copper ``shapes`` (shape, net, layer: teardrops; and the same as ``teardrops``, wire -> polygons, for the fill): the vias to add, the
    traces to add as (net, layer, points), and the pads that were left with no
    place for one. ``cut``: (pad, layer) of the pads a pour reaches only in a
    piece cut off from the rest of it (``pours.estimate``): joined as if that
    pour did not reach them, by a via to a plane of the net on another layer."""
    rules = board.rules
    lonely = _lonely(board, cut)
    if not lonely:
        return [], [], []
    r = rules.via_diameter / 2.0
    keep = rules.apart(rules.via_clearance)  # what a via keeps from foreign copper
    edge = (rules.clearance if rules.edge_clearance is None else rules.edge_clearance)
    inside_via = board.outline.buffer(-(edge + r))
    margin = shapely.union_all([m.buffer(edge) for m in board.margin]) if board.margin else None  # (KiCad holds copper off these too)
    no_via = shapely.union_all(board.no_via) if board.no_via else None  # (areas no via may touch)
    nlayers = len(board.layers)
    every = nlayers + len(board.unrouted)  # (a via goes through the layers left out of the routing too, and lands in their planes)
    # Everything a via or a stub must keep clear of, per layer: (shape, net, what it asks; -2 = keepout).
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
            things[wire_layer[w]].append((LineString(line).buffer(rules.drawn(net) / 2.0, quad_segs=4), net, rules.clearance))
    for shape, net, layer in shapes:
        things[layer].append((shape, net, rules.clearance))
    through = {p.shape for p in board.pads if not p.is_via and (p.layers is None or len(p.layers) == nlayers > 1)}
    through |= {o.shape for o in board.obstacles if o.via}
    trees = [shapely.STRtree([t[0] for t in ts]) if ts else None for ts in things]
    widest = max([keep, rules.clearance] + [t[2] for ts in things for t in ts])
    # net -> (layer, its fill, for a trace that ends in it; that drawn in by a via's radius, for a via: the fill flows
    # round a via of its net, wherever there is room for the via among foreign copper). The fill is the estimate of
    # what KiCad will pour with the routing as it stands (``pours.estimate``): the pieces that reach the net's pads,
    # not the pour's outline, which a trace or a keepout can leave empty.
    pours: dict[int, list] = {}
    wanted = {p.net_id for p in lonely}
    from weaveengine import pours as estimates  # (it reads FILL from here)
    for pour in estimates.estimate(board, lines, wire_net, wire_layer, teardrops):
        if pour.net in wanted:
            reach = [piece.shape for piece in pour.pieces if piece.kind in ("main", "via")]
            if not reach:  # (a plane with no pad of its net on it, only vias to join: any of its fill will do)
                reach = [piece.shape for piece in pour.pieces]
            fill = shapely.union_all(reach) if reach else Polygon()
            pours.setdefault(pour.net, []).append((pour.layer, fill, fill.buffer(-(r - LANDING))))

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

    fixed: dict[int, list] = {}  # net -> the centres of its fixed vias
    for o in board.obstacles:
        if o.via and o.net_id in wanted:
            fixed.setdefault(o.net_id, []).append((o.shape.centroid.x, o.shape.centroid.y))
    vias: list[Pad] = []
    traces: list[tuple[int, int, list]] = []
    left: list[Pad] = []
    next_id = max((p.pad_id for p in board.pads), default=-1) + 1
    for pad in lonely:
        layer = next((i for i in range(nlayers) if pad.on(i)), 0)
        half = rules.drawn(pad.net_id) / 2.0
        start = pad.centre

        def stub_ok(x: float, y: float) -> bool:
            stub = LineString([start, (x, y)]).buffer(half, quad_segs=4)
            if margin is not None and stub.difference(pad.shape).intersects(margin):
                return False
            return board.outline.buffer(-edge).contains(stub) and clear(stub, pad.net_id, layer, rules.clearance)

        def lands(x: float, y: float) -> bool:
            """Whether a via there is inside a plane of the pad's net on a layer the pad has no copper on
            (a via into its own layer's plane joins nothing)."""
            return any(not pad.on(at) and s.contains(Point(x, y)) for at, _, s in pours.get(pad.net_id, ()))

        # A via of its net close by, placed for another pad or fixed (a locked one that lands in the plane), is shared.
        near = [v.centre for v in vias if v.net_id == pad.net_id and pad.shape.distance(Point(v.centre)) - r <= SHARE]
        near += [c for c in fixed.get(pad.net_id, ()) if pad.shape.distance(Point(c)) - r <= SHARE and lands(*c)]
        shared = next((c for c in sorted(near, key=lambda c: math.dist(c, start)) if stub_ok(*c)), None)
        if shared is not None:
            traces.append((pad.net_id, layer, [start, shared]))
            # (later vias and stubs of other nets keep clear of this stub)
            things[layer].append((LineString([start, shared]).buffer(half, quad_segs=4), pad.net_id, rules.clearance))
            trees[layer] = shapely.STRtree([t[0] for t in things[layer]])
            continue

        def legal(x: float, y: float) -> bool:
            if not inside_via.contains(Point(x, y)) or not lands(x, y):
                return False
            disc = circle_polygon(x, y, r)
            if margin is not None and disc.intersects(margin):
                return False
            if no_via is not None and disc.intersects(no_via):
                return False
            # (two vias of the net may touch, but their holes keep ``HOLES`` apart: KiCad's hole to hole)
            if any(math.dist((x, y), v.centre) < max(2 * r + (0.0 if v.net_id == pad.net_id else keep), rules.via_drill + HOLES) - 1e-9 for v in vias):
                return False
            return all(clear(disc, pad.net_id, i, keep, drilled=True) for i in range(every)) and stub_ok(x, y)

        # A plane of its net on its own layer that stops short of it, or does not join it: a short trace into the
        # plane's fill (its pieces that reach the net's pads, not the one this pad may be cut off in), no via.
        into = None
        for at, shape, _ in pours.get(pad.net_id, ()):
            if at == layer and pad.shape.distance(shape) <= REACH:
                inner = shape.buffer(-(half + GAP))
                if not inner.is_empty:
                    q = shapely.ops.nearest_points(inner, Point(start))[0]
                    if pad.shape.distance(q) <= REACH and stub_ok(q.x, q.y):
                        into = (q.x, q.y)
                        break
        if into is not None:
            traces.append((pad.net_id, layer, [start, into]))
            things[layer].append((LineString([start, into]).buffer(half, quad_segs=4), pad.net_id, rules.clearance))
            trees[layer] = shapely.STRtree([t[0] for t in things[layer]])
            continue
        spot = _spot(pad, r, legal)
        if spot is None:
            left.append(pad)
            continue
        via = Pad.circle(next_id, spot[0], spot[1], r, pad.net_id, name=f"via{next_id}", is_via=True)
        via.fanout = True
        next_id += 1
        vias.append(via)
        traces.append((pad.net_id, layer, [start, spot]))
        # (later vias and stubs keep clear of this one and its stub)
        for i in range(every):
            things[i].append((via.shape, pad.net_id, keep))
        things[layer].append((LineString([start, spot]).buffer(half, quad_segs=4), pad.net_id, rules.clearance))
        trees = [shapely.STRtree([t[0] for t in ts]) if ts else None for ts in things]
    return vias, traces, left


def _lonely(board: Board, cut=frozenset()) -> list[Pad]:
    """The pads of plane nets no plane reaches, by itself or through copper of
    its net it touches: other pads (an exposed pad's own holes, say) and fixed
    wiring (a locked trace to a locked via in the plane). Not other copper the
    KiCad board gives a net (text, an opening in the solder mask): KiCad does
    not count it as joining anything."""
    by_net: dict[int, list[Pad]] = {}
    for p in board.pads:
        if p.net_id in board.plane_nets and not p.is_via:
            by_net.setdefault(p.net_id, []).append(p)
    fixed: dict[int, list] = {}
    for o in board.obstacles:
        if o.net_id in by_net and not o.copper:
            fixed.setdefault(o.net_id, []).append(o)
    out = []
    for net in sorted(by_net):
        pads = by_net[net]
        # (shape, layers, whether a plane reaches it) of each pad of the net, then of each piece of its fixed copper
        items = [(p.shape, p.layers, covered(board, p, cut)) for p in pads]
        items += [(o.shape, o.layers, any(n == net and o.on(at) and shape.intersects(o.shape) for n, at, shape in board.pours))
                  for o in fixed.get(net, ())]
        tree = shapely.STRtree([shape for shape, _, _ in items])
        todo = [i for i, (_, _, reached) in enumerate(items) if reached]
        reached = set(todo)
        while todo:
            shape, layers, _ = items[todo.pop()]
            for j in tree.query(shape, predicate="intersects").tolist():
                other = items[j][1]
                if j not in reached and (layers is None or other is None or layers & other):
                    reached.add(j)
                    todo.append(j)
        out += [p for k, p in enumerate(pads) if k not in reached]
    return out


def _spot(pad: Pad, r: float, legal) -> tuple[float, float] | None:
    """The nearest place beside ``pad`` a via of its net may stand, or None."""
    cx, cy = pad.centre
    tries = []
    for k in range(DIRECTIONS):
        dx, dy = math.cos(2 * math.pi * k / DIRECTIONS), math.sin(2 * math.pi * k / DIRECTIONS)
        t = 0.0
        while t < 20.0 and pad.shape.distance(Point(cx + t * dx, cy + t * dy)) < r + GAP:  # out of the pad
            t += STEP / 2
        while t < 20.0:
            gap = pad.shape.distance(Point(cx + t * dx, cy + t * dy)) - r
            if gap > REACH:
                break
            tries.append((gap, k, cx + t * dx, cy + t * dy))
            t += STEP
    for _, _, x, y in sorted(tries):
        if legal(x, y):
            return (x, y)
    return None
