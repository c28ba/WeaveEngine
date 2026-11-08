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
place is left as it was, and named.
"""
import math

import shapely
import shapely.ops
from shapely.geometry import LineString, Point

from weaveengine.board import Board, Pad, circle_polygon

DIRECTIONS = 24    # rays round a pad a via is tried along
REACH = 2.0        # mm beyond the pad a via is looked for
STEP = 0.1         # mm between the places tried along a ray
SHARE = 1.0        # mm: a via of the net this near the pad is joined to instead of a new one
GAP = 0.05         # mm between a pad and its via
HOLES = 0.25       # mm a via keeps from a through-hole pad of its own net (drill to drill: KiCad's default minimum)
FILL = 0.25        # mm: a pour's fill is not narrower than this (KiCad's default zone minimum width)


def covered(board: Board, pad: Pad) -> bool:
    """Whether a plane of the pad's net reaches the pad on a layer it has copper on."""
    centre = Point(pad.centre)
    return any(net == pad.net_id and pad.on(layer) and shape.contains(centre) for net, layer, shape in board.pours)


def fanout(board: Board, lines: dict, wire_net: dict, wire_layer: dict, shapes=()) -> tuple[list[Pad], list[tuple[int, int, list]], list[Pad]]:
    """For the routed ``board`` (its vias among its pads), its traces and other
    copper ``shapes`` (shape, net, layer: teardrops): the vias to add, the
    traces to add as (net, layer, points), and the pads that were left with no
    place for one."""
    rules = board.rules
    lonely = _lonely(board)
    if not lonely:
        return [], [], []
    r = rules.via_diameter / 2.0
    keep = rules.apart(rules.via_clearance)  # what a via keeps from foreign copper
    edge = (rules.clearance if rules.edge_clearance is None else rules.edge_clearance)
    inside_via = board.outline.buffer(-(edge + r))
    margin = shapely.union_all([m.buffer(edge) for m in board.margin]) if board.margin else None  # (KiCad holds copper off these too)
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
                things[i].append((o.shape, o.net_id if (o.copper or o.net_id >= 0) else -2, rules.apart(o.clearance)))
    for w, line in lines.items():
        if len(line) >= 2 and w in wire_layer:
            net = wire_net[w]
            things[wire_layer[w]].append((LineString(line).buffer(rules.drawn(net) / 2.0, quad_segs=4), net, rules.clearance))
    for shape, net, layer in shapes:
        things[layer].append((shape, net, rules.clearance))
    through = {p.shape for p in board.pads if not p.is_via and (p.layers is None or len(p.layers) == nlayers > 1)}
    trees = [shapely.STRtree([t[0] for t in ts]) if ts else None for ts in things]
    widest = max([keep, rules.clearance] + [t[2] for ts in things for t in ts])
    def filled(shape, net: int, layer: int):
        """Roughly where KiCad's fill of a pour of ``net`` will be: its outline, held off the board edge and
        foreign copper, without the parts narrower than a fill's minimum width."""
        tree = trees[layer]
        near = [things[layer][i] for i in tree.query(shape, predicate="dwithin", distance=widest).tolist()] if tree else []
        held = [s.buffer(asks if other != -2 else 0.0, quad_segs=4) for s, other, asks in near if other != net]
        fill = shape.intersection(board.outline.buffer(-edge))
        if held:
            fill = fill.difference(shapely.union_all(held))
        return fill.buffer(-FILL / 2.0).buffer(FILL / 2.0)

    # net -> (layer, its fill, for a trace that ends in it; its outline drawn in by a via's radius, for a via: the
    # fill flows round a via of its net, wherever there is room for the via among foreign copper)
    pours: dict[int, list] = {}
    wanted = {p.net_id for p in lonely}
    for net, at, shape in board.pours:
        if net in wanted:
            pours.setdefault(net, []).append((at, filled(shape, net, at), shape.buffer(-r)))

    def clear(shape, net: int, layer: int, need: float, drilled: bool = False) -> bool:
        """Whether ``shape`` (copper of ``net``) keeps ``need`` from everything foreign on ``layer``,
        and, if ``drilled`` (a via), ``HOLES`` from every through-hole pad."""
        tree = trees[layer]
        if tree is None:
            return True
        for i in tree.query(shape, predicate="dwithin", distance=widest + need).tolist():
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

        near = [v for v in vias if v.net_id == pad.net_id and pad.shape.distance(Point(v.centre)) - r <= SHARE]
        shared = next((v for v in sorted(near, key=lambda v: math.dist(v.centre, start)) if stub_ok(*v.centre)), None)
        if shared is not None:
            traces.append((pad.net_id, layer, [start, shared.centre]))
            continue

        def legal(x: float, y: float) -> bool:
            # (inside a plane on a layer the pad has no copper on: a via into its own layer's plane joins nothing)
            if not inside_via.contains(Point(x, y)) or not any(not pad.on(at) and s.contains(Point(x, y)) for at, _, s in pours.get(pad.net_id, ())):
                return False
            disc = circle_polygon(x, y, r)
            if margin is not None and disc.intersects(margin):
                return False
            if any(math.dist((x, y), v.centre) < 2 * r + (0.0 if v.net_id == pad.net_id else keep) - 1e-9 for v in vias):
                return False
            return all(clear(disc, pad.net_id, i, keep, drilled=True) for i in range(every)) and stub_ok(x, y)

        # A plane of its net on its own layer that stops short of it: a short trace into the plane, no via.
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


def _lonely(board: Board) -> list[Pad]:
    """The pads of plane nets no plane reaches, by itself or through copper of
    its net it touches (an exposed pad's own holes, say)."""
    by_net: dict[int, list[Pad]] = {}
    for p in board.pads:
        if p.net_id in board.plane_nets and not p.is_via:
            by_net.setdefault(p.net_id, []).append(p)
    out = []
    for net in sorted(by_net):
        pads = by_net[net]
        reached = {p.pad_id for p in pads if covered(board, p)}
        grew = True
        while grew:
            grew = False
            for p in pads:
                if p.pad_id not in reached and any(q.pad_id in reached and q.shape.intersects(p.shape)
                                                   and (p.layers is None or q.layers is None or p.layers & q.layers) for q in pads):
                    reached.add(p.pad_id)
                    grew = True
        out += [p for p in pads if p.pad_id not in reached]
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
