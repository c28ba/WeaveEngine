"""An estimate of the copper KiCad will fill in each pour of a routing (design section 11.3):
the pour's outline less every other net's copper and its clearance, without the slivers, in
pieces; the piece with most of the net's pads is the main one, and the others hold pads that
are cut off, or none (islands)."""
import math
from dataclasses import dataclass, field

import shapely
from shapely import affinity
from shapely.geometry import LineString, Point

from weaveengine.realize.taper import with_wedges


@dataclass
class Piece:
    shape: object                                   # a Polygon
    kind: str                                       # "main", "cut" (holds pads of the net, none of them reached), "via" (a pad through the board or a via joins it to the main piece) or "island"
    pads: list[str] = field(default_factory=list)   # names of the net's pads in it (vias not among them)
    pad_ids: set = field(default_factory=set)       # and the ids of those and of its vias


@dataclass
class Pour:
    net: int
    layer: int
    outline: object
    pieces: list[Piece] = field(default_factory=list)

    @property
    def cut_off(self) -> list[str]:
        return [name for piece in self.pieces if piece.kind == "cut" for name in piece.pads]

    @property
    def islands(self) -> int:
        return sum(1 for piece in self.pieces if piece.kind == "island")


def estimate(board, lines: dict, wire_net: dict, wire_layer: dict, teardrops: dict | None = None,
             clearance: float | None = None, min_width: float | None = None, widths: dict | None = None) -> list[Pour]:
    """The estimated fill of each pour of ``board`` (its vias among its pads) for the routed
    ``lines``. ``clearance``: the zones' (default: each zone's own, or the board's less the routing
    margin, whichever is more: ``Board.pour_clearance``). ``min_width``: the narrowest fill (default:
    each zone's own minimum thickness, ``Board.pour_width``). A pad the zone joins by a thermal relief
    is joined only where one of its spokes reaches the fill (``spokes``); where zones of two nets
    overlap, the one of higher priority fills first and the other keeps clear of its fill.
    ``widths``: wire -> [(points, width)] for traces drawn in pieces (``realize.taper``)."""
    found, wires = fills(board, lines, wire_net, wire_layer, teardrops, clearance, min_width, widths)
    return classify(board, found, wires)


def copper_of(board, lines: dict, wire_net: dict, wire_layer: dict, teardrops: dict | None = None,
              widths: dict | None = None) -> list[tuple]:
    """(shape, net, layer) of every trace and teardrop of the routing."""
    rules = board.rules
    wires = []
    for w, line in lines.items():
        if len(line) >= 2 and w in wire_layer:
            net = wire_net[w]
            for pts, width in (widths or {}).get(w) or [(line, rules.drawn(net))]:
                wires.append((LineString(pts).buffer(width / 2.0, quad_segs=4), net, wire_layer[w]))
    for w, drops in (teardrops or {}).items():
        if w in wire_layer:
            wires += [(shapely.Polygon(poly[:5]).buffer(0), wire_net[w], wire_layer[w]) for poly in drops]
    return wires


def fills(board, lines: dict, wire_net: dict, wire_layer: dict, teardrops: dict | None = None,
          clearance: float | None = None, min_width: float | None = None, widths: dict | None = None,
          nets: set | None = None, wires: list | None = None, window=None) -> tuple[dict, list]:
    """The fill of each pour (its index in ``board.pours`` -> its copper, with the net's own pads and traces in it, as
    one shape: ``estimate``), and the routing's copper (``copper_of``). ``nets``: only the pours of these nets (the
    zones of other nets that fill before them, which they keep clear of, are filled but not returned); ``wires``: the
    routing's copper, if known already; ``window``: only the fill within this shape (``refill``)."""
    rules = board.rules
    base = rules.clearance - rules.margin if clearance is None else clearance
    inside = pour_area(board, clearance)
    # Higher priority first: a zone of another net that it overlaps is filled already (KiCad's order).
    rank = lambda k: board.zone(*board.pours[k][:2]).priority
    order = sorted(range(len(board.pours)), key=lambda k: (-rank(k), k))
    if nets is not None:
        want = {k for k in order if board.pours[k][0] in nets}
        more = want
        while more:
            more = {j for j in order if j not in want for k in more
                    if board.pours[j][1] == board.pours[k][1] and board.pours[j][0] != board.pours[k][0] and rank(j) > rank(k)}
            want |= more
        order = [k for k in order if k in want]
    layers = {board.pours[k][1] for k in order}
    def asks(net: int) -> float:  # (what a net's class asks, the board's if it is not known, never more than the board's)
        if clearance is not None or not (board.net_clearance or board.default_clearance):
            return base
        got = board.net_clearance.get(net, board.default_clearance)
        return base if got is None else min(base, got)

    # Each layer's copper: (shape, net, its own local clearance; None for a keepout, which asks none).
    copper: dict[int, list] = {at: [] for at in layers}
    for p in board.pads:
        for at in layers:
            if p.on(at):
                copper[at].append((p.shape, p.net_id, p.clearance))
    for o in board.obstacles:
        for at in layers:
            if o.on(at) and o.hole:  # (KiCad keeps the hole clearance from a hole with no copper, not the zone's)
                copper[at].append((o.shape.buffer(o.clearance, quad_segs=4), -2, None))
            elif o.on(at) and o.fill:
                copper[at].append((o.shape, o.net_id if o.copper or o.net_id >= 0 else -2, o.clearance if o.copper or o.net_id >= 0 else None))
    if wires is None:
        wires = copper_of(board, lines, wire_net, wire_layer, teardrops, widths)
    for shape, net, at in wires:
        if at in layers:
            copper[at].append((shape, net, 0.0))
    trees = {at: shapely.STRtree([c[0] for c in cs]) for at, cs in copper.items() if cs}
    filled: dict[int, list] = {}  # layer -> (net, priority, fill) of the pours done
    found: dict[int, object] = {}
    for k in order:
        net, at, outline = board.pours[k]
        zone = board.zone(net, at)
        own_zone = (board.zone(net, at).clearance or 0.0) if clearance is None else 0.0
        keep = max(asks(net), own_zone)  # (and from another net's copper, the more of the two classes)
        pair = lambda other: max(keep, asks(other))
        area = outline.intersection(inside)
        if window is not None:
            area = area.intersection(window)
        fill = area
        tree = trees.get(at)
        # What other nets' copper and zones take, which the spokes keep clear of too (KiCad's order: the reliefs out, the
        # clearances out, the spokes in, the clearances out again, then what is narrower than the least width).
        blocked = []
        if tree is not None:
            near = [copper[at][i] for i in tree.query(area, predicate="dwithin", distance=max(base, keep) + own_zone + 1.0).tolist()]
            blocked += [s.buffer(max(ask, pair(other)) if ask is not None else 0.0, quad_segs=4) for s, other, ask in near if other != net]
        blocked += [f.buffer(pair(other), quad_segs=4) for other, rank_, f in filled.get(at, ()) if other != net and rank_ > zone.priority]
        blocked = shapely.union_all(blocked) if blocked else None
        if blocked is not None:
            fill = fill.difference(blocked)
        own = [p for p in board.pads if p.net_id == net and p.on(at) and p.shape.intersects(outline)
               and (window is None or p.shape.intersects(window))]
        how = {p.pad_id: joined(p, zone) for p in own}
        relieved = [p for p in own if how[p.pad_id] in ("thermal", "none")]
        if relieved:  # (the fill keeps its gap from a pad it joins by spokes, and its clearance from one it does not join)
            fill = fill.difference(shapely.union_all([p.shape.buffer(relief(p, zone)[0] if how[p.pad_id] == "thermal" else keep, quad_segs=4)
                                                      for p in relieved]))
        least = board.pour_width(net, at) if min_width is None else min_width
        whole = fill
        fill = fill.buffer(-least / 2.0 + EPSILON).buffer(least / 2.0 - EPSILON)
        # The spokes of the pads it joins by a thermal relief: those whose end is in the fill, or in a spoke of another
        # pad whose end is in it (two pads facing each other closer than their gaps). Then cut and thinned as the fill.
        thermal = [p for p in own if how[p.pad_id] == "thermal"]
        drawn = [s for p in thermal for s in spokes(p, zone, least)]
        kept = [shape for shape, tip in drawn if fill.distance(tip) <= 1e-6]
        if len(kept) < len(drawn):
            index = shapely.STRtree([shape for shape, _ in drawn])
            for i, (shape, tip) in enumerate(drawn):
                if fill.distance(tip) > 1e-6 and any(j != i and drawn[j][0].distance(tip) <= 1e-6 and shape.distance(drawn[j][1]) <= 1e-6
                                                     for j in index.query(tip, predicate="dwithin", distance=1e-6).tolist()):
                    kept.append(shape)
        grown = shapely.union_all([whole, *kept]).intersection(area) if kept else whole
        if kept and blocked is not None:
            grown = grown.difference(blocked)
        fill = _prune(grown, least)
        # The net's own pads are copper whatever their width, and its traces join what they touch.
        joins = [p.shape.intersection(inside) for p in own if how[p.pad_id] in ("solid", "thermal")]
        joins += [shape for shape, other, at_ in wires if other == net and at_ == at and shape.intersects(outline)
                  and (window is None or shape.intersects(window))]
        fill = shapely.union_all([fill, *joins])
        filled.setdefault(at, []).append((net, zone.priority, fill))
        found[k] = fill
    if nets is not None:
        found = {k: fill for k, fill in found.items() if board.pours[k][0] in nets}
    return found, wires


def pour_area(board, clearance: float | None = None):
    """Where a pour may fill: the board less its edge clearance, as KiCad's fill keeps it (the board's own, whatever
    the traces were asked to keep), and as much from drawings on the Margin layer."""
    rules = board.rules
    base = rules.clearance - rules.margin if clearance is None else clearance
    edge = (rules.clearance if rules.edge_clearance is None else rules.edge_clearance) - rules.margin
    inside = board.outline.buffer(-(max(edge, base) if board.pour_edge is None else board.pour_edge))
    if board.margin:
        inside = inside.difference(shapely.union_all([m.buffer(edge if board.pour_edge is None else board.pour_edge) for m in board.margin]))
    return inside


def refill(board, fill_of: dict, nets: set, wires: list, near, reach: float) -> dict:
    """The fills of ``nets``' pours (``fill_of``: pour index -> fill, ``fills``) made again where something changed
    within ``near`` (a shape): within ``reach`` of it, the fill is made anew (from a window a little larger, so that
    what is made at its edge is as the whole fill makes it), and the rest is as it was."""
    inner = near.buffer(reach, join_style="mitre")
    outer = inner.buffer(REFILL, join_style="mitre")
    local, _ = fills(board, {}, {}, {}, nets=nets, wires=wires, window=outer)
    return {k: shapely.union_all([fill_of[k].difference(inner), local[k].intersection(inner)]) for k in local}


REFILL = 3.0  # mm round what is made anew by ``refill`` that it fills too, and does not keep


def with_copper(board, fill_of: dict, shapes) -> dict:
    """``fill_of`` (pour index -> fill, ``fills``) with more copper of the pours' net joined in as ``fills`` joins it
    (``shapes``: (shape, layer), layer None for a via, on every layer): where it meets a pour's outline. Copper of
    its own net takes nothing from a fill, so the rest of it is as it was."""
    out = dict(fill_of)
    for k, fill in fill_of.items():
        _, at, outline = board.pours[k]
        more = [shape for shape, layer in shapes if (layer is None or layer == at) and shape.intersects(outline)]
        if more:
            out[k] = shapely.union_all([fill, *more])
    return out


def classify(board, fill_of: dict, wires: list) -> list[Pour]:
    """The pieces of each pour's fill (``fills``: pour index -> fill, with all the pours of each net in it) and their
    kinds; the pours in the order of ``board.pours``."""
    pours = []
    for k in sorted(fill_of):
        net, at, outline = board.pours[k]
        zone = board.zone(net, at)
        own = [p for p in board.pads if p.net_id == net and p.on(at) and p.shape.intersects(outline) and joined(p, zone) != "none"]
        fill = fill_of[k]
        parts = sorted(getattr(fill, "geoms", [fill]), key=lambda g: -g.area)
        pour = Pour(net, at, outline)
        has = [[p for p in own if g.intersects(p.shape)] for g in parts]
        main = max(range(len(parts)), key=lambda j: (sum(not p.is_via for p in has[j]), parts[j].area), default=None)
        for j, g in enumerate(parts):
            if g.geom_type == "Polygon" and not g.is_empty:
                names = [p.name or str(p.pad_id) for p in has[j] if not p.is_via]
                pour.pieces.append(Piece(g, "main" if j == main and names else "", names, {p.pad_id for p in has[j]}))
        pours.append(pour)
    _join(pours, _touching(board, {pour.net for pour in pours}, wires))
    # The islands KiCad removes, by the zone's island removal, are not there; but a zone that is islands on every
    # layer it fills (joined to nothing at all) is left whole. (One zone over several layers: one ``Zone``.)
    zones: dict[int, list] = {}
    for pour in pours:
        zones.setdefault(id(board.zone(pour.net, pour.layer)), []).append(pour)
    for group in zones.values():
        zone = board.zone(group[0].net, group[0].layer)
        if zone.islands == 1 or all(x.kind == "island" for pour in group for x in pour.pieces):
            continue
        for pour in group:
            pour.pieces = [x for x in pour.pieces if x.kind != "island" or (zone.islands == 2 and x.shape.area >= zone.island_area)]
    return pours


def _prune(fill, least: float):
    """What KiCad keeps of ``fill`` at a least width of ``least``: shrunk by half of it, the pieces then less than
    it across either way dropped (a blob is not worth keeping), grown back."""
    shrunk = fill.buffer(-least / 2.0 + EPSILON)
    parts = [g for g in getattr(shrunk, "geoms", [shrunk])
             if g.geom_type == "Polygon" and not g.is_empty and max(g.bounds[2] - g.bounds[0], g.bounds[3] - g.bounds[1]) >= least]
    return shapely.union_all(parts).buffer(least / 2.0 - EPSILON) if parts else shapely.Polygon()


def _touching(board, nets: set, wires=()) -> dict:
    """Pad id -> one pad of the group of the net's pads whose copper meets on a layer they share (a pad through
    the board inside a part's surface pad, say), or that a trace of the net joins (``wires``: shape, net, layer):
    KiCad sees them joined, on any layer."""
    pads = [p for p in board.pads if p.net_id in nets]
    group = {p.pad_id: p.pad_id for p in pads}

    def root(a: int) -> int:
        while group[a] != a:
            group[a] = a = group[group[a]]
        return a

    if pads:
        a, b = shapely.STRtree([p.shape for p in pads]).query([p.shape for p in pads], predicate="intersects")
        for i, j in zip(a.tolist(), b.tolist()):
            p, q = pads[i], pads[j]
            if i < j and p.net_id == q.net_id and (p.layers is None or q.layers is None or p.layers & q.layers):
                group[root(p.pad_id)] = root(q.pad_id)
        tree = shapely.STRtree([p.shape for p in pads])
        for shape, net, at in wires:
            if net in nets:
                ends = [pads[i] for i in tree.query(shape, predicate="intersects").tolist() if pads[i].net_id == net and pads[i].on(at)]
                for q in ends[1:]:
                    group[root(q.pad_id)] = root(ends[0].pad_id)
    return {k: root(k) for k in group}


GAP, SPOKE = 0.5, 0.5  # mm: KiCad's default thermal relief gap and spoke width
ERROR = 0.005          # mm: KiCad's default maximum error, which its spokes reach past the gap
EPSILON = 0.001        # mm: what KiCad's fill leaves of the least width, so that copper just that wide stays


def joined(pad, zone) -> str:
    """How a pour joins ``pad``: "solid", "thermal" or "none" (a via always solid, as KiCad joins it)."""
    if pad.is_via:
        return "solid"
    how = pad.zone_connect if pad.zone_connect in ("solid", "thermal", "none") else zone.connect
    if how == "tht":
        how = "thermal" if pad.layers is None else "solid"
    return how


def relief(pad, zone) -> tuple[float, float]:
    """The thermal relief of ``pad`` in ``zone``: its gap and its spokes' width (the pad's own, else the zone's)."""
    return pad.thermal_gap or zone.gap or GAP, pad.spoke or zone.spoke or SPOKE


def _turn(pad) -> float:
    """The angle of ``pad`` on the board, in degrees: as the .kicad_pcb gives it, else along its longest side."""
    if pad.turn is not None:
        return pad.turn
    points = list(pad.shape.exterior.coords) if hasattr(pad.shape, "exterior") else []
    if pad.radius is not None or len(points) < 3:
        return 0.0
    a, b = max(zip(points, points[1:]), key=lambda e: math.dist(*e))
    return math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))


def spokes(pad, zone, least: float) -> list[tuple[object, Point]]:
    """The four spokes of the thermal relief of ``pad``, as KiCad draws them (``ZONE_FILLER::buildThermalSpokes``):
    each with the point at its end, which must be in the fill for KiCad to keep it. Square-ended, as wide as the
    relief asks but no wider than the pad; from the middle of the pad's box, in the pad's own frame, out to that box
    grown by the gap, KiCad's error and half the fill's ``least`` width; at the pad's spoke angle to its sides (45
    degrees for a round pad, else 90: along them). None if narrower than ``least``."""
    gap, width = relief(pad, zone)
    turn = _turn(pad)
    flat = affinity.rotate(pad.shape, -turn, origin=(0.0, 0.0))
    x0, y0, x1, y1 = flat.bounds
    hx, hy = (x1 - x0) / 2.0, (y1 - y0) / 2.0
    if pad.radius is not None:
        hx = hy = pad.radius
    width = min(width, 2.0 * min(hx, hy))
    if width < least - 1e-9:
        return []
    angle = pad.spoke_angle if pad.spoke_angle is not None else 45.0 if pad.radius is not None else 90.0
    grow = gap + ERROR + least / 2.0
    c, s = math.cos(math.radians(turn)), math.sin(math.radians(turn))
    mx, my = (x0 + x1) / 2.0, (y0 + y1) / 2.0
    cx, cy = mx * c - my * s, mx * s + my * c
    out = []
    for k in range(4):
        a = math.radians(angle + 90.0 * k)
        dx, dy = math.cos(a), math.sin(a)
        if pad.radius is not None:  # (KiCad turns a round pad's spokes, built along its box, rather than cutting them at it)
            reach = hx + grow
        else:
            reach = min((hx + grow) / abs(dx) if abs(dx) > 1e-9 else math.inf, (hy + grow) / abs(dy) if abs(dy) > 1e-9 else math.inf)
        ex, ey = cx + reach * (dx * c - dy * s), cy + reach * (dx * s + dy * c)
        out.append((LineString([(cx, cy), (ex, ey)]).buffer(width / 2.0, cap_style="flat"), Point(ex, ey)))
    return out


def _join(pours: list[Pour], same: dict | None = None) -> None:
    """Gives the pieces their kind. A pad through the board or a via is in a piece of each layer's pour of its net:
    pieces that share one (or pads whose copper meets: ``same``) are joined, and the joined pieces that hold a main piece are connected (KiCad sees the
    pad connect them); the rest hold pads that are cut off, or none."""
    for net in {pour.net for pour in pours}:
        pieces = [piece for pour in pours if pour.net == net for piece in pour.pieces]
        group = list(range(len(pieces)))

        def find(i: int) -> int:
            while group[i] != i:
                group[i] = group[group[i]]
                i = group[i]
            return i

        first: dict[int, int] = {}
        for i, piece in enumerate(pieces):
            for pad in {(same or {}).get(x, x) for x in piece.pad_ids}:
                group[find(i)] = find(first.setdefault(pad, i))
        joined = {find(i) for i, piece in enumerate(pieces) if piece.kind == "main"}
        for i, piece in enumerate(pieces):
            if piece.kind != "main":
                piece.kind = "via" if find(i) in joined and piece.pad_ids else "cut" if piece.pads else "island"


def of_result(result) -> list[Pour]:
    return estimate(result.board, result.polylines, result.wire_net, result.wire_layer, with_wedges(result.teardrops, result.wedges), widths=result.widths)


def note(pour: Pour, board) -> str:
    """One line: ``GND on B.Cu in 3 pieces: 2 pads cut off (U1-4, C3-2), 1 island``."""
    names = [*board.layers, *board.unrouted]
    cut, islands = pour.cut_off, pour.islands
    bits = []
    if cut:
        bits.append(f"{len(cut)} pad{'s' if len(cut) > 1 else ''} cut off ({', '.join(cut[:8])}{f' and {len(cut) - 8} more' if len(cut) > 8 else ''})")
    if islands:
        bits.append(f"{islands} island{'s' if islands > 1 else ''}")
    count = len(pour.pieces)
    return (f"{board.net_names.get(pour.net, pour.net)} on {names[pour.layer] if pour.layer < len(names) else pour.layer} in "
            f"{count} piece{'s' if count != 1 else ''}" + (": " + ", ".join(bits) if bits else ""))
