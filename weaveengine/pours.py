"""An estimate of the copper KiCad will fill in each pour of a routing (design section 11.3):
the pour's outline less every other net's copper and its clearance, without the slivers, in
pieces; the piece with most of the net's pads is the main one, and the others hold pads that
are cut off, or none (islands)."""
import math
from dataclasses import dataclass, field

import shapely
from shapely.geometry import LineString, Point



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
             clearance: float | None = None, min_width: float | None = None) -> list[Pour]:
    """The estimated fill of each pour of ``board`` (its vias among its pads) for the routed
    ``lines``. ``clearance``: the zones' (default: each zone's own, or the board's less the routing
    margin, whichever is more: ``Board.pour_clearance``). ``min_width``: the narrowest fill (default:
    each zone's own minimum thickness, ``Board.pour_width``). A pad the zone joins by a thermal relief
    is joined only where one of its spokes reaches the fill (``spokes``); where zones of two nets
    overlap, the one of higher priority fills first and the other keeps clear of its fill."""
    rules = board.rules
    base = rules.clearance - rules.margin if clearance is None else clearance
    edge = (rules.clearance if rules.edge_clearance is None else rules.edge_clearance) - rules.margin
    inside = board.outline.buffer(-max(edge, base))
    layers = {at for _, at, _ in board.pours}
    # Each layer's copper: (shape, net, clearance asked; None for a keepout, which asks none).
    copper: dict[int, list] = {at: [] for at in layers}
    for p in board.pads:
        for at in layers:
            if p.on(at):
                copper[at].append((p.shape, p.net_id, max(base, p.clearance)))
    for o in board.obstacles:
        for at in layers:
            if o.on(at):
                copper[at].append((o.shape, o.net_id if o.copper or o.net_id >= 0 else -2, max(base, o.clearance) if o.copper or o.net_id >= 0 else 0.0))
    for w, line in lines.items():
        if len(line) >= 2 and wire_layer.get(w) in layers:
            net = wire_net[w]
            copper[wire_layer[w]].append((LineString(line).buffer(rules.drawn(net) / 2.0, quad_segs=4), net, base))
    for w, drops in (teardrops or {}).items():
        if wire_layer.get(w) in layers:
            for poly in drops:
                copper[wire_layer[w]].append((shapely.Polygon(poly[:5]).buffer(0), wire_net[w], base))
    trees = {at: shapely.STRtree([c[0] for c in cs]) for at, cs in copper.items() if cs}
    # Higher priority first: a zone of another net that it overlaps is filled already (KiCad's order).
    order = sorted(range(len(board.pours)), key=lambda k: (-board.zone(*board.pours[k][:2]).priority, k))
    filled: dict[int, list] = {}  # layer -> (net, priority, fill) of the pours done
    found: dict[int, Pour] = {}
    for k in order:
        net, at, outline = board.pours[k]
        zone = board.zone(net, at)
        keep = base if clearance is not None else max(base, board.pour_clearance(net, at))
        fill = outline.intersection(inside)
        tree = trees.get(at)
        if tree is not None:
            near = [copper[at][i] for i in tree.query(outline, predicate="dwithin", distance=keep + 1.0).tolist()]
            held = [s.buffer(max(ask, keep) if ask else ask, quad_segs=4) for s, other, ask in near if other != net]
            if held:
                fill = fill.difference(shapely.union_all(held))
        above = [f.buffer(keep, quad_segs=4) for other, rank, f in filled.get(at, ()) if other != net and rank > zone.priority]
        if above:
            fill = fill.difference(shapely.union_all(above))
        own = [p for p in board.pads if p.net_id == net and p.on(at) and p.shape.intersects(outline)]
        how = {p.pad_id: joined(p, zone) for p in own}
        relieved = [p for p in own if how[p.pad_id] in ("thermal", "none")]
        if relieved:  # (the fill keeps its gap from a pad it joins by spokes, and its clearance from one it does not join)
            fill = fill.difference(shapely.union_all([p.shape.buffer(relief(p, zone)[0] if how[p.pad_id] == "thermal" else keep, quad_segs=4)
                                                      for p in relieved]))
        least = board.pour_width(net, at) if min_width is None else min_width
        fill = fill.buffer(-least / 2.0).buffer(least / 2.0)
        # The net's own pads are copper whatever their width; one joined by a thermal relief, only with the spokes that reach the fill.
        joins = [p.shape.intersection(inside) for p in own if how[p.pad_id] == "solid"]
        thermal = [p for p in own if how[p.pad_id] == "thermal"]
        index = shapely.STRtree([p.shape for p in thermal]) if thermal else None
        for p in thermal:
            reach = relief(p, zone)[0] * 2.0 + 0.5 * max(p.shape.bounds[2] - p.shape.bounds[0], p.shape.bounds[3] - p.shape.bounds[1])
            near = [thermal[i] for i in index.query(p.shape, predicate="dwithin", distance=reach).tolist() if thermal[i] is not p]
            joins += [p.shape.intersection(inside), *spokes(p, zone, fill, near)]
        fill = shapely.union_all([fill, *joins])
        filled.setdefault(at, []).append((net, zone.priority, fill))
        own = [p for p in own if how[p.pad_id] != "none"]
        parts = sorted(getattr(fill, "geoms", [fill]), key=lambda g: -g.area)
        pour = Pour(net, at, outline)
        has = [[p for p in own if g.intersects(p.shape)] for g in parts]
        main = max(range(len(parts)), key=lambda k: (sum(not p.is_via for p in has[k]), parts[k].area), default=None)
        for j, g in enumerate(parts):
            if g.geom_type == "Polygon" and not g.is_empty:
                names = [p.name or str(p.pad_id) for p in has[j] if not p.is_via]
                pour.pieces.append(Piece(g, "main" if j == main and names else "", names, {p.pad_id for p in has[j]}))
        found[k] = pour
    pours = [found[k] for k in range(len(board.pours))]
    _join(pours, _touching(board, {pour.net for pour in pours}))
    return pours


def _touching(board, nets: set) -> dict:
    """Pad id -> one pad of the group of the net's pads whose copper meets on a layer they share (a pad through
    the board inside a part's surface pad, say): KiCad sees them joined, on any layer."""
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
    return {k: root(k) for k in group}


GAP, SPOKE = 0.5, 0.5  # mm: KiCad's default thermal relief gap and spoke width
TIP = 0.01             # mm past the gap where a spoke's end is looked for in the fill


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


def spokes(pad, zone, fill, others=()) -> list:
    """The spokes of the thermal relief of ``pad`` that reach ``fill``: four, along the
    pad's sides (at 45 degrees to them for a round pad, as KiCad draws them by default),
    each from the pad across its gap; one is kept if its end, just past the gap, is in the fill,
    or in the relief of another of ``others`` (pads of the net packed closer than their gaps: KiCad
    joins a spoke that ends in another), which it then joins."""
    gap, width = relief(pad, zone)
    cx, cy = pad.centre
    points = list(pad.shape.exterior.coords) if hasattr(pad.shape, "exterior") else []
    turn = math.pi / 4.0
    if pad.radius is None and len(points) > 2:  # (along its longest side)
        a, b = max(zip(points, points[1:]), key=lambda e: math.dist(*e))
        turn = math.atan2(b[1] - a[1], b[0] - a[0])
    out = []
    for k in range(4):
        dx, dy = math.cos(turn + k * math.pi / 2.0), math.sin(turn + k * math.pi / 2.0)
        reach = max(((x - cx) * dx + (y - cy) * dy for x, y in points), default=0.0)
        end = reach + gap + width / 2.0
        tip = Point(cx + dx * (reach + gap + TIP), cy + dy * (reach + gap + TIP))
        if fill.distance(tip) > 1e-6:
            near = next((q for q in others if q.shape.distance(tip) <= relief(q, zone)[0]), None)
            if near is None:
                continue
            out.append(LineString([(cx, cy), (tip.x, tip.y), near.centre]).buffer(width / 2.0, cap_style="flat"))
            continue
        out.append(LineString([(cx, cy), (cx + dx * end, cy + dy * end)]).buffer(width / 2.0, cap_style="flat"))
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
    return estimate(result.board, result.polylines, result.wire_net, result.wire_layer, result.teardrops)


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
