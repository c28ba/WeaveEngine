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
    reach: tuple | None = None                      # how the copper of its net hangs together (``reach``)
    left: list = field(default_factory=list)        # the groups of its net's pads (ids) not joined to the rest: the same on each pour of the net

    @property
    def cut_off(self) -> list[str]:
        return [name for piece in self.pieces if piece.kind == "cut" for name in piece.pads]

    @property
    def islands(self) -> int:
        return sum(1 for piece in self.pieces if piece.kind == "island")


def unjoined(pours: list[Pour]) -> list[list[int]]:
    """The groups of pads (their ids) of the pours' nets not joined to the rest of their net: one item each that
    KiCad finds left unconnected (``Pour.left``)."""
    seen = {}
    for pour in pours:
        seen.setdefault(pour.net, pour.left)
    return [group for net in sorted(seen) for group in seen[net]]


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
    routing's copper, if known already; ``window``: only the fill within this shape (``refill``). The other zones
    (``Board.areas``: teardrops, copper on one pad) are filled as KiCad fills them, and not returned."""
    rules = board.rules
    base = rules.clearance - rules.margin if clearance is None else clearance
    inside = pour_area(board, clearance)
    zones = board.pours + board.areas  # (the pours first: their index is theirs in ``board.pours``)
    # Higher priority first: a zone of another net that it overlaps is filled already (KiCad's order).
    rank = lambda k: board.zone(*zones[k][:2]).priority
    order = sorted(range(len(zones)), key=lambda k: (-rank(k), k))
    if nets is not None:
        want = {k for k in order if k < len(board.pours) and zones[k][0] in nets}
        more = want
        while more:
            more = {j for j in order if j not in want for k in more
                    if zones[j][1] == zones[k][1] and zones[j][0] != zones[k][0] and rank(j) > rank(k)}
            want |= more
        order = [k for k in order if k in want]
    layers = {zones[k][1] for k in order}
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
    pads_of: dict[int, list] = {}
    for p in board.pads:
        pads_of.setdefault(p.net_id, []).append(p)
    wires_of: dict[tuple, list] = {}
    for shape, net, at in wires:
        wires_of.setdefault((net, at), []).append(shape)
    filled: dict[int, list] = {}  # layer -> (net, priority, fill, its bounds) of the zones done
    made: dict = {}  # (what, by how much) -> it grown so: a thing near many zones is grown once

    def grow(key, shape, by: float, bounds):
        """``shape`` grown by ``by``, where it matters to an area within ``bounds``: a long trace or a large fill
        is cut to them first (a board may have hundreds of small zones, teardrops, each beside it)."""
        x0, y0, x1, y1 = bounds
        a0, b0, a1, b1 = shape.bounds
        if a0 < x0 - by - 1.0 or b0 < y0 - by - 1.0 or a1 > x1 + by + 1.0 or b1 > y1 + by + 1.0:
            return shape.intersection(shapely.box(x0 - by - 0.1, y0 - by - 0.1, x1 + by + 0.1, y1 + by + 0.1)).buffer(by, quad_segs=4)
        key = (key, round(by, 9))
        if key not in made:
            made[key] = shape.buffer(by, quad_segs=4)
        return made[key]

    found: dict[int, object] = {}
    for k in order:
        net, at, outline = zones[k]
        zone = board.zone(net, at)
        own_zone = (board.zone(net, at).clearance or 0.0) if clearance is None else 0.0
        keep = max(asks(net), own_zone)  # (and from another net's copper, the more of the two classes)
        pair = lambda other: max(keep, asks(other))
        area = outline.intersection(inside)
        if window is not None:
            area = area.intersection(window)
        if k >= len(board.pours) and area.is_empty:
            continue
        fill = area
        tree = trees.get(at)
        # What other nets' copper and zones take, which the spokes keep clear of too (KiCad's order: the reliefs out, the
        # clearances out, the spokes in, the clearances out again, then what is narrower than the least width).
        blocked = []
        reach = max(base, keep) + own_zone + 1.0
        bounds = area.bounds
        if tree is not None:
            for i in tree.query(area, predicate="dwithin", distance=reach).tolist():
                s, other, ask = copper[at][i]
                if other != net:
                    blocked.append(grow(("copper", at, i), s, max(ask, pair(other)) if ask is not None else 0.0, bounds))
        x0, y0, x1, y1 = bounds
        blocked += [grow(("zone", at, j), f, pair(other), bounds) for j, (other, rank_, f, (a0, b0, a1, b1)) in enumerate(filled.get(at, ()))
                    if other != net and rank_ > zone.priority and a0 < x1 + reach and x0 < a1 + reach and b0 < y1 + reach and y0 < b1 + reach]
        # (what lies beyond the area's box takes nothing from it: cut there first, then joined, the cheaper)
        blocked = shapely.union_all(shapely.intersection(blocked, shapely.box(x0 - EPSILON, y0 - EPSILON, x1 + EPSILON, y1 + EPSILON)))             if blocked else None
        if blocked is not None:
            fill = fill.difference(blocked)
        own = [p for p in pads_of.get(net, ()) if p.on(at) and p.shape.intersects(outline)
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
        joins += [shape for shape in wires_of.get((net, at), ()) if shape.intersects(outline)
                  and (window is None or shape.intersects(window))]
        fill = shapely.union_all([fill, *joins])
        filled.setdefault(at, []).append((net, zone.priority, fill, fill.bounds))
        if k < len(board.pours):
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


def piece_key(pour, piece) -> tuple:
    """A piece's name in ``reach``'s groups."""
    return ("piece", pour.layer) + tuple(round(v, 6) for v in piece.shape.bounds) + (round(piece.shape.area, 6),)


def reach(board, pours: list, wires: list, net: int) -> tuple[dict, list, object]:
    """How the copper of ``net`` hangs together, as KiCad sees it: its pads and vias, the pieces of its pours' fill
    (``pours``), its traces and teardrops (``wires``: shape, net, layer) and its fixed copper, joined where they meet
    on a layer they share. Returns each thing's group (("pad", id), ``_piece``, ("fixed", id) -> its root), the pads
    (not vias) not in the main group, and the main group's root: of the groups with fill in them, the one with most
    pads (None if the net has no fill at all: every pad is left)."""
    pads = [p for p in board.pads if p.net_id == net]
    items = [(("pad", p.pad_id), p.shape, p.layers, 0 if p.is_via else 1) for p in pads]
    items += [(("wire", k), shape, frozenset({layer}), 0) for k, (shape, other, layer) in enumerate(wires) if other == net]
    items += [(("fixed", id(o)), o.shape, o.layers, 0) for o in board.obstacles if o.net_id == net and not o.copper]
    small = len(items)  # (pads, traces and fixed copper: what meets is found by shape; the pieces of fill, by what is in them)
    items += [(piece_key(pour, x), x.shape, frozenset({pour.layer}), 0) for pour in pours if pour.net == net for x in pour.pieces]
    group = list(range(len(items)))

    def find(i: int) -> int:
        while group[i] != i:
            group[i] = group[group[i]]
            i = group[i]
        return i

    if small:
        a, b = shapely.STRtree([it[1] for it in items[:small]]).query([it[1] for it in items[:small]], predicate="intersects")
        for i, j in zip(a.tolist(), b.tolist()):
            la, lb = items[i][2], items[j][2]
            if i < j and (la is None or lb is None or la & lb):
                group[find(i)] = find(j)
    # A piece holds the pads its pour joins (``Piece.pad_ids``), and the traces and fixed copper of the net in its pour,
    # which the fill takes in whole (a point of each tells which piece).
    index = {it[0]: i for i, it in enumerate(items[:small])}
    by_layer: dict = {}
    for i in range(small, len(items)):
        by_layer.setdefault(next(iter(items[i][2])), []).append(i)
    i = small
    for pour in pours:
        if pour.net != net:
            continue
        for x in pour.pieces:
            for pad in x.pad_ids:
                if ("pad", pad) in index:
                    group[find(i)] = find(index["pad", pad])
            i += 1
    for at, rows in by_layer.items():
        tree = shapely.STRtree([items[i][1] for i in rows])
        inner = [k for k in range(small) if items[k][0][0] != "pad" and (items[k][2] is None or at in items[k][2])]
        if not inner:
            continue
        spots = shapely.points([items[k][1].representative_point().coords[0] for k in inner])
        found, hit = tree.query(spots, predicate="intersects")
        for f, h in zip(found.tolist(), hit.tolist()):
            group[find(inner[f])] = find(rows[h])
    held: dict = {}  # root -> (pads, fill)
    for i, it in enumerate(items):
        count, fill = held.get(find(i), (0, 0.0))
        held[find(i)] = (count + it[3], fill + (it[1].area + 1e-9 if it[0][0] == "piece" else 0.0))
    filled = [root for root, (_, fill) in held.items() if fill > 0]
    main = max(filled, key=lambda root: (held[root][0], held[root][1], -root)) if filled else None
    comp = {it[0]: find(i) for i, it in enumerate(items)}
    lost = [p for p in pads if not p.is_via and (main is None or comp[("pad", p.pad_id)] != main)]
    return comp, lost, main


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
    # A piece is joined if it hangs together with the net's main group (``reach``): through pads and vias, the net's
    # traces, fixed copper, and the pieces of its other pours.
    for net in {pour.net for pour in pours}:
        mine = [pour for pour in pours if pour.net == net]
        found = reach(board, mine, wires, net)
        comp, _, main = found
        padded = {comp["pad", p.pad_id] for p in board.pads if p.net_id == net and not p.is_via}
        if main not in padded:
            main = None  # (a plane no pad is joined to: islands, to KiCad)
        groups: dict = {}
        for p in found[1]:
            groups.setdefault(comp["pad", p.pad_id], []).append(p.pad_id)
        left = sorted(groups.values(), key=lambda g: (-len(g), min(g)))
        for pour in mine:
            pour.reach, pour.left = found, left if main is not None else left[1:]  # (no main group: all but one need joining)
            for x in pour.pieces:
                group = comp[piece_key(pour, x)]
                x.kind = ("main" if x.kind == "main" else "via") if group == main else "cut" if group in padded else "island"
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


GAP, SPOKE = 0.5, 0.5  # mm: KiCad's default thermal relief gap and spoke width
ERROR = 0.005          # mm: KiCad's default maximum error, which its spokes reach past the gap
EPSILON = 0.001        # mm: what KiCad's fill leaves of the least width, so that copper just that wide stays
HELD = 0.65           # of a spoke beyond its pad in a piece of fill: it joins the pad to that piece (fills cut some spokes short)


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


def starved(board, pour: Pour, pad, wires: list, traces=()) -> bool:
    """Whether KiCad's check of thermal reliefs finds ``pad`` starved in ``pour`` (its pieces, ``classify``): fewer
    spokes join it to the fill than the board asks (``Board.min_spokes``). As KiCad counts them
    (``DRC_TEST_PROVIDER_ZONE_CONNECTIONS``): its spokes that the fill keeps; those into a piece that meets nothing
    else of the net (``wires``: shape, net, layer) only if there are no others; and, if still too few, the traces of
    its net on the pour's layer (``traces``: (layer, points)) from inside the pad to a point in the fill. A pad no
    spoke reaches at all is not checked: it is not joined (or by traces alone)."""
    zone = board.zone(pour.net, pour.layer)
    if pad.net_id != pour.net or pad.is_via or not pad.on(pour.layer) or joined(pad, zone) != "thermal":
        return False
    real = alone = 0
    for shape, _ in spokes(pad, zone, board.pour_width(pour.net, pour.layer)):
        out = shape.difference(pad.shape)
        if out.area <= 0:
            continue
        piece = next((x for x in pour.pieces if x.shape.intersection(out).area >= HELD * out.area), None)
        if piece is None:
            continue
        lone = piece.pad_ids <= {pad.pad_id} and not any(
            net == pour.net and at == pour.layer and piece.shape.intersects(shape_) for shape_, net, at in wires)
        alone += lone
        real += not lone
    if real == alone == 0 or real >= board.min_spokes:
        return False
    fill = shapely.union_all([x.shape for x in pour.pieces]) if traces else None
    for layer, pts in traces:
        if layer == pour.layer:
            a, b = Point(pts[0]), Point(pts[-1])
            real += (pad.shape.contains(a) and fill.contains(b)) or (pad.shape.contains(b) and fill.contains(a))
    return (real or alone) < board.min_spokes


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
