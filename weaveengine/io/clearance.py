"""The copper a session holds, to check a curve against before it is written
in place of part of a trace (``io.arcs.fit_arcs``).

An arc fitted to a run of a trace lies inside the run, nearer the thing the
trace goes round. It may come nearer other copper than the run did only where
that copper lies between the run and what it goes round. ``Copper.allows``
measures that against the true shapes (round pads as circles, not the polygons
the router holds them as), the traces as they are written so far, the
teardrops and wedges, the keepouts and the board edge.
"""
import math

import shapely
from shapely.geometry import LineString, Point, Polygon

CELL = 1.0  # mm: the grid round pads are found in
SLACK = 1e-4  # mm: below what the session can hold (its unit is commonly 0.1 micron), and a hundredth of the margin


class Copper:
    """Per layer: (shape, net or None for no net, how far its copper reaches
    beyond the shape, the gap a trace keeps from it). Traces are keyed so that
    the writer can put in what it writes in their place (``replace``): a fitted
    arc only moves a trace inwards, within the box it had, so the tree built
    over the drawn traces still finds it."""

    def __init__(self, board, result, teardrops: bool = True):
        rules = board.rules
        edge = rules.clearance if rules.edge_clearance is None else rules.edge_clearance
        self.items: dict[int, list] = {}
        self.trace_at: dict[tuple, tuple[int, int]] = {}
        self.round = {}  # layer -> grid cell -> centres of round pads, vias and pad ends there (``near``)
        parts = {}
        for layer in range(len(board.layers)):
            items = []
            for pad in board.pads_on(layer):
                if pad.pad_id not in parts:
                    parts[pad.pad_id] = [(pad.centre, pad.radius)] if pad.radius else round_parts(pad.shape)
                shape, extent = pad_copper(pad, parts[pad.pad_id])
                items.append([shape, pad.net_id, extent, rules.apart(pad.clearance)])
                for (x, y), _ in parts[pad.pad_id]:
                    self.round.setdefault(layer, {}).setdefault((math.floor(x / CELL), math.floor(y / CELL)), []).append((x, y))
            for obs in board.obstacles_on(layer):
                items.append([obs.shape, obs.net_id if obs.net_id >= 0 else None, 0.0, rules.apart(obs.clearance)])
            outline = board.outline
            items.append([shapely.boundary(outline), None, 0.0, edge])
            self.items[layer] = items
        for wire, line in result.polylines.items():
            net, layer = result.wire_net[wire], result.wire_layer.get(wire, 0)
            items = self.items.setdefault(layer, [])
            for piece, (pts, width) in enumerate(result.widths.get(wire) or [(line, rules.drawn(net))]):
                if len(pts) >= 2:
                    self.trace_at[wire, piece] = (layer, len(items))
                    line = LineString(pts)
                    items.append([line, net, width / 2.0, rules.clearance, line])  # (as written so far, and as drawn)
            shapes = list(result.wedges.get(wire, []))
            if teardrops:
                shapes += [poly[:5] for poly in result.teardrops.get(wire, [])]
            for poly in shapes:
                shape = Polygon(poly).buffer(0)
                if not shape.is_empty:
                    items.append([shape, net, 0.0, rules.clearance])
        self.trees = {layer: shapely.STRtree([item[0] for item in items]) for layer, items in self.items.items()}
        self.reach = max((item[2] + item[3] for items in self.items.values() for item in items), default=0.0)

    def allows(self, layer: int, net: int, half: float, old, new) -> bool:
        """Whether the trace (of ``net``, half as wide as ``half``) drawn along
        ``new`` instead of ``old`` keeps from all copper of other nets what
        that copper asks, or at least what ``old`` kept from it as drawn."""
        if layer not in self.trees:
            return True
        old_line, new_line = LineString(old), LineString(new)
        x0, y0, x1, y1 = old_line.bounds
        far = self.reach + half
        items = self.items[layer]
        for k in self.trees[layer].query(shapely.box(x0 - far, y0 - far, x1 + far, y1 + far)):
            shape, other, extent, need = items[k][:4]
            if other is not None and other == net and net >= 0:
                continue
            gap = new_line.distance(shape) - extent - half
            if gap >= need - SLACK:
                continue
            # (what the drawn traces kept from each other: measured from what this
            # one is now, an exception would ratchet down, a trace at a time)
            if gap < old_line.distance(items[k][4] if len(items[k]) > 4 else shape) - extent - half - SLACK:
                return False
        return True

    def near(self, layer: int):
        """``near(point, distance)``: the centres of round pads and vias on the layer within ``distance`` of the point."""
        cells = self.round.get(layer, {})

        def near(p, distance):
            x0, x1 = math.floor((p[0] - distance) / CELL), math.floor((p[0] + distance) / CELL)
            y0, y1 = math.floor((p[1] - distance) / CELL), math.floor((p[1] + distance) / CELL)
            return [c for gx in range(x0, x1 + 1) for gy in range(y0, y1 + 1) for c in cells.get((gx, gy), ())
                    if math.dist(c, p) <= distance]
        return near

    def replace(self, wire: int, piece: int, pts) -> None:
        """The trace piece as written, for the checks of the traces after it."""
        if (wire, piece) in self.trace_at and len(pts) >= 2:
            layer, k = self.trace_at[wire, piece]
            self.items[layer][k][0] = LineString(pts)


def pad_copper(pad, parts=None):
    """A pad's true copper as (shape, how far it reaches beyond the shape): a
    round pad is its centre and radius; an oval (held as a polygon round it,
    ``io.dsn``) the line between its round ends and their radius; any other
    pad its polygon."""
    if pad.radius:
        return Point(pad.centre), pad.radius
    ends = round_parts(pad.shape) if parts is None else parts
    if len(ends) == 2 and math.isclose(ends[0][1], ends[1][1], rel_tol=1e-6) and _oval(pad.shape, ends):
        return LineString([ends[0][0], ends[1][0]]), ends[0][1]
    return pad.shape, 0.0


def round_parts(shape) -> list[tuple[tuple[float, float], float]]:
    """The round parts of a pad's outline, as (centre, radius): runs of three
    or more corners that turn alike, with sides of one length (an oval's ends,
    held as ``board.circle_polygon`` holds a circle, and a rounded rectangle's
    corners). The radius is to the middle of the sides (the true circle of a
    circumscribed polygon)."""
    pts = list(shape.exterior.coords)[:-1] if shape.geom_type == "Polygon" else []
    n = len(pts)
    if n < 5:
        return []
    turn, side = [], []
    for k in range(n):
        a, b, c = pts[k - 1], pts[k], pts[(k + 1) % n]
        h1, h2 = math.atan2(b[1] - a[1], b[0] - a[0]), math.atan2(c[1] - b[1], c[0] - b[0])
        turn.append((h2 - h1 + math.pi) % (2.0 * math.pi) - math.pi)
        side.append(math.dist(b, c))

    def alike(k, m):
        return (math.radians(1.0) < abs(turn[k]) < math.radians(46.0) and abs(turn[k] - turn[m]) < 1e-3
                and abs(side[k] - side[m]) <= 1e-3 * side[m])

    start = next((k for k in range(n) if not alike(k, k - 1)), None)
    if start is None:
        return []
    out, k = [], start
    while k < start + n:
        m = k + 1
        while m < start + n and alike(m % n, (m - 1) % n):
            m += 1
        if m - k >= 3 and alike(k % n, k % n):
            a, b, c = pts[k % n], pts[(k + 1) % n], pts[(k + 2) % n]
            centre = _circumcentre(a, b, c)
            if centre is not None:
                out.append((centre, math.dist(centre, b) * math.cos(turn[(k + 1) % n] / 2.0)))
        k = m
    return out


def _circumcentre(a, b, c):
    d = 2.0 * (a[0] * (b[1] - c[1]) + b[0] * (c[1] - a[1]) + c[0] * (a[1] - b[1]))
    if abs(d) < 1e-15:
        return None
    sa, sb, sc = a[0] ** 2 + a[1] ** 2, b[0] ** 2 + b[1] ** 2, c[0] ** 2 + c[1] ** 2
    return ((sa * (b[1] - c[1]) + sb * (c[1] - a[1]) + sc * (a[1] - b[1])) / d,
            (sa * (c[0] - b[0]) + sb * (a[0] - c[0]) + sc * (b[0] - a[0])) / d)


def _oval(shape, ends) -> bool:
    """Whether ``shape`` is the polygon an oval with these round ends is held as (``io.dsn``): it holds the oval, within a few microns of it."""
    (a, r), (b, _) = ends
    true = LineString([a, b]).buffer(r, quad_segs=32)
    return shape.buffer(1e-6).contains(true) and shape.area - true.area < 0.05 * true.area


def outline_of(runs) -> list[tuple[float, float]]:
    """The runs of ``io.arcs.fit_arcs`` as one polyline, the arcs sampled a degree at a time."""
    from weaveengine.io.arcs import sampled
    out = []
    for run in runs:
        if run[0] == "path":
            pts = run[1]
        else:
            centre = run[2]
            pts = []
            for a, b in run[1]:
                radius = math.dist(a, centre)
                a1, a2 = math.atan2(a[1] - centre[1], a[0] - centre[0]), math.atan2(b[1] - centre[1], b[0] - centre[0])
                pts += sampled(a, b, centre, radius, (a2 - a1 + math.pi) % (2.0 * math.pi) - math.pi)
        for p in pts:
            if not out or p != out[-1]:
                out.append(p)
    return out
