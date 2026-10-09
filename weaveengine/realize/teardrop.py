"""Teardrops where a trace meets a pad or via (post-processing, M8)."""
import math

import shapely
from shapely.geometry import LineString, Point, Polygon

from weaveengine.board import Board

LENGTH = 1.0       # teardrop length beyond the pad edge, in pad half-widths
WIDTH = 0.9        # teardrop width at the pad, as a fraction of the pad's width
MAX_LENGTH = 1.0   # mm: a big pad does not get a teardrop that reaches across the board
MAX_WIDTH = 2.0    # mm
BREATHING = 1.5    # a teardrop keeps this many clearances from foreign copper, or is left out


def _point_at(line: list[tuple[float, float]], dist: float):
    """Point at arc length ``dist`` from the start of the polyline, or None if it is shorter."""
    for (ax, ay), (bx, by) in zip(line, line[1:]):
        seg = math.hypot(bx - ax, by - ay)
        if dist <= seg:
            f = dist / seg if seg else 0.0
            return (ax + f * (bx - ax), ay + f * (by - ay))
        dist -= seg
    return None


def _tangent(t, tip, r: float, towards):
    """Point where the tangent from ``t`` touches the circle (tip, r), on the side of ``towards``."""
    dx, dy = tip[0] - t[0], tip[1] - t[1]
    d = math.hypot(dx, dy)
    if d <= r:
        return towards
    a = math.asin(r / d)
    best = None
    for sign in (1.0, -1.0):
        ca, sa = math.cos(sign * a), math.sin(sign * a)
        length = math.sqrt(d * d - r * r)
        q = (t[0] + (dx * ca - dy * sa) / d * length, t[1] + (dx * sa + dy * ca) / d * length)
        if best is None or math.dist(q, towards) < math.dist(best, towards):
            best = q
    return best


def tip_of(poly, half_width: float = 0.0):
    """Centre of the track where the teardrop starts (stored as the outline's sixth entry)."""
    return poly[5]


def _round(centre, radius: float, p, half_width: float, max_width: float = MAX_WIDTH):
    """Teardrop from the track point ``p`` to the tangent points on a circular pad."""
    cx, cy = centre
    ux, uy = p[0] - cx, p[1] - cy
    dist = math.hypot(ux, uy)
    r = min(WIDTH * radius, max_width / 2.0)
    if dist <= r or half_width >= r:
        return None
    ux, uy = ux / dist, uy / dist
    nx, ny = -uy, ux
    cos_a = r / dist
    sin_a = math.sqrt(1.0 - cos_a * cos_a)
    t1 = (cx + r * (cos_a * ux + sin_a * nx), cy + r * (cos_a * uy + sin_a * ny))
    t2 = (cx + r * (cos_a * ux - sin_a * nx), cy + r * (cos_a * uy - sin_a * ny))
    # The sides are tangent to the track's round end at p, so that traces of
    # the track's width fanned out from p stay inside the teardrop.
    return [centre, t1, _tangent(t1, p, half_width, (p[0] + half_width * nx, p[1] + half_width * ny)),
            _tangent(t2, p, half_width, (p[0] - half_width * nx, p[1] - half_width * ny)), t2, tuple(p)]


def _polygonal(centre, outline: list[tuple[float, float]], p, half_width: float, max_width: float = MAX_WIDTH,
               shrink: float = WIDTH):
    """Teardrop from the track point ``p`` to the two corners of a (convex) pad
    that bound it as seen from ``p``. The pad is shrunk to WIDTH first so the
    teardrop stays inside the pad's own footprint at the pad end."""
    cx, cy = centre
    ux, uy = cx - p[0], cy - p[1]
    dist = math.hypot(ux, uy)
    if dist < 1e-9:
        return None
    best_l = best_r = None
    for x, y in outline:
        x, y = cx + shrink * (x - cx), cy + shrink * (y - cy)
        ang = math.atan2(ux * (y - p[1]) - uy * (x - p[0]), ux * (x - p[0]) + uy * (y - p[1]))
        if best_l is None or ang > best_l[0]:
            best_l = (ang, (x, y))
        if best_r is None or ang < best_r[0]:
            best_r = (ang, (x, y))
    if best_l[0] <= 0 or best_r[0] >= 0:
        return None  # the track point is inside the pad
    # Seen from p towards the pad, positive angles are on the left: that is -n for n = rot90(p - c).
    nx, ny = uy / dist, -ux / dist
    left, right = (p[0] - half_width * nx, p[1] - half_width * ny), (p[0] + half_width * nx, p[1] + half_width * ny)
    width = math.dist(best_l[1], best_r[1])
    if width <= 2 * half_width:
        return None
    if width > max_width + 1e-9 and shrink > 1e-3:
        # Too wide for a large pad: attach to a smaller copy of the pad instead.
        return _polygonal(centre, outline, p, half_width, max_width, shrink * max_width / width * 0.999)
    return [centre, best_l[1], _tangent(best_l[1], p, half_width, left), _tangent(best_r[1], p, half_width, right), best_r[1], tuple(p)]


def teardrops(board: Board, polylines: dict[int, list[tuple[float, float]]], wire_net: dict[int, int],
              wire_pads: dict[int, tuple[int, int]], layer: int = 0, max_length: float = MAX_LENGTH,
              max_width: float = MAX_WIDTH, breathing: float = BREATHING) -> dict[int, list[list[tuple[float, float]]]]:
    """wire id -> teardrops, each [pad centre, pad point, track left, track right, pad point, tip]:
    the first five are the outline, the sixth is the centre of the track where the teardrop starts.

    Round pads get tangent lines to the pad circle; other convex pads get lines
    to the two pad corners that bound the pad as seen from the trace.

    Teardrops are added after routing, so they never take space a trace could
    have used. Each is limited to ``max_length`` beyond the pad and
    ``max_width`` across, and what it adds to the pad keeps ``breathing``
    clearances from foreign copper and traces if it can. Where things are
    cramped it is shortened; then it keeps the plain clearance, like a trace;
    then it is made narrower; and only then left out. It always keeps the full
    clearance from the board edge.
    """
    roomy = board.rules.clearance * max(1.0, breathing)
    proud = 0.0
    rules = board.rules
    pads = {p.pad_id: p for p in board.pads_on(layer)}
    copper = [(p.shape, p.net_id) for p in pads.values()] + [(o.shape, o.net_id) for o in board.obstacles_on(layer)]
    ids = list(polylines)
    lines = [LineString(polylines[w]) for w in ids]
    tree_c = shapely.STRtree([c[0] for c in copper]) if copper else None
    tree_w = shapely.STRtree(lines) if lines else None
    edge = rules.outline_inset - rules.base_width / 2.0
    out: dict[int, list] = {}
    for w, line in polylines.items():
        net = wire_net[w]
        half = rules.width(net) / 2.0
        for pad_id, pts in zip(wire_pads[w], (line, line[::-1])):
            pad = pads.get(pad_id)
            if pad is None:
                continue
            if pad.radius is not None:
                reach = size = pad.radius
            else:
                # Where the trace leaves the pad, and half the pad's smaller dimension (its inscribed radius).
                leave = LineString(pts).intersection(pad.shape.exterior)
                if leave.is_empty:
                    continue
                reach = max(Point(pad.centre).distance(g) for g in getattr(leave, "geoms", [leave]))
                size = pad.shape.exterior.distance(Point(pad.centre))
            # As large as there is room for, and a small one rather than none:
            # shorter, then with no more than the clearance every trace keeps, then narrower.
            for length, widest, room in ((LENGTH, max_width, roomy), (LENGTH / 2.0, max_width, roomy),
                                         (LENGTH / 2.0, max_width, rules.clearance), (LENGTH / 2.0, min(max_width, 2.0 * size) / 2.0, rules.clearance)):
                p = _point_at(pts, reach + min(length * size, max_length))
                if p is None:
                    continue
                if pad.radius is not None:
                    poly = _round(pad.centre, pad.radius, p, half, widest)
                else:
                    poly = _polygonal(pad.centre, list(pad.shape.exterior.coords)[:-1], p, half, widest)
                if poly is None:
                    continue
                shape = Polygon(poly[:5])
                if not shape.is_valid or not board.outline.contains(shape) or shape.distance(board.outline.exterior) < edge - 1e-6:
                    continue
                # What has to keep its distance is the copper the teardrop adds:
                # its part over the pad is the pad's, which stands where it
                # stands. (Tested whole, a via at the least clearance from a
                # neighbour lost every teardrop on that side.)
                added = shape.difference(pad.shape)
                if added.is_empty:
                    continue
                clear = True
                if tree_c is not None:
                    for c in tree_c.query(added, predicate="dwithin", distance=room + proud * 2 * half - 1e-6).tolist():
                        if copper[c][1] != net:
                            clear = False
                for j in (tree_w.query(added, predicate="dwithin", distance=room + max(rules.net_width.values(), default=rules.trace_width)).tolist() if clear else []):
                    other = ids[j]
                    if wire_net[other] != net and added.distance(lines[j]) < room + proud * 2 * half + rules.width(wire_net[other]) / 2.0 - 1e-6:
                        clear = False
                if clear:
                    out.setdefault(w, []).append(poly)
                    break
    return out
