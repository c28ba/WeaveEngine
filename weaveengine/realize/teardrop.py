"""Teardrops where a trace meets a pad or via (post-processing, M8)."""
import math

import shapely
from shapely.geometry import LineString, Point, Polygon

from weaveengine.board import Board

LENGTH = 1.0   # teardrop length beyond the pad edge, in pad half-widths
WIDTH = 0.9    # teardrop width at the pad, as a fraction of the pad's width


def _point_at(line: list[tuple[float, float]], dist: float):
    """Point at arc length ``dist`` from the start of the polyline, or None if it is shorter."""
    for (ax, ay), (bx, by) in zip(line, line[1:]):
        seg = math.hypot(bx - ax, by - ay)
        if dist <= seg:
            f = dist / seg if seg else 0.0
            return (ax + f * (bx - ax), ay + f * (by - ay))
        dist -= seg
    return None


def _round(centre, radius: float, p, half_width: float):
    """Teardrop from the track point ``p`` to the tangent points on a circular pad."""
    cx, cy = centre
    ux, uy = p[0] - cx, p[1] - cy
    dist = math.hypot(ux, uy)
    r = WIDTH * radius
    if dist <= r or half_width >= r:
        return None
    ux, uy = ux / dist, uy / dist
    nx, ny = -uy, ux
    cos_a = r / dist
    sin_a = math.sqrt(1.0 - cos_a * cos_a)
    t1 = (cx + r * (cos_a * ux + sin_a * nx), cy + r * (cos_a * uy + sin_a * ny))
    t2 = (cx + r * (cos_a * ux - sin_a * nx), cy + r * (cos_a * uy - sin_a * ny))
    return [centre, t1, (p[0] + half_width * nx, p[1] + half_width * ny),
            (p[0] - half_width * nx, p[1] - half_width * ny), t2]


def _polygonal(centre, outline: list[tuple[float, float]], p, half_width: float):
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
        x, y = cx + WIDTH * (x - cx), cy + WIDTH * (y - cy)
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
    if math.dist(best_l[1], best_r[1]) <= 2 * half_width:
        return None
    return [centre, best_l[1], left, right, best_r[1]]


def teardrops(board: Board, polylines: dict[int, list[tuple[float, float]]], wire_net: dict[int, int],
              wire_pads: dict[int, tuple[int, int]], layer: int = 0) -> dict[int, list[list[tuple[float, float]]]]:
    """wire id -> teardrop polygons [pad centre, pad point, track left, track right, pad point].

    Round pads get tangent lines to the pad circle; other convex pads get lines
    to the two pad corners that bound the pad as seen from the trace. A
    teardrop is shortened, then dropped, if it would come within the clearance
    of foreign copper, a foreign trace or the board edge.
    """
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
            for length in (LENGTH, LENGTH / 2.0):
                p = _point_at(pts, reach + length * size)
                if p is None:
                    continue
                if pad.radius is not None:
                    poly = _round(pad.centre, pad.radius, p, half)
                else:
                    poly = _polygonal(pad.centre, list(pad.shape.exterior.coords)[:-1], p, half)
                if poly is None:
                    continue
                shape = Polygon(poly)
                if not shape.is_valid or not board.outline.contains(shape) or shape.distance(board.outline.exterior) < edge - 1e-6:
                    continue
                clear = True
                if tree_c is not None:
                    for c in tree_c.query(shape, predicate="dwithin", distance=rules.clearance - 1e-6).tolist():
                        if copper[c][1] != net:
                            clear = False
                for j in (tree_w.query(shape, predicate="dwithin", distance=rules.clearance + rules.base_width).tolist() if clear else []):
                    other = ids[j]
                    if wire_net[other] != net and shape.distance(lines[j]) < rules.clearance + rules.width(wire_net[other]) / 2.0 - 1e-6:
                        clear = False
                if clear:
                    out.setdefault(w, []).append(poly)
                    break
    return out
