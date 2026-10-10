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

    Between two pads too close for a teardrop each, the trace gets one piece of
    copper from pad to pad with straight sides instead, in the same form: two
    halves whose "track left" and "track right" are the ends of the line they
    share across its middle.

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
        # Each end: its pad, the trace seen from it, how far along the trace the pad reaches, the pad's size.
        ends = []
        for pad_id, pts in zip(wire_pads[w], (line, line[::-1])):
            pad = pads.get(pad_id)
            if pad is None:
                ends.append((None, pts, 0.0, 0.0))
            elif pad.radius is not None:
                ends.append((pad, pts, pad.radius, pad.radius))
            else:
                # Where the trace leaves the pad, and half the pad's smaller dimension (its inscribed radius).
                leave = LineString(pts).intersection(pad.shape.exterior)
                reach = 0.0 if leave.is_empty else max(Point(pad.centre).distance(g) for g in getattr(leave, "geoms", [leave]))
                ends.append((pad if not leave.is_empty else None, pts, reach, pad.shape.exterior.distance(Point(pad.centre))))
        def shaped(pad, p, widest):
            """The teardrop of ``pad`` with its point at p, or None."""
            if pad.radius is not None:
                return _round(pad.centre, pad.radius, p, half, widest)
            return _polygonal(pad.centre, list(pad.shape.exterior.coords)[:-1], p, half, widest)

        def fits(shape, own, room: float) -> bool:
            """Whether the copper ``shape`` adds to the pads ``own`` is on the
            board and keeps ``room`` from foreign copper and traces. What has
            to keep its distance is what is added: the part over a pad is the
            pad's, which stands where it stands. (Tested whole, a via at the
            least clearance from a neighbour lost every teardrop on that side.)"""
            if not shape.is_valid or not board.outline.contains(shape) or shape.distance(board.outline.exterior) < edge - 1e-6:
                return False
            added = shape
            for pad in own:
                added = added.difference(pad.shape)
            if added.is_empty:
                return False
            if tree_c is not None and any(copper[c][1] != net for c in tree_c.query(added, predicate="dwithin", distance=room - 1e-6).tolist()):
                return False
            for j in tree_w.query(added, predicate="dwithin", distance=room + max(rules.net_width.values(), default=rules.trace_width)).tolist():
                if wire_net[ids[j]] != net and added.distance(lines[j]) < room + rules.width(wire_net[ids[j]]) / 2.0 - 1e-6:
                    return False
            return True

        # Of the trace between its two pads each end's teardrop may take half: on a
        # trace shorter than two teardrops they would otherwise reach past each
        # other, each with its point inside the other's pad.
        share = (sum(math.dist(p, q) for p, q in zip(line, line[1:])) - ends[0][2] - ends[1][2]) / 2.0
        (pad_a, pts, reach_a, size_a), (pad_b, _, _, size_b) = ends
        if pad_a is not None and pad_b is not None and share > 0.0 and min(LENGTH * size_a, max_length) + min(LENGTH * size_b, max_length) > 2.0 * share:
            # No room for two teardrops: they would meet at a waist with a notch
            # either side of it. One piece of copper from pad to pad instead,
            # with straight sides, kept as two halves that share its middle.
            mid = _point_at(pts, reach_a + share)
            for widest, room in ((max_width, roomy), (max_width, rules.clearance), (min(max_width, 2.0 * min(size_a, size_b)) / 2.0, rules.clearance)):
                one, two = (shaped(pad_a, mid, widest), shaped(pad_b, mid, widest)) if mid is not None else (None, None)
                if one is None or two is None:
                    continue
                # Which point of the one pad is joined to which of the other: so that the sides do not cross.
                near, far = (two[4], two[1]) if not LineString([one[1], two[4]]).intersects(LineString([one[4], two[1]])) else (two[1], two[4])
                left = ((one[1][0] + near[0]) / 2.0, (one[1][1] + near[1]) / 2.0)
                right = ((one[4][0] + far[0]) / 2.0, (one[4][1] + far[1]) / 2.0)
                if math.dist(left, right) < 2.0 * half:
                    continue  # (narrower in the middle than the trace: two teardrops after all)
                tip = ((left[0] + right[0]) / 2.0, (left[1] + right[1]) / 2.0)
                if fits(Polygon([one[1], near, far, one[4]]), (pad_a, pad_b), room):
                    out[w] = [[one[0], one[1], left, right, one[4], tip], [two[0], near, left, right, far, tip]]
                    break
            if w in out:
                continue
        for pad, pts, reach, size in ends:
            if pad is None or share <= half:
                continue
            # As large as there is room for, and a small one rather than none:
            # shorter, then with no more than the clearance every trace keeps, then narrower.
            for length, widest, room in ((LENGTH, max_width, roomy), (LENGTH / 2.0, max_width, roomy),
                                         (LENGTH / 2.0, max_width, rules.clearance), (LENGTH / 2.0, min(max_width, 2.0 * size) / 2.0, rules.clearance)):
                p = _point_at(pts, reach + min(length * size, max_length, share))
                poly = shaped(pad, p, widest) if p is not None else None
                if poly is not None and fits(Polygon(poly[:5]), (pad,), room):
                    out.setdefault(w, []).append(poly)
                    break
    return out
