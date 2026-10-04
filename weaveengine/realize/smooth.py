"""Corner smoothing (post-processing, M8).

Relaxation leaves a trace as straight runs joined at corners, some of them
sharp. Each corner that turns by more than a few degrees is replaced by the
largest arc that fits: tangent to both runs, inside the corner. An arc is only
accepted if it keeps every clearance (to foreign copper, foreign traces and the
board edge) and sweeps over nothing foreign, so the trace keeps passing every
obstacle on the same side. Where no arc fits, the corner stays as it is.
"""
import math

import shapely
from shapely.geometry import LineString, Polygon

from weaveengine.board import Board

MIN_TURN = math.radians(12.0)   # corners gentler than this are left alone
ARC_STEP = math.radians(6.0)    # an arc is drawn as a polyline turning this much per segment
SHARE = 0.48                    # how much of a run one corner may use (the next corner needs its share)
TRIES = (1.0, 0.6, 0.35, 0.2, 0.1)
SLACK = 1e-4                    # mm kept beyond each clearance, so the result is not on the limit


def _arc(a, b, c, d: float):
    """Polyline of the arc tangent to b->a and b->c at distance ``d`` from the corner b."""
    u1 = ((a[0] - b[0]), (a[1] - b[1]))
    u2 = ((c[0] - b[0]), (c[1] - b[1]))
    l1, l2 = math.hypot(*u1), math.hypot(*u2)
    u1 = (u1[0] / l1, u1[1] / l1)
    u2 = (u2[0] / l2, u2[1] / l2)
    cos_a = max(-1.0, min(1.0, u1[0] * u2[0] + u1[1] * u2[1]))
    alpha = math.acos(cos_a)            # angle inside the corner
    turn = math.pi - alpha
    radius = d * math.tan(alpha / 2.0)
    bx, by = u1[0] + u2[0], u1[1] + u2[1]
    bl = math.hypot(bx, by)
    if bl < 1e-12 or radius < 1e-9:
        return None
    centre = (b[0] + bx / bl * d / math.cos(alpha / 2.0), b[1] + by / bl * d / math.cos(alpha / 2.0))
    p1 = (b[0] + u1[0] * d, b[1] + u1[1] * d)
    p2 = (b[0] + u2[0] * d, b[1] + u2[1] * d)
    a1 = math.atan2(p1[1] - centre[1], p1[0] - centre[0])
    a2 = math.atan2(p2[1] - centre[1], p2[0] - centre[0])
    sweep = (a2 - a1 + math.pi) % (2 * math.pi) - math.pi
    n = max(2, math.ceil(turn / ARC_STEP))
    # Vertices on a slightly larger circle, so the chords do not cut inside the true arc.
    out = [p1]
    grow = radius / math.cos(abs(sweep) / n / 2.0)
    for i in range(n):
        ang = a1 + sweep * (i + 0.5) / n
        out.append((centre[0] + grow * math.cos(ang), centre[1] + grow * math.sin(ang)))
    out.append(p2)
    return out


def smooth(board: Board, polylines: dict[int, list[tuple[float, float]]], wire_net: dict[int, int], layer: int = 0,
           min_turn: float = MIN_TURN) -> tuple[dict[int, list[tuple[float, float]]], int, int]:
    """Returns (smoothed polylines, corners rounded, sharp corners left as they were)."""
    rules = board.rules
    ids = list(polylines)
    lines = [LineString(polylines[w]) for w in ids]
    wire_tree = shapely.STRtree(lines) if lines else None
    copper = [(p.shape, p.net_id) for p in board.pads_on(layer)] + [(o.shape, o.net_id) for o in board.obstacles_on(layer)]
    copper_tree = shapely.STRtree([c[0] for c in copper]) if copper else None
    widest = max([rules.trace_width, *rules.net_width.values()])
    edge = rules.outline_inset - rules.base_width / 2.0
    boundary = board.outline.exterior
    rounded = kept = 0
    out: dict[int, list] = {}

    def fits(w: int, net: int, half: float, arc, corner) -> bool:
        line = LineString(arc)
        swept = Polygon([corner] + arc)
        if line.distance(boundary) < edge + half + SLACK:
            return False
        if copper_tree is not None:
            for i in copper_tree.query(line, predicate="dwithin", distance=rules.clearance + half + SLACK).tolist():
                if copper[i][1] != net or net < 0:
                    return False
            for i in copper_tree.query(swept, predicate="intersects").tolist():
                if copper[i][1] != net or net < 0:
                    return False  # something foreign sits in the corner: cutting it would pass it on the other side
        for j in wire_tree.query(line, predicate="dwithin", distance=rules.clearance + half + widest / 2.0 + SLACK).tolist():
            other = ids[j]
            if other != w and wire_net[other] != net:
                if line.distance(lines[j]) < rules.clearance + half + rules.width(wire_net[other]) / 2.0 + SLACK:
                    return False
        for j in wire_tree.query(swept, predicate="intersects").tolist():
            if ids[j] != w and wire_net[ids[j]] != net:
                return False
        return True

    for w, pts in polylines.items():
        net = wire_net[w]
        half = rules.width(net) / 2.0
        if len(pts) < 3:
            out[w] = pts
            continue
        new = [pts[0]]
        for i in range(1, len(pts) - 1):
            a, b, c = pts[i - 1], pts[i], pts[i + 1]
            h1 = math.atan2(b[1] - a[1], b[0] - a[0])
            h2 = math.atan2(c[1] - b[1], c[0] - b[0])
            turn = abs((h2 - h1 + math.pi) % (2 * math.pi) - math.pi)
            if turn < min_turn:
                new.append(b)
                continue
            reach = SHARE * min(math.dist(a, b), math.dist(b, c))
            done = False
            for part in TRIES:
                arc = _arc(a, b, c, reach * part) if reach * part > 1e-4 else None
                if arc is not None and fits(w, net, half, arc, b):
                    new.extend(arc)
                    rounded += 1
                    done = True
                    break
            if not done:
                new.append(b)
                kept += 1
        new.append(pts[-1])
        out[w] = new
    return out, rounded, kept
