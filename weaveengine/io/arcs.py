"""Arcs found in a trace's polyline, for the session writer.

The router draws a curve (a rounded corner, a trace taut round a clearance
circle) as a polyline whose edges are all tangent to the circle: the first and
last edge touch it at the curve's ends, the rest at their middles, and the
vertices between stand a little outside it (``smooth._arc``, ``relax._arc``).
This finds such runs again and gives each as start, end and centre, so the
session can hold one arc where there were many short segments.
"""
import math

TOL = 2e-5                  # mm: how far an edge may be from tangent to the circle, and an end from it
SAG = 5e-4                  # mm: the most the vertices between may stand out from the arc (a real corner is not an arc)
MIN_TURN = math.radians(0.5)  # a run turning less than this at its first vertex is left as a line
MAX_PIECE = math.radians(90.0)  # the longest arc written in one piece (the Specctra format means a quarter arc)


def find_arcs(pts: list[tuple[float, float]]) -> list[tuple[int, int, tuple[float, float], float]]:
    """Runs of ``pts`` that are one circular arc, as (first index, last index, centre, radius).

    A run has at least two edges, all tangent to one circle, turning the same
    way, the first tangent at its first point and the last at its last point.
    Where several runs start at a point, the longest is taken.
    """
    found = []
    m = len(pts) - 1
    i = 0
    while i < m - 1:
        best = _run_from(pts, i)
        if best is None:
            i += 1
            continue
        j, centre, radius = best
        found.append((i, j, centre, radius))
        i = j
    return found


def _run_from(pts, i):
    """The longest arc starting at point ``i`` with its first edge tangent there: (last index, centre, radius) or None."""
    a, b, c = pts[i], pts[i + 1], pts[i + 2]
    ex, ey = b[0] - a[0], b[1] - a[1]
    length = math.hypot(ex, ey)
    if length < 1e-9:
        return None
    turn = _turn(a, b, c)
    if abs(turn) < MIN_TURN or abs(turn) > math.pi - 1e-3:
        return None
    side = 1.0 if turn > 0 else -1.0
    radius = length / math.tan(abs(turn) / 2.0)
    # (the left normal of the first edge is (-ey, ex); the centre is on the side the trace turns to)
    centre = (a[0] - side * ey / length * radius, a[1] + side * ex / length * radius)
    best = None
    for k in range(i, len(pts) - 1):
        p, q = pts[k], pts[k + 1]
        if k > i and _turn(pts[k - 1], p, q) * side <= 0.0:
            break
        if not _tangent(p, q, centre, radius):
            break
        if k > i and math.dist(p, centre) - radius > SAG:
            break
        if k > i and abs(math.dist(q, centre) - radius) <= TOL:
            best = (k + 1, centre, radius)
    return best


def _turn(a, b, c) -> float:
    """The signed angle the path turns by at ``b`` (left is positive)."""
    h1 = math.atan2(b[1] - a[1], b[0] - a[0])
    h2 = math.atan2(c[1] - b[1], c[0] - b[0])
    return (h2 - h1 + math.pi) % (2.0 * math.pi) - math.pi


def _tangent(p, q, centre, radius) -> bool:
    """Whether the edge p q touches the circle, at a point of the edge."""
    vx, vy = q[0] - p[0], q[1] - p[1]
    n = vx * vx + vy * vy
    if n < 1e-18:
        return False
    t = ((centre[0] - p[0]) * vx + (centre[1] - p[1]) * vy) / n
    if t < -1e-6 or t > 1.0 + 1e-6:
        return False
    return abs(math.hypot(p[0] + t * vx - centre[0], p[1] + t * vy - centre[1]) - radius) <= TOL


def _split(start, end, centre, radius, a1, sweep):
    count = max(1, math.ceil(abs(sweep) / MAX_PIECE - 1e-9))
    pts = [start]
    for k in range(1, count):
        ang = a1 + sweep * k / count
        pts.append((centre[0] + radius * math.cos(ang), centre[1] + radius * math.sin(ang)))
    pts.append(end)
    return list(zip(pts, pts[1:]))


def arc_pieces(pts, first, last, centre, radius):
    """The run ``pts[first..last]`` found by ``find_arcs`` as (from, to) pieces of at most 90 degrees."""
    # The first and last edges are tangent at the ends, so the turns at the vertices between add up to the sweep.
    sweep = sum(_turn(pts[k - 1], pts[k], pts[k + 1]) for k in range(first + 1, last))
    a1 = math.atan2(pts[first][1] - centre[1], pts[first][0] - centre[0])
    return _split(pts[first], pts[last], centre, radius, a1, sweep)
