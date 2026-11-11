"""Arcs found in a trace's polyline, for the session writer.

The router draws a curve (a rounded corner, a trace taut round a clearance
circle) as a polyline whose edges are all tangent to the circle: the first and
last edge touch it at the curve's ends, the rest at their middles, and the
vertices between stand a little outside it (``smooth._arc``, ``relax._arc``).
This finds such runs again and gives each as start, end and centre, so the
session can hold one arc where there were many short segments.

That finds the curve round one disc. A trace wrapped round a round pad passes
several: the pad is held as a polygon (``board.circle_polygon``, inflated with
mitred corners), and the trace goes round each of its corners on a disc of its
own, with a straight run along each side between. Such a run is one circle
within a few microns, round the pad's centre, but no edge of it is tangent to
that circle. ``fit_arcs`` finds those runs too (``circle_runs``): whole turns
of the polyline in a row, round the centre of a pad, a via or an oval pad's
round end (``near``), or else round a fitted centre if they are one circle to
a few microns. It writes the arc that lies inside every point of the run,
joined to the trace on either side by tangents: nearer the pad than the run,
so at the run's own least distance from its true circle. Such an arc comes
nearer other copper only where that lies between the run and the pad, and
the writer checks each one (``accept``, ``io.clearance``); where that turns it
down, circles a little further out are tried, then shorter runs.
"""
import math

TOL = 2e-5                  # mm: how far an edge may be from tangent to the circle, and an end from it
SAG = 5e-4                  # mm: the most the vertices between may stand out from the arc (a real corner is not an arc)
MIN_TURN = math.radians(0.5)  # a run turning less than this at its first vertex is left as a line
MAX_PIECE = math.radians(90.0)  # the longest arc written in one piece (the Specctra format means a quarter arc)
FIT = 0.03                  # a run is one circle if its points stand out from it by at most this share of its radius,
FIT_MM = (2e-3, 0.03)       # (mm) but at least the first and at most the second
FIT_FREE = 0.01             # mm: the same, round a centre only fitted (not a pad's or a via's)
STRAIGHT = 1e-4             # radians: a point turning less than this either way is on a straight line (a run may go on through it)
FIT_CENTRE = 0.1            # a pad's or via's centre this share of the radius from the fitted one is tried as the run's centre
OUTWARDS = (0.0, 0.35, 0.7)  # radii tried round a centre: from inside the run, out this share of the way to its furthest point
CORNER_REACH = 3.0          # mm: how far from a corner the trace turns at alone the centre of a pad it goes round may be
MAX_RADIUS = 25.0           # mm: a run of a larger radius is left as it is (it is near enough straight)


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


def _circle(pts):
    """The least-squares circle through ``pts`` (Kasa's fit): its centre, or None if they are in a line."""
    n = len(pts)
    mx, my = sum(p[0] for p in pts) / n, sum(p[1] for p in pts) / n
    suu = suv = svv = suuu = svvv = suvv = svuu = 0.0
    for x, y in pts:
        u, v = x - mx, y - my
        suu, suv, svv = suu + u * u, suv + u * v, svv + v * v
        suuu, svvv, suvv, svuu = suuu + u * u * u, svvv + v * v * v, suvv + u * v * v, svuu + v * u * u
    det = suu * svv - suv * suv
    if abs(det) < 1e-18 * max(1.0, (suu + svv) ** 2):
        return None
    bu, bv = (suuu + suvv) / 2.0, (svvv + svuu) / 2.0
    return (mx + (bu * svv - bv * suv) / det, my + (bv * suu - bu * suv) / det)


def _to_segment(c, p, q) -> float:
    vx, vy = q[0] - p[0], q[1] - p[1]
    n = vx * vx + vy * vy
    t = 0.0 if n == 0.0 else max(0.0, min(1.0, ((c[0] - p[0]) * vx + (c[1] - p[1]) * vy) / n))
    return math.hypot(p[0] + t * vx - c[0], p[1] + t * vy - c[1])


def _rounds(pts, i, j, side, near=None):
    """The arcs that may stand for ``pts[i..j]`` (with the points before and
    after it joined to them by tangents), best first, as (start, end, centre,
    radius, sweep). ``near(point, distance)`` gives the centres of round things
    (pads, vias) within ``distance`` of a point: the run is most likely round
    one of them, and the arc round that centre keeps the run's own distance
    from it, so those come first. Round each centre, the first arc is the
    largest circle every edge of the run keeps out of (so it lies inside the
    run); then circles a little larger, out towards the run's points, for where
    that one comes too near what the run goes round (a pad's corner, when the
    centre is only fitted)."""
    fitted = _circle(pts[i:j + 1]) if j - i >= 2 else None
    if fitted is not None:
        rough = math.dist(fitted, pts[i])
        if rough > 2.0 * MAX_RADIUS:
            return
        centres = [*(near(fitted, FIT_CENTRE * rough) if near else ()), fitted]
    else:  # (one or two corners: only round a pad or via; the nearest first)
        centres = sorted(near(pts[i], CORNER_REACH) if near else (), key=lambda c: math.dist(c, pts[i]))
    p, q = pts[i - 1], pts[j + 1]
    found = False
    for centre in centres:
        if found and centre is fitted:
            return  # (a pad's centre that fits is taken before the fitted one, which is only near the true centre)
        inner = min(_to_segment(centre, pts[k], pts[k + 1]) for k in range(i - 1, j + 1))
        if not 1e-6 < inner <= MAX_RADIUS:
            continue
        out = max(math.dist(pts[k], centre) for k in range(i, j + 1)) - inner
        if out > (FIT_FREE if centre is fitted else min(max(FIT * inner, FIT_MM[0]), FIT_MM[1])):
            continue
        # The run goes round the centre one way, all along, and less than a whole turn.
        total = 0.0
        for k in range(i - 1, j + 1):
            a, b = pts[k], pts[k + 1]
            step = math.atan2((a[0] - centre[0]) * (b[1] - centre[1]) - (a[1] - centre[1]) * (b[0] - centre[0]),
                              (a[0] - centre[0]) * (b[0] - centre[0]) + (a[1] - centre[1]) * (b[1] - centre[1]))
            if step * side <= 0.0:
                break
            total += step
        else:
            if abs(total) >= 2.0 * math.pi - 1e-3:
                continue
            found = True
            dp, dq = math.dist(p, centre), math.dist(q, centre)
            for share in OUTWARDS:
                radius = inner + share * out
                if radius > min(dp, dq):
                    break
                # Tangents from p and to q: where they touch the circle.
                back, ahead = math.acos(min(1.0, radius / dp)), math.acos(min(1.0, radius / dq))
                sweep = total - side * (back + ahead)
                if sweep * side < MIN_TURN:
                    break
                a1 = math.atan2(p[1] - centre[1], p[0] - centre[0]) + side * back
                a2 = a1 + sweep
                yield ((centre[0] + radius * math.cos(a1), centre[1] + radius * math.sin(a1)),
                       (centre[0] + radius * math.cos(a2), centre[1] + radius * math.sin(a2)), centre, radius, sweep)


def _units(pts, discs):
    """Where ``pts`` turns, in order, as (first, last, side, disc): the arcs of
    single discs (``discs``, from ``find_arcs``, with ``disc`` its index there)
    and the points between that turn on their own (``disc`` None)."""
    starts = {d[0]: k for k, d in enumerate(discs)}
    out, k = [], 1
    while k < len(pts) - 1:
        if k in starts:
            first, last = discs[starts[k]][:2]
            out.append((first, last, 1.0 if _turn(pts[first], pts[first + 1], pts[first + 2]) > 0 else -1.0, starts[k]))
            k = last if last > k else k + 1
            if out[-1][1] == k and k not in starts:
                k += 1
            continue
        turn = _turn(pts[k - 1], pts[k], pts[k + 1])
        if abs(turn) > STRAIGHT:
            out.append((k, k, 1.0 if turn > 0 else -1.0, None))
        k += 1
    if 0 in starts:  # (an arc from the very first point)
        first, last = discs[starts[0]][:2]
        out.insert(0, (first, last, 1.0 if _turn(pts[first], pts[first + 1], pts[first + 2]) > 0 else -1.0, starts[0]))
    return out


def _join(first, second):
    """Two runs one right after the other (the second starts at the point after
    the first ends), meeting along the line tangent to both their circles
    instead of at those two points: the runs with their arcs ended and started
    there, or None if there is no such line (or they overlap)."""
    (i1, j1, s1, e1, c1, r1, w1), (i2, j2, s2, e2, c2, r2, w2) = first, second
    side1, side2 = (1.0 if w1 > 0 else -1.0), (1.0 if w2 > 0 else -1.0)
    ta, tb = e1, s2
    for _ in range(30):  # (from one circle's touching point to the other's, and back, until they settle)
        d = math.dist(tb, c1)
        if d <= r1:
            return None
        a = math.atan2(tb[1] - c1[1], tb[0] - c1[0]) - side1 * math.acos(r1 / d)
        ta2 = (c1[0] + r1 * math.cos(a), c1[1] + r1 * math.sin(a))
        d = math.dist(ta2, c2)
        if d <= r2:
            return None
        a = math.atan2(ta2[1] - c2[1], ta2[0] - c2[0]) + side2 * math.acos(r2 / d)
        tb2 = (c2[0] + r2 * math.cos(a), c2[1] + r2 * math.sin(a))
        moved = math.dist(ta, ta2) + math.dist(tb, tb2)
        ta, tb = ta2, tb2
        if moved < 1e-9:
            break
    else:
        return None

    def turned(p, q, c):
        return (math.atan2(q[1] - c[1], q[0] - c[0]) - math.atan2(p[1] - c[1], p[0] - c[0]) + math.pi) % (2.0 * math.pi) - math.pi

    w1, w2 = w1 + turned(e1, ta, c1), w2 - turned(s2, tb, c2)
    if w1 * side1 < MIN_TURN or w2 * side2 < MIN_TURN or math.dist(ta, tb) < 1e-6:
        return None
    return (i1, j1, s1, ta, c1, r1, w1), (i2, j2, tb, e2, c2, r2, w2)


def _drawn(run):
    """A run's arc as points (``sampled``), from where it starts to where it ends."""
    return sampled(run[2], run[3], run[4], run[5], run[6])


def circle_runs(pts, accept=None, near=None, discs=None, min_radius: float = 0.0):
    """Runs of ``pts`` that are one circle within ``FIT``, as (first, last,
    start, end, centre, radius, sweep): the run ``pts[first..last]`` is to be
    replaced by the arc from ``start`` to ``end`` round ``centre`` (``sweep``
    radians, left positive), joined by straight lines to the trace on either
    side: from ``pts[first - 1]`` (or the end of the run before, if that ends
    at ``first - 1``) and to ``pts[last + 1]`` (or the start of the run after),
    all tangent to the circle.

    A run is made of whole turns of the polyline (``_units``): arcs of single
    discs (``discs``, default ``find_arcs``) and points turning on their own,
    two or more of them (or one such point, round a pad's centre), one after another,
    all turning the same way, with straight edges between: a trace round the
    corners of a polygon. ``accept(first, last, points)`` may turn down the
    points (sampled arcs and lines) that are to take the place of
    ``pts[first - 1..last + 1]``. Of the longest run accepted from each turn,
    those are taken that turn furthest in all without overlapping (two that
    meet go over to the line tangent to both, ``_join``). Arcs of a radius
    under ``min_radius`` are not taken. ``near``: see ``_rounds``."""
    units = _units(pts, find_arcs(pts) if discs is None else discs)
    m = len(pts) - 1
    ok = (lambda i, j, points: True) if accept is None else accept
    longest = []
    for u, (i, _, side, _) in enumerate(units):
        if i < 1:
            continue
        top = u
        while top + 1 < len(units) and units[top + 1][2] == side and units[top + 1][0] >= units[top][1]:
            top += 1
        for v in range(top, u - (units[u][3] is None), -1):  # (the longest first; a corner alone may go round a pad)
            j = units[v][1]
            if j > m - 1:
                continue
            arc = next((arc for arc in _rounds(pts, i, j, side, near)
                        if arc[3] >= min_radius and ok(i, j, [pts[i - 1]] + sampled(*arc) + [pts[j + 1]])), None)
            if arc is not None:
                longest.append((i, j, *arc))
                break
    longest.sort(key=lambda run: run[1])
    # Per run, in that order: the set that turns furthest and ends with it, as
    # (turn, runs). A run follows one ending two or more points before it, or
    # one ending right before it, the two joined.
    ending = []
    for k, run in enumerate(longest):
        best = (abs(run[6]), [run])
        for t in range(k):
            turn, runs = ending[t]
            if longest[t][1] + 2 <= run[0]:
                if turn + abs(run[6]) > best[0]:
                    best = (turn + abs(run[6]), runs + [run])
            elif longest[t][1] + 1 == run[0] and turn + abs(run[6]) > best[0]:
                joined = _join(runs[-1], run)
                if joined is None:
                    continue
                a, b = joined
                left = pts[a[0] - 1] if len(runs) < 2 or runs[-2][1] + 1 != a[0] else runs[-2][3]
                if ok(a[0], b[1], [left] + _drawn(a) + _drawn(b) + [pts[b[1] + 1]]):
                    best = (turn + abs(run[6]), runs[:-1] + [a, b])
        ending.append(best)
    return max(ending, key=lambda e: e[0], default=(0.0, []))[1]


def sampled(start, end, centre, radius, sweep, step=math.radians(1.0)):
    """The arc as points at most ``step`` apart, ends included."""
    n = max(1, math.ceil(abs(sweep) / step))
    a1 = math.atan2(start[1] - centre[1], start[0] - centre[0])
    return [start] + [(centre[0] + radius * math.cos(a1 + sweep * k / n), centre[1] + radius * math.sin(a1 + sweep * k / n))
                      for k in range(1, n)] + [end]


def fit_arcs(pts, accept=None, min_radius: float = 0.0, near=None):
    """``pts`` as runs to write, in order: ("path", points) and ("arc", pieces,
    centre), each arc in pieces (from, to) of at most 90 degrees: the runs
    that are one circle (``circle_runs``, with ``accept``), and the arcs of
    single discs (``find_arcs``) outside them. Arcs of a radius under
    ``min_radius`` are left as segments. ``near``: see ``_rounds``."""
    discs = find_arcs(pts)

    def ok(i, j, points):
        return accept is None or accept(i, j, points)

    # (first, last, how): a circle run's tangents and arc, or a disc's arc, in place of pts[first..last]
    spans = [(f, l, ("circle", start, end, centre, radius, sweep))
             for f, l, start, end, centre, radius, sweep in circle_runs(pts, ok, near, discs, min_radius)]
    taken = [(f - 1, l + 1) for f, l, _ in spans]
    for first, last, centre, radius in discs:
        if radius >= min_radius and not any(a < last and first < b for a, b in taken):
            spans.append((first, last, ("disc", centre, radius)))
    spans.sort()
    runs, here, at = [], [pts[0]], 0
    for k, (first, last, how) in enumerate(spans):
        if how[0] == "circle":
            _, start, end, centre, radius, sweep = how
            here += pts[at + 1:first] + [start]
            runs.append(("path", here))
            runs.append(("arc", _split(start, end, centre, radius, math.atan2(start[1] - centre[1], start[0] - centre[0]), sweep), centre))
            if k + 1 < len(spans) and spans[k + 1][0] == last + 1:
                here, at = [end], last  # (joined to the next run by the line tangent to both)
            else:
                here, at = [end, pts[last + 1]], last + 1
        else:
            _, centre, radius = how
            here += pts[at + 1:first + 1]
            runs.append(("path", here))
            runs.append(("arc", arc_pieces(pts, first, last, centre, radius), centre))
            here, at = [pts[last]], last
    here += pts[at + 1:]
    runs.append(("path", here))
    return runs
