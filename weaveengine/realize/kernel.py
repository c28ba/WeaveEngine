"""Compiled string pulling for the relaxation (design sections 3 and 13.1).

A wire passes a row of discs, each on a known side: the vertices of the
triangles it crosses, with as radius the room the wires between it and the
vertex take up. Its trace is the shortest line that does: straight runs
tangent to the discs it touches, and arcs round them.

This is the funnel algorithm with discs for points, and one thing does not
carry over. Between points the two sides of the funnel only meet at its apex.
A disc reaches into the other side's string anywhere along it, so the string
to the last disc on the left may have to go round discs of the right first,
and back. The funnel is therefore kept as what it stands for: two taut
strings from the start, one to the last disc met on each side, each free to
touch discs of either side.
"""
import numpy as np

from weaveengine.topo.kernel import njit


@njit(cache=True, nogil=True)
def heading(x, y, r, a, b):
    """Unit direction of the string running from disc a to disc b. r is signed:
    positive for a disc the string passes on its left."""
    dx, dy = x[b] - x[a], y[b] - y[a]
    n2 = dx * dx + dy * dy
    if n2 < 1e-18:
        return 0.0, 0.0
    turn = r[b] - r[a]
    l2 = n2 - turn * turn
    if l2 <= 0.0:
        # One disc reaches into the other (an over-full gate): there is no
        # such line. Pass square to the line of centres; DRC will report it.
        n = np.sqrt(n2) if turn > 0.0 else -np.sqrt(n2)
        return dy / n, -dx / n
    run = np.sqrt(l2)
    return (run * dx + turn * dy) / n2, (run * dy - turn * dx) / n2


@njit(cache=True, nogil=True)
def blocked(x, y, r, a, c, q, on_left):
    """Whether disc q, to be passed on the left (``on_left``) or on the right,
    stands in the way of the string from disc a to disc c. Only the string
    itself counts, from where it leaves a to where it lands on c: a disc it
    would meet beyond that is passed later, on the way round c. (Between
    points this is the funnel's usual test of which side of the line q lies on.)"""
    hx, hy = heading(x, y, r, a, c)
    ax, ay = x[a] + r[a] * hy, y[a] - r[a] * hx
    run = (x[c] + r[c] * hy - ax) * hx + (y[c] - r[c] * hx - ay) * hy
    qx, qy = x[q] - ax, y[q] - ay
    along = qx * hx + qy * hy
    if along < 0.0 or (hx == 0.0 and hy == 0.0):  # behind the string; or a and c are one point: no string
        return False
    if along > run:
        qx, qy = qx - run * hx, qy - run * hy
        return qx * qx + qy * qy < r[q] * r[q] - 1e-12
    side = qy * hx - qx * hy  # how far q's centre lies to the left of the string
    return side < r[q] - 1e-12 if on_left else side > r[q] + 1e-12


@njit(cache=True, nogil=True)
def inside(x, y, r, left, a, b):
    """Whether disc a lies inside disc b, both passed on the same side: then a
    is never touched."""
    if left[a] != left[b] or abs(r[a]) > abs(r[b]):
        return False
    dx, dy, gap = x[b] - x[a], y[b] - y[a], abs(r[b]) - abs(r[a])
    return dx * dx + dy * dy <= gap * gap + 1e-18


@njit(cache=True, nogil=True)
def lifts(x, y, r, left, most, a, b, c):
    """Whether the string a -> b -> c comes off disc b: it does not turn round
    the disc there, or by more than it can. ``most`` is that limit, from how
    far the wire's route goes round the vertex. It tells a turn of 350 degrees
    from a turn of 10 the other way, and it is what keeps the string on its
    route: taut round the far side of a disc is taut too."""
    if inside(x, y, r, left, b, c):
        return True
    ax, ay = heading(x, y, r, a, b)
    bx, by = heading(x, y, r, b, c)
    turn, ahead = ax * by - ay * bx, ax * bx + ay * by
    if not left[b]:
        turn = -turn
    if abs(turn) <= 1e-12 and ahead >= 0.0:
        return ahead > 0.0  # straight on: not held by the disc (no string at all: leave it)
    return np.arctan2(turn, ahead) % (2.0 * np.pi) > most[b]


@njit(cache=True, nogil=True)
def reach(x, y, r, left, most, mine, n, other, m, c, last):
    """Takes the string ``mine`` (n discs) on to disc c: drops the discs it
    lifts off, and if a disc of the string ``other`` (m discs) then stands in
    the way, goes round by that string instead. Afterwards, if c stands in the
    way of ``other``, that string goes by c from there on. (``last``: c is the
    wire's end point, which has no side and is in nobody's way.) Returns both lengths."""
    while n > 1 and lifts(x, y, r, left, most, mine[n - 2], mine[n - 1], c):
        n -= 1
    same = 0  # discs the two strings start with in common
    while same < n and same < m and mine[same] == other[same]:
        same += 1
    if inside(x, y, r, left, c, mine[n - 1]):
        return n, m
    for j in range(same, m):
        if (last or left[other[j]] != left[c]) and blocked(x, y, r, mine[n - 1], c, other[j], left[other[j]]):
            n = m
            mine[:n] = other[:m]
            while n > 1 and lifts(x, y, r, left, most, mine[n - 2], mine[n - 1], c):
                n -= 1
            while same < n and same < m and mine[same] == other[same]:
                same += 1
            break
    mine[n] = c
    n += 1
    for j in range(max(same, 1), 0 if last else m):
        if blocked(x, y, r, other[j - 1], other[j], c, left[c]):
            # The rest of the other string, from disc j on, hangs from c;
            # less the discs it now lifts off.
            while j < m - 1 and lifts(x, y, r, left, most, c, other[j], other[j + 1]):
                j += 1
            rest = other[j:m].copy()
            other[:n] = mine[:n]
            other[n:n + m - j] = rest
            m = n + m - j
            break
    return n, m


@njit(cache=True, nogil=True)
def pull(ptr, x, y, r, left, most, out_ptr, out, come, go):
    """Pulls every wire taut. Wire w owns the discs ptr[w]:ptr[w + 1], in the
    order it meets them: its start point, the discs (centre x, y; signed radius
    r; left: passed on the left; most: the furthest it can turn round the
    disc), its end point.

    Writes the discs each wire touches to out[out_ptr[w]:out_ptr[w + 1]], and
    for each of those the point where the string comes on to it and the point
    where it goes off again.
    """
    nw = ptr.shape[0] - 1
    longest = 0
    for w in range(nw):
        if ptr[w + 1] - ptr[w] > longest:
            longest = ptr[w + 1] - ptr[w]
    port, star = np.empty(longest + 1, dtype=np.int64), np.empty(longest + 1, dtype=np.int64)  # the strings to the left and to the right
    at = 0
    for w in range(nw):
        a0, a1 = ptr[w], ptr[w + 1]
        port[0] = star[0] = a0
        n = m = 1
        for c in range(a0 + 1, a1):
            if left[c] or c == a1 - 1:  # the end point has no side: reached like a disc on the left
                n, m = reach(x, y, r, left, most, port, n, star, m, c, c == a1 - 1)
            else:
                m, n = reach(x, y, r, left, most, star, m, port, n, c, False)
        first = out_ptr[w] = at
        out[at:at + n] = port[:n]
        at += n
        come[first, 0], come[first, 1] = x[a0], y[a0]
        for j in range(first, at - 1):
            a, b = out[j], out[j + 1]
            hx, hy = heading(x, y, r, a, b)
            go[j, 0], go[j, 1] = x[a] + r[a] * hy, y[a] - r[a] * hx
            come[j + 1, 0], come[j + 1, 1] = x[b] + r[b] * hy, y[b] - r[b] * hx
        go[at - 1, 0], go[at - 1, 1] = x[a1 - 1], y[a1 - 1]
    out_ptr[nw] = at
    return at
