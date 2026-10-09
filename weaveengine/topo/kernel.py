"""The search kernel (design sections 3, 8 and 19.7): the slot-aware A* on flat
arrays, compiled with numba. numba is optional: without it (or with the
environment variable WEAVEENGINE_NO_NUMBA set) the same functions run as plain
Python, with the same results, only slower.
"""
import os
import sys

import numpy as np

AVAILABLE = False   # the compiled kernels are in use
WHY_NOT = ""        # if not: the reason, for the user (see weaveengine.accel)


def _cache_dir() -> None:
    """numba caches compiled code next to the source. Inside a packaged app
    (or a read-only install) that is not writable, so point it at the user's
    cache folder instead. Must happen before numba is imported."""
    if "NUMBA_CACHE_DIR" in os.environ:
        return
    here = os.path.dirname(os.path.abspath(__file__))
    if getattr(sys, "frozen", False) or not os.access(here, os.W_OK):
        base = (os.path.expanduser("~/Library/Caches") if sys.platform == "darwin"
                else os.environ.get("LOCALAPPDATA") or os.environ.get("XDG_CACHE_HOME") or os.path.expanduser("~/.cache"))
        os.environ["NUMBA_CACHE_DIR"] = os.path.join(base, "WeaveEngine", "numba")


NOTES: list[str] = []  # things worth telling the user that are not failures (e.g. no on-disk cache)

try:
    if os.environ.get("WEAVEENGINE_NO_NUMBA"):
        WHY_NOT = "switched off by the WEAVEENGINE_NO_NUMBA environment variable"
        raise ImportError(WHY_NOT)
    _cache_dir()
    from numba import njit as _njit
    AVAILABLE = True
except Exception as error:  # not only ImportError: a broken LLVM install fails in other ways
    AVAILABLE = False
    WHY_NOT = WHY_NOT or f"numba could not be loaded ({type(error).__name__}: {error})"


def njit(**options):
    """numba.njit that cannot fail: without numba the function is returned as
    it is; if the on-disk cache cannot be used (some packaged or read-only
    installs) it compiles without one; if even declaring it fails, the kernels
    are switched off with the reason recorded."""
    def wrap(fn):
        global AVAILABLE, WHY_NOT
        if not AVAILABLE:
            return fn
        try:
            return _njit(**options)(fn)
        except Exception as first:
            try:
                compiled = _njit(**{k: v for k, v in options.items() if k != "cache"})(fn)
                NOTES.append(f"compiled code cannot be cached on disk here, so it is recompiled at every start "
                             f"({type(first).__name__}: {first})")
                return compiled
            except Exception as second:
                AVAILABLE = False
                WHY_NOT = f"numba could not prepare the kernels ({type(second).__name__}: {second})"
                return fn
    return wrap

SLOTS = 16  # node id = half_edge * 16 + slot
RIDE = 0.05  # what a length costs beside a wire of the same net: next to nothing (11), but shorter is still better


class Tables:
    """Flat copies of a planar map's transition tables, plus the search workspace."""

    def __init__(self, pmap):
        # Room for more half-edges than the map has: via sites add edges, and
        # the workspace below is too large to copy every time one does.
        e2 = self.room = 2 * pmap.num_edges + max(256, pmap.num_edges // 4)
        # Per half-edge, its (up to two) transitions side by side, as the search reads them:
        # exit edge, next half-edge, triangle, corner, corner at the u end of this edge, of the exit edge.
        self.tr = np.zeros((e2, 2, 6), dtype=np.int32)
        self.length = np.zeros((e2, 2), dtype=np.float64)
        self.n = np.zeros(e2, dtype=np.int32)
        self.update(pmap, range(2 * pmap.num_edges))
        nodes = e2 * SLOTS
        self.g = np.full(nodes, np.inf)
        self.parent = np.full(nodes, -1, dtype=np.int32)
        self.parent_tr = np.zeros(nodes, dtype=np.int8)
        self.touched = np.zeros(nodes, dtype=np.int32)
        size = max(4 * nodes, 1024)
        self.heap_f = np.zeros(size)
        self.heap_g = np.zeros(size)
        self.heap_n = np.zeros(size, dtype=np.int32)
        self.no_penalty = np.zeros(1)
        self.no_corridor = np.zeros(1, dtype=np.uint8)
        self.ride = np.zeros(e2 // 2 + 1, dtype=np.int64)  # per edge: the places beside a wire of the net being routed
        self.own = np.zeros(e2 // 2 + 1, dtype=np.int32)   # per edge: 1 + which pad of that net it is an edge of (0: of none)

    def update(self, pmap, half_edges) -> bool:
        """Copies the transitions of ``half_edges`` from the map (its per-edge
        tables the search reads directly). Returns False if the map has
        outgrown the room: build anew."""
        if 2 * pmap.num_edges > self.room:
            return False
        trans = pmap.trans
        for he in half_edges:
            row = trans[he]
            self.n[he] = len(row)
            for j, row_j in enumerate(row):
                self.tr[he, j] = row_j[:6]
                self.length[he, j] = row_j[6]
        return True


@njit(cache=True, nogil=True)
def estimate(x, y, tx, ty, rad, hw, far, fx0, fy0, fcell):
    """What it costs at least from x y to the goal at tx ty (radius ``rad``):
    the straight line, of which only the way to the nearest wire of the
    wire's own net has to be new. ``far`` is a coarse map of that distance
    (cells of side ``fcell`` from fx0 fy0; one cell holding infinity if the
    net has no wire here). Beside its own net a wire costs ``RIDE`` of its
    length, and the search has to know: an estimate that took every length in
    full would turn it away from the detour that leads to its own trace."""
    d = np.hypot(x - tx, y - ty) - rad
    if d <= 0.0:
        return 0.0
    i = min(max(int((x - fx0) / fcell), 0), far.shape[0] - 1)
    j = min(max(int((y - fy0) / fcell), 0), far.shape[1] - 1)
    if far[i, j] < d:
        d = RIDE * d + (1.0 - RIDE) * far[i, j]
    return d * hw


@njit(cache=True, nogil=True)
def enter(node0, first, last, c, over, e, root, how, pres, hard_cap, use_hist, hist, ride, h,
          g, parent, parent_tr, touched, n_touched, heap_f, heap_g, heap_n, size):
    """Starts a wire on edge e (node0: its first node) at each of the places
    first..last, at cost c: from a pad, from a seed, or going on through a pad
    of its net. ``over``: by how much the edge is then over-full, which a
    place beside a wire of the net is not charged. Returns (nodes touched,
    heap size, or -1 if the heap is full)."""
    apart = c
    if over > 1e-9:
        apart = np.inf if hard_cap else apart + pres * (over if over > 1.0 else 1.0)
    if use_hist:
        apart += hist[e]
    for p in range(first, last + 1):
        node = node0 + p
        cost = c if (ride[e] >> p) & 1 else apart
        if cost >= g[node]:
            continue
        if g[node] == np.inf:
            touched[n_touched] = node
            n_touched += 1
        g[node] = cost
        parent[node] = root
        parent_tr[node] = how
        if size >= heap_f.shape[0]:
            return n_touched, -1
        i = size
        size += 1
        f = cost + h
        while i > 0:
            up = (i - 1) >> 1
            if heap_f[up] <= f:
                break
            heap_f[i], heap_g[i], heap_n[i] = heap_f[up], heap_g[up], heap_n[up]
            i = up
        heap_f[i], heap_g[i], heap_n[i] = f, cost, node
    return n_touched, size


THROUGH = 2  # ``parent_tr`` of a node that goes on from a pad of the net the wire came to (0, 1: a transition)


@njit(cache=True, nogil=True)
def astar(starts, start_cost, seed_tri, seed_cost, seed_x, seed_y, dst_pad, tri_e, tri_v, edge_v, edge_t,
          tr, tlen, tn, kind, owner, mid,
          count, corner, load, cap, hist, penalty, use_penalty, corridor, use_corridor,
          relaxed, pres, use_hist, cross_pen, hard_cap, weight, ride, own, own_ptr, own_edge, own_cost, own_node,
          tx, ty, rad, hw, far, fx0, fy0, fcell, bound,
          g, parent, parent_tr, touched, heap_f, heap_g, heap_n, best, best_node):
    """The one search (design section 8). Cheapest way to ``dst_pad``.

    It starts from the pad edges ``starts`` (each at its ``start_cost``) and
    from seeds: a point in the middle cell of triangle ``seed_tri[i]`` at
    ``seed_cost[i]`` (a via from another layer; the middle cell is the part of
    a triangle no wire has cut off). ``parent`` of a seed's first node is -2 - i.

    Returns (goal node, or -1 if none, or -2 if the heap overflowed; cost;
    nodes touched). ``g``/``parent`` are left filled for the caller to read
    paths, which must then reset the first ``touched`` entries. ``best`` and
    ``best_node`` are filled, per triangle, with the cheapest cost at which
    its middle cell was reached and the node where: every triangle a via
    could usefully go in has been reached by the time the goal is. The search
    gives up on anything that cannot cost less than ``bound``.

    ``ride[e]`` has bit p set where place p on edge e is beside a wire of the
    net being routed. A wire there is the same trace as its neighbour: it puts
    no load on the gate, and from one such place to the next it adds next to
    no length.

    ``own[e]`` is 1 + k on the edges of pad k of the net being routed, other
    than those the wire starts and ends on (11). Such a pad is the net's
    copper: the wire may run into it and go on from any of its edges
    (``own_edge[own_ptr[k]:own_ptr[k + 1]]``), as a new piece. ``own_cost``
    and ``own_node`` are filled, per pad, with the cheapest arrival.
    """
    n_touched = 0
    size = 0
    for si in range(starts.shape[0] + seed_tri.shape[0] * 3):
        if si < starts.shape[0]:
            e = starts[si]
            n = count[e]
            first, last, root, c = 0, n, -1, start_cost[si]
            node0 = (2 * e) * SLOTS
            over = load[e] + weight - cap[e] if n > 0 else 0.0   # one wire always fits its own pad edge
        else:
            sj = (si - starts.shape[0]) // 3
            t = seed_tri[sj]
            e = tri_e[t, (si - starts.shape[0]) % 3]
            n = count[e]
            side = 1 if edge_t[e, 0] == t else 0
            if si - starts.shape[0] == 3 * sj and seed_cost[sj] < best[t]:
                best[t] = seed_cost[sj]
                best_node[t] = -2 - sj
            if kind[e] != 0 or edge_t[e, side] < 0:
                continue
            u = edge_v[e, 0]
            ku = 0 if tri_v[t, 0] == u else (1 if tri_v[t, 1] == u else 2)
            first = last = corner[t, ku]  # the middle of the gate, seen from inside the triangle
            root = -2 - sj
            c = seed_cost[sj] + np.hypot(mid[e, 0] - seed_x[sj], mid[e, 1] - seed_y[sj])
            node0 = (2 * e + side) * SLOTS
            over = load[e] + weight - cap[e]
        if n >= SLOTS - 1 or (use_corridor and corridor[e] == 0):
            continue
        if use_penalty:
            c += penalty[e]
        h = estimate(mid[e, 0], mid[e, 1], tx, ty, rad, hw, far, fx0, fy0, fcell)
        n_touched, size = enter(node0, first, last, c, over, e, root, 0, pres, hard_cap, use_hist, hist, ride, h,
                                g, parent, parent_tr, touched, n_touched, heap_f, heap_g, heap_n, size)
        if size < 0:
            return -2, 0.0, n_touched

    while size > 0:
        if heap_f[0] >= bound:
            break  # nothing left that could beat what the caller already has
        gn = heap_g[0]
        node = heap_n[0]
        # pop
        size -= 1
        if size > 0:
            f, c, nd = heap_f[size], heap_g[size], heap_n[size]
            i = 0
            while True:
                child = 2 * i + 1
                if child >= size:
                    break
                if child + 1 < size and heap_f[child + 1] < heap_f[child]:
                    child += 1
                if heap_f[child] >= f:
                    break
                heap_f[i], heap_g[i], heap_n[i] = heap_f[child], heap_g[child], heap_n[child]
                i = child
            heap_f[i], heap_g[i], heap_n[i] = f, c, nd
        if gn > g[node]:
            continue
        he = node // SLOTS
        p = node - he * SLOTS
        e = he >> 1
        if (he & 1) == 1 and kind[e] == 2:
            if owner[e] == dst_pad:
                return node, gn, n_touched
            # A pad of its net on the way: it goes on from any edge of the pad.
            # Across the pad it is the pad's copper, like a wire it runs beside.
            # (The estimate of what is left knows the net's wires, not its pads:
            # it can be too high by the width of a pad, and the route found too
            # dear by as much. Knowing the pads would blunt it over the whole
            # board for a net with many.)
            k = own[e] - 1
            if gn < own_cost[k]:
                own_cost[k] = gn
                own_node[k] = node
            for q in range(own_ptr[k], own_ptr[k + 1]):
                b = own_edge[q]
                nb = count[b]
                if nb >= SLOTS - 1 or (use_corridor and corridor[b] == 0):
                    continue
                c = gn + RIDE * np.hypot(mid[b, 0] - mid[e, 0], mid[b, 1] - mid[e, 1])
                if use_penalty:
                    c += penalty[b]
                h = estimate(mid[b, 0], mid[b, 1], tx, ty, rad, hw, far, fx0, fy0, fcell)
                n_touched, size = enter((2 * b) * SLOTS, 0, nb, c, load[b] + weight - cap[b] if nb > 0 else 0.0, b, node, THROUGH,
                                        pres, hard_cap, use_hist, hist, ride, h,
                                        g, parent, parent_tr, touched, n_touched, heap_f, heap_g, heap_n, size)
                if size < 0:
                    return -2, 0.0, n_touched
            continue
        ne = count[e]
        here = (ride[e] >> p) & 1 == 1
        if tn[he] > 0:
            t = tr[he, 0, 2]
            r = p if tr[he, 0, 4] else ne - p
            if r == corner[t, tr[he, 0, 3]] and gn < best[t]:
                best[t] = gn
                best_node[t] = node
        for j in range(tn[he]):
            b = tr[he, j, 0]
            if kind[b] == 2 and owner[b] != dst_pad and own[b] == 0:
                continue
            if use_corridor and corridor[b] == 0:
                continue
            nb = count[b]
            if nb >= SLOTS - 1:
                continue
            r = p if tr[he, j, 4] else ne - p
            lim = corner[tr[he, j, 2], tr[he, j, 3]]
            c = gn
            if r > lim:
                if not relaxed:
                    continue
                c += cross_pen * (r - lim)
                r = lim
            nxt = tr[he, j, 1]
            pb = r if tr[he, j, 5] else nb - r
            if (ride[b] >> pb) & 1:
                c += tlen[he, j] * (RIDE if here else 1.0)
            else:
                c += tlen[he, j]
                over = load[b] + weight - cap[b]
                if over > 1e-9 and (nb > 0 or nxt >= 0):
                    if hard_cap:
                        continue
                    c += pres * (over if over > 1.0 else 1.0)
                if use_hist:
                    c += hist[b]
            if use_penalty:
                c += penalty[b]
            nn = (nxt if nxt >= 0 else 2 * b + 1) * SLOTS + pb
            if c < g[nn]:
                if g[nn] == np.inf:
                    touched[n_touched] = nn
                    n_touched += 1
                g[nn] = c
                parent[nn] = node
                parent_tr[nn] = j
                if size >= heap_f.shape[0]:
                    return -2, 0.0, n_touched
                i = size
                size += 1
                f = c + estimate(mid[b, 0], mid[b, 1], tx, ty, rad, hw, far, fx0, fy0, fcell)
                while i > 0:
                    up = (i - 1) >> 1
                    if heap_f[up] <= f:
                        break
                    heap_f[i], heap_g[i], heap_n[i] = heap_f[up], heap_g[up], heap_n[up]
                    i = up
                heap_f[i], heap_g[i], heap_n[i] = f, c, nn
    return -1, 0.0, n_touched


@njit(cache=True, nogil=True)
def walk_back(parent, node, out):
    """The nodes from ``node`` back to where its search started, into ``out``;
    returns how many, and the parent (below zero) of the last."""
    n = 0
    while True:
        out[n] = node
        n += 1
        prev = parent[node]
        if prev < 0 or n == out.shape[0]:
            return n, prev
        node = prev


walk_back_plain = getattr(walk_back, "py_func", walk_back)
astar_plain = getattr(astar, "py_func", astar)  # the same search as plain Python: the fallback if the compiled one fails


@njit(cache=True, nogil=True)
def cells(tri_v, vx, vy, x0, y0, cell, nx, ny, start, items):
    """Sorts the triangles into a grid of nx by ny cells (side ``cell``, origin
    x0 y0): the triangles whose bounding box meets cell c are
    items[start[c]:start[c + 1]]. Returns how many entries that takes; if
    ``items`` is shorter, nothing is written to it."""
    n_tri = tri_v.shape[0]
    start[:] = 0
    for fill in range(2):
        for t in range(n_tri):
            a, b, c = tri_v[t, 0], tri_v[t, 1], tri_v[t, 2]
            i0 = max(int(np.floor((min(vx[a], vx[b], vx[c]) - x0) / cell)), 0)
            i1 = min(int(np.floor((max(vx[a], vx[b], vx[c]) - x0) / cell)), nx - 1)
            j0 = max(int(np.floor((min(vy[a], vy[b], vy[c]) - y0) / cell)), 0)
            j1 = min(int(np.floor((max(vy[a], vy[b], vy[c]) - y0) / cell)), ny - 1)
            for i in range(i0, i1 + 1):
                for j in range(j0, j1 + 1):
                    if fill:
                        items[start[i * ny + j]] = t
                    start[i * ny + j] += 1
        if fill:  # each start now stands at its cell's end: the start of the next
            for c in range(nx * ny, 0, -1):
                start[c] = start[c - 1]
            start[0] = 0
        else:
            total = 0
            for c in range(nx * ny):
                total, start[c] = total + start[c], total
            start[nx * ny] = total
            if total > items.shape[0]:
                return total
    return start[nx * ny]


@njit(cache=True, nogil=True)
def locate(tri_v, vx, vy, px, py, out, x0, y0, cell, nx, ny, start, items):
    """Triangle containing each point (-1 if none), from the grid ``cells`` made."""
    for n in range(px.shape[0]):
        x, y = px[n], py[n]
        out[n] = -1
        i, j = int(np.floor((x - x0) / cell)), int(np.floor((y - y0) / cell))
        if i < 0 or j < 0 or i >= nx or j >= ny:
            continue
        for q in range(start[i * ny + j], start[i * ny + j + 1]):
            t = items[q]
            inside = True
            for k in range(3):
                a, b = tri_v[t, (k + 1) % 3], tri_v[t, (k + 2) % 3]
                if (vx[b] - vx[a]) * (y - vy[a]) - (vy[b] - vy[a]) * (x - vx[a]) < -1e-12:
                    inside = False
                    break
            if inside:
                out[n] = t
                break


@njit(cache=True, nogil=True)
def roomy(tri_v, vx, vy, corner, pitch, keep, least, t, x, y):
    """Whether the point x y really lies in the middle cell of triangle t:
    ``least`` clear of the triangle's edges, and far enough from each corner
    for the via's keep-off and the wires that cut that corner."""
    for k in range(3):
        a, b = tri_v[t, k], tri_v[t, (k + 1) % 3]
        if np.hypot(vx[a] - x, vy[a] - y) - keep < (corner[t, k] - 1) * pitch:
            return False
        ex, ey = vx[b] - vx[a], vy[b] - vy[a]
        if (ex * (y - vy[a]) - ey * (x - vx[a])) / max(np.hypot(ex, ey), 1e-12) < least:
            return False
    return True


@njit(cache=True, nogil=True)
def rooms(tri_v, vx, vy, corner, pitch, keep, least, tris, px, py):
    """``roomy`` for each point in its triangle."""
    out = np.zeros(tris.shape[0], dtype=np.bool_)
    for i in range(tris.shape[0]):
        out[i] = roomy(tri_v, vx, vy, corner, pitch, keep, least, tris[i], px[i], py[i])
    return out


@njit(cache=True, nogil=True)
def via_points(best, extra, bound, gx, gy, onward, tri_v, vx, vy, corner, pitch, keep, least, legal, x0, y0, cell, tx, ty, apart):
    """Where a search may change layer (12.3). ``best[t]`` is the cost at which
    it reached the middle of triangle t. Five points are tried in each triangle
    reached: the incentre, the centroid, and one towards each corner. A point
    is kept if a via may be there (``legal``, a grid of cells from x0 y0), it
    has room (``roomy``), it is more than ``apart`` from the vias there are
    (tx ty, in order of x), and its cost, ``best[t] + extra``, plus what the
    way on to gx gy costs at least (``onward`` of the straight line: all of
    it, or ``RIDE`` of it for a wire whose net has copper it may run beside)
    stays under ``bound``.
    Returns (x, y, cost, triangle) of the points kept."""
    nx, ny = legal.shape
    n = 0
    for t in range(best.shape[0]):
        if best[t] + extra < bound:
            n += 5
    px, py, cost, tri = np.empty(n), np.empty(n), np.empty(n), np.empty(n, dtype=np.int64)
    n = 0
    for t in range(best.shape[0]):
        c = best[t] + extra
        if not c < bound:
            continue
        a, b, d = tri_v[t, 0], tri_v[t, 1], tri_v[t, 2]
        sa = np.sqrt((vx[b] - vx[d]) * (vx[b] - vx[d]) + (vy[b] - vy[d]) * (vy[b] - vy[d]))  # the side opposite each corner
        sb = np.sqrt((vx[d] - vx[a]) * (vx[d] - vx[a]) + (vy[d] - vy[a]) * (vy[d] - vy[a]))
        sd = np.sqrt((vx[a] - vx[b]) * (vx[a] - vx[b]) + (vy[a] - vy[b]) * (vy[a] - vy[b]))
        ix = (vx[a] * sa + vx[b] * sb + vx[d] * sd) / (sa + sb + sd)
        iy = (vy[a] * sa + vy[b] * sb + vy[d] * sd) / (sa + sb + sd)
        for k in range(5):
            if k == 0:
                x, y = ix, iy
            elif k == 1:
                x, y = (vx[a] + vx[b] + vx[d]) / 3.0, (vy[a] + vy[b] + vy[d]) / 3.0
            else:
                x, y = (vx[tri_v[t, k - 2]] + ix) / 2.0, (vy[tri_v[t, k - 2]] + iy) / 2.0
            i, j = int((x - x0) / cell), int((y - y0) / cell)
            i, j = min(max(i, 0), nx - 1), min(max(j, 0), ny - 1)
            if not legal[i, j] or not c + onward * np.hypot(x - gx, y - gy) < bound:
                continue
            if not roomy(tri_v, vx, vy, corner, pitch, keep, least, t, x, y):
                continue
            free = True
            for q in range(np.searchsorted(tx, x - apart), tx.shape[0]):
                if tx[q] > x + apart:
                    break
                if np.hypot(tx[q] - x, ty[q] - y) <= apart:
                    free = False
                    break
            if free:
                px[n], py[n], cost[n], tri[n] = x, y, c, t
                n += 1
    return px[:n], py[:n], cost[:n], tri[:n]
