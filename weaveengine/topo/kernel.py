"""Compiled A* kernel for the slot-aware search (design sections 3, 8 and 19.7).

The same algorithm as the pure-Python loop in ``search.py``, on flat arrays and
compiled with numba. numba is optional: without it (or with the environment
variable WEAVEENGINE_NO_NUMBA set) the Python loop is used.
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


class Tables:
    """Flat copies of a planar map's transition tables, plus the search workspace."""

    def __init__(self, pmap):
        # Room for more half-edges than the map has: via sites add edges, and
        # the workspace below is too large to copy every time one does.
        e2 = self.room = 2 * pmap.num_edges + max(256, pmap.num_edges // 4)
        self.b = np.zeros((e2, 2), dtype=np.int32)
        self.nxt = np.zeros((e2, 2), dtype=np.int32)
        self.t = np.zeros((e2, 2), dtype=np.int32)
        self.k = np.zeros((e2, 2), dtype=np.int32)
        self.cue = np.zeros((e2, 2), dtype=np.uint8)
        self.cub = np.zeros((e2, 2), dtype=np.uint8)
        self.length = np.zeros((e2, 2), dtype=np.float64)
        self.n = np.zeros(e2, dtype=np.int32)
        self.update(pmap, range(2 * pmap.num_edges))
        nodes = e2 * SLOTS
        self.g = np.full(nodes, np.inf)
        self.parent = np.full(nodes, -1, dtype=np.int32)
        self.parent_tr = np.zeros(nodes, dtype=np.int8)
        self.banned = np.zeros(nodes, dtype=np.uint8)
        self.touched = np.zeros(nodes, dtype=np.int32)
        size = max(4 * nodes, 1024)
        self.heap_f = np.zeros(size)
        self.heap_g = np.zeros(size)
        self.heap_n = np.zeros(size, dtype=np.int32)
        self.no_penalty = np.zeros(1)
        self.no_corridor = np.zeros(1, dtype=np.uint8)

    def update(self, pmap, half_edges) -> bool:
        """Copies the transitions of ``half_edges`` and the per-edge tables from
        the map. Returns False if the map has outgrown the room: build anew."""
        if 2 * pmap.num_edges > self.room:
            return False
        trans = pmap.trans
        for he in half_edges:
            row = trans[he]
            self.n[he] = len(row)
            for j, (b, nxt, t, k, cue, cub, length) in enumerate(row):
                self.b[he, j], self.nxt[he, j], self.t[he, j], self.k[he, j] = b, nxt, t, k
                self.cue[he, j], self.cub[he, j], self.length[he, j] = cue, cub, length
        self.kind = pmap.edge_kind.astype(np.int8)
        self.owner = pmap.edge_owner.astype(np.int32)
        self.mid = np.ascontiguousarray(pmap.edge_mid, dtype=np.float64)
        return True


@njit(cache=True, nogil=True)
def astar(starts, dst_pad, tb, tnxt, tt, tk, tcue, tcub, tlen, tn, kind, owner, mid,
          count, corner, load, cap, hist, penalty, use_penalty, corridor, use_corridor,
          relaxed, pres, use_hist, cross_pen, hard_cap, weight, tx, ty, rad, hw,
          g, parent, parent_tr, banned, touched, heap_f, heap_g, heap_n):
    """Returns (goal node or -1 if none or -2 if the heap overflowed, cost, nodes touched).
    ``g``/``parent`` are left filled for the caller to read the path, which must
    then reset the first ``touched`` entries."""
    n_touched = 0
    size = 0
    limit = heap_f.shape[0]
    for si in range(starts.shape[0]):
        e = starts[si]
        n = count[e]
        if n >= SLOTS - 1 or (use_corridor and corridor[e] == 0):
            continue
        over = load[e] + weight - cap[e] if n > 0 else 0.0
        if over > 1e-9 and hard_cap:
            continue
        c = 0.0
        if over > 1e-9:
            c += pres * (over if over > 1.0 else 1.0)
        if use_hist:
            c += hist[e]
        if use_penalty:
            c += penalty[e]
        h = np.hypot(mid[e, 0] - tx, mid[e, 1] - ty) - rad
        h = 0.0 if h < 0.0 else h * hw
        for p in range(n + 1):
            node = (2 * e) * SLOTS + p
            if banned[node]:
                continue
            g[node] = c
            parent[node] = -1
            touched[n_touched] = node
            n_touched += 1
            # push
            i = size
            size += 1
            f = c + h
            while i > 0:
                up = (i - 1) >> 1
                if heap_f[up] <= f:
                    break
                heap_f[i], heap_g[i], heap_n[i] = heap_f[up], heap_g[up], heap_n[up]
                i = up
            heap_f[i], heap_g[i], heap_n[i] = f, c, node

    while size > 0:
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
            return node, gn, n_touched
        ne = count[e]
        for j in range(tn[he]):
            b = tb[he, j]
            if kind[b] == 2 and owner[b] != dst_pad:
                continue
            if use_corridor and corridor[b] == 0:
                continue
            nb = count[b]
            if nb >= SLOTS - 1:
                continue
            r = p if tcue[he, j] else ne - p
            lim = corner[tt[he, j], tk[he, j]]
            c = gn + tlen[he, j]
            if r > lim:
                if not relaxed:
                    continue
                c += cross_pen * (r - lim)
                r = lim
            nxt = tnxt[he, j]
            over = load[b] + weight - cap[b]
            if over > 1e-9 and (nb > 0 or nxt >= 0):
                if hard_cap:
                    continue
                c += pres * (over if over > 1.0 else 1.0)
            if use_hist:
                c += hist[b]
            if use_penalty:
                c += penalty[b]
            pb = r if tcub[he, j] else nb - r
            nn = (nxt if nxt >= 0 else 2 * b + 1) * SLOTS + pb
            if c < g[nn] and not banned[nn]:
                if g[nn] == np.inf:
                    touched[n_touched] = nn
                    n_touched += 1
                g[nn] = c
                parent[nn] = node
                parent_tr[nn] = j
                h = np.hypot(mid[b, 0] - tx, mid[b, 1] - ty) - rad
                h = 0.0 if h < 0.0 else h * hw
                if size >= limit:
                    return -2, 0.0, n_touched
                i = size
                size += 1
                f = c + h
                while i > 0:
                    up = (i - 1) >> 1
                    if heap_f[up] <= f:
                        break
                    heap_f[i], heap_g[i], heap_n[i] = heap_f[up], heap_g[up], heap_n[up]
                    i = up
                heap_f[i], heap_g[i], heap_n[i] = f, c, nn
    return -1, 0.0, n_touched


@njit(cache=True, nogil=True)
def flood(starts, seed_tri, seed_cost, seed_x, seed_y, dst_pad, tri_e, tri_v, edge_v, edge_t,
          tb, tnxt, tt, tk, tcue, tcub, tlen, tn, kind, owner, mid, count, corner, load, cap, weight,
          g, label, touched, heap_f, heap_n, best, origin, goal):
    """Cheapest legal length to the middle cell of every triangle that can be
    reached without crossing a wire or over-filling a gate (design 12.4). The
    middle cell is the part of a triangle no wire has cut off, where a via
    site would be put.

    The search starts from the pad edges ``starts`` (cost 0) and from seeds: a
    point in the middle cell of triangle ``seed_tri[i]`` at ``seed_cost[i]``
    (a via from another layer). Fills, per triangle, ``best`` and ``origin``
    (the seed the cheapest way came from; -1 = a pad edge), and ``goal`` with
    the cost and origin of the cheapest arrival at ``dst_pad``.
    Returns the number of ``g`` entries touched, or -1 if the heap overflowed."""
    n_touched = 0
    size = 0
    limit = heap_f.shape[0]
    for si in range(starts.shape[0]):
        e = starts[si]
        n = count[e]
        if n >= SLOTS - 1 or (n > 0 and load[e] + weight - cap[e] > 1e-9):
            continue
        for p in range(n + 1):
            node = (2 * e) * SLOTS + p
            if g[node] == np.inf:
                touched[n_touched] = node
                n_touched += 1
            g[node] = 0.0
            label[node] = -1
            if size >= limit:
                return -1
            heap_f[size], heap_n[size] = 0.0, node
            size += 1
    for si in range(seed_tri.shape[0]):
        t = seed_tri[si]
        c0 = seed_cost[si]
        if c0 < best[t]:
            best[t] = c0
            origin[t] = si
        for k in range(3):
            e = tri_e[t, k]
            nb = count[e]
            c = c0 + np.hypot(mid[e, 0] - seed_x[si], mid[e, 1] - seed_y[si])
            if kind[e] == 2:
                if owner[e] == dst_pad and not (nb > 0 and load[e] + weight - cap[e] > 1e-9) and c < goal[0]:
                    goal[0], goal[1] = c, si
                continue
            side = 1 if edge_t[e, 0] == t else 0
            if kind[e] != 0 or edge_t[e, side] < 0 or nb >= SLOTS - 1 or load[e] + weight - cap[e] > 1e-9:
                continue
            u = edge_v[e, 0]
            ku = 0 if tri_v[t, 0] == u else (1 if tri_v[t, 1] == u else 2)
            node = (2 * e + side) * SLOTS + corner[t, ku]  # the middle of the gate, seen from inside t
            if c < g[node]:
                if g[node] == np.inf:
                    touched[n_touched] = node
                    n_touched += 1
                g[node] = c
                label[node] = si
                if size >= limit:
                    return -1
                i = size
                size += 1
                while i > 0:
                    up = (i - 1) >> 1
                    if heap_f[up] <= c:
                        break
                    heap_f[i], heap_n[i] = heap_f[up], heap_n[up]
                    i = up
                heap_f[i], heap_n[i] = c, node
    while size > 0:
        gn = heap_f[0]
        node = heap_n[0]
        size -= 1
        if size > 0:
            f, nd = heap_f[size], heap_n[size]
            i = 0
            while True:
                child = 2 * i + 1
                if child >= size:
                    break
                if child + 1 < size and heap_f[child + 1] < heap_f[child]:
                    child += 1
                if heap_f[child] >= f:
                    break
                heap_f[i], heap_n[i] = heap_f[child], heap_n[child]
                i = child
            heap_f[i], heap_n[i] = f, nd
        if gn > g[node]:
            continue
        he = node // SLOTS
        p = node - he * SLOTS
        ne = count[he >> 1]
        if tn[he] > 0:
            t = tt[he, 0]
            r = p if tcue[he, 0] else ne - p
            if r == corner[t, tk[he, 0]] and gn < best[t]:
                best[t] = gn
                origin[t] = label[node]
        for j in range(tn[he]):
            b = tb[he, j]
            nxt = tnxt[he, j]
            nb = count[b]
            r = p if tcue[he, j] else ne - p
            if r > corner[tt[he, j], tk[he, j]]:
                continue
            c = gn + tlen[he, j]
            if kind[b] == 2:
                if owner[b] == dst_pad and not (nb > 0 and load[b] + weight - cap[b] > 1e-9) and c < goal[0]:
                    goal[0], goal[1] = c, label[node]
                continue
            if nxt < 0 or nb >= SLOTS - 1 or load[b] + weight - cap[b] > 1e-9:
                continue
            nn = nxt * SLOTS + (r if tcub[he, j] else nb - r)
            if c < g[nn]:
                if g[nn] == np.inf:
                    touched[n_touched] = nn
                    n_touched += 1
                g[nn] = c
                label[nn] = label[node]
                if size >= limit:
                    return -1
                i = size
                size += 1
                while i > 0:
                    up = (i - 1) >> 1
                    if heap_f[up] <= c:
                        break
                    heap_f[i], heap_n[i] = heap_f[up], heap_n[up]
                    i = up
                heap_f[i], heap_n[i] = c, nn
    return n_touched


@njit(cache=True, nogil=True)
def locate(tri_v, tri_n, vx, vy, px, py, out, hint, x0, y0, cell):
    """Triangle containing each point (-1 if none). ``hint`` is a grid of
    cells (side ``cell``, origin x0 y0) that this call fills with a triangle
    near each cell; the search walks from there, and scans every triangle only
    where the walk runs into a wall."""
    n_tri = tri_v.shape[0]
    nx, ny = hint.shape
    hint[:, :] = -1
    for big in (1, 0):  # large triangles first, so that small ones, which say more, are kept
        for t in range(n_tri):
            a, b, c = tri_v[t, 0], tri_v[t, 1], tri_v[t, 2]
            i0 = int((min(vx[a], vx[b], vx[c]) - x0) / cell)
            i1 = int((max(vx[a], vx[b], vx[c]) - x0) / cell)
            j0 = int((min(vy[a], vy[b], vy[c]) - y0) / cell)
            j1 = int((max(vy[a], vy[b], vy[c]) - y0) / cell)
            if ((i1 - i0 + 1) * (j1 - j0 + 1) > 16) != (big == 1):
                continue
            for i in range(max(i0, 0), min(i1, nx - 1) + 1):
                for j in range(max(j0, 0), min(j1, ny - 1) + 1):
                    hint[i, j] = t
    cur = 0
    for i in range(px.shape[0]):
        x, y = px[i], py[i]
        found = -1
        ci, cj = int((x - x0) / cell), int((y - y0) / cell)
        t = cur
        if 0 <= ci < nx and 0 <= cj < ny and hint[ci, cj] >= 0:
            t = hint[ci, cj]
        for _ in range(n_tri):
            move = -1
            for k in range(3):
                a, b = tri_v[t, (k + 1) % 3], tri_v[t, (k + 2) % 3]
                if (vx[b] - vx[a]) * (y - vy[a]) - (vy[b] - vy[a]) * (x - vx[a]) < -1e-12:
                    move = k
                    break
            if move < 0:
                found = t
                break
            t = tri_n[t, move]
            if t < 0:
                break
        if found < 0:
            for t in range(n_tri):
                inside = True
                for k in range(3):
                    a, b = tri_v[t, (k + 1) % 3], tri_v[t, (k + 2) % 3]
                    if (vx[b] - vx[a]) * (y - vy[a]) - (vy[b] - vy[a]) * (x - vx[a]) < -1e-12:
                        inside = False
                        break
                if inside:
                    found = t
                    break
        out[i] = found
        if found >= 0:
            cur = found
