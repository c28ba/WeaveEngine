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
        e2 = 2 * pmap.num_edges
        self.b = np.zeros((e2, 2), dtype=np.int32)
        self.nxt = np.zeros((e2, 2), dtype=np.int32)
        self.t = np.zeros((e2, 2), dtype=np.int32)
        self.k = np.zeros((e2, 2), dtype=np.int32)
        self.cue = np.zeros((e2, 2), dtype=np.uint8)
        self.cub = np.zeros((e2, 2), dtype=np.uint8)
        self.length = np.zeros((e2, 2), dtype=np.float64)
        self.n = np.zeros(e2, dtype=np.int32)
        for he, row in enumerate(pmap.trans):
            self.n[he] = len(row)
            for j, (b, nxt, t, k, cue, cub, length) in enumerate(row):
                self.b[he, j], self.nxt[he, j], self.t[he, j], self.k[he, j] = b, nxt, t, k
                self.cue[he, j], self.cub[he, j], self.length[he, j] = cue, cub, length
        self.kind = pmap.edge_kind.astype(np.int8)
        self.owner = pmap.edge_owner.astype(np.int32)
        self.mid = np.ascontiguousarray(pmap.edge_mid, dtype=np.float64)
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
