"""Slot-aware topological A* (design section 8).

Hot loop: locals, lists, tuples, ints, floats and heapq only (section 3).
"""
import heapq
import math
from dataclasses import dataclass, field

import numpy as np

from weaveengine.topo import kernel
from weaveengine.topo.costs import CostParams
from weaveengine.topo.planar_map import TERMINAL, PlanarMap
from weaveengine.topo.state import Step, TopoState

SLOT_CAP = 15  # node id is half_edge * 16 + slot; a gate with 15 wires is full
MAX_RETRIES = 12
PROBE_AFTER = 600    # node expansions before checking whether the target is fenced in
PROBE_LIMIT = 4000   # how much of the target's side to explore for that check
_DEFAULT = CostParams()


@dataclass
class Route:
    steps: list[Step]
    cost: float
    length: float                                    # midpoint-to-midpoint estimate
    blocking: set[int] = field(default_factory=set)  # relaxed mode: wires it would cross
    layer: int = 0                                   # set by the planner

    @property
    def gates(self) -> tuple[int, ...]:
        return tuple(s[0] for s in self.steps)


def route(pmap: PlanarMap, state: TopoState, src_pad: int, dst_pad: int, params: CostParams | None = None,
          mode: str = "normal", corridor: set[int] | None = None, penalty: dict[int, float] | None = None,
          congestion: bool = True, hard_cap: bool = False, weight: float = 1.0) -> Route | None:
    """Cheapest planar path from ``src_pad`` to ``dst_pad``, or None.

    mode: "normal"   - only crossing-free slots are explored;
          "relaxed"  - crossings allowed at ``cross_penalty`` each, reported in
                       ``Route.blocking`` (the result cannot be inserted);
          "corridor" - like normal, restricted to the edge set ``corridor``.
    ``penalty`` adds a per-gate cost (candidate diversification). ``hard_cap``
    forbids exceeding capacity instead of pricing it. ``weight`` is the load
    the wire puts on a gate (1 for a base-width trace).
    """
    if mode not in ("normal", "relaxed", "corridor"):
        raise ValueError(f"unknown search mode {mode!r}")
    if mode == "corridor" and corridor is None:
        raise ValueError("corridor mode needs a corridor")
    if mode != "corridor":
        corridor = None
    params = params or _DEFAULT
    starts = pmap.pad_edges.get(src_pad)
    if not starts or dst_pad not in pmap.pad_edges or src_pad == dst_pad:
        return None

    trans, kind, owner, mids = pmap.trans, pmap.edge_kind_list, pmap.edge_owner_list, pmap.edge_mid_list
    order, cnt, cap, hist, load = state.gate_order, state.corner_cnt, state.cap, state.hist, state.load
    relaxed = mode == "relaxed"
    pres = params.pres_fac if congestion else 0.0
    use_hist = congestion
    cross_pen = params.cross_penalty
    hw = params.h_weight
    tx, ty = pmap.pad_centre[dst_pad]
    rad = pmap.pad_radius[dst_pad]
    hypot, push, pop, inf = math.hypot, heapq.heappush, heapq.heappop, math.inf

    if kernel.AVAILABLE:
        try:
            return _route_compiled(pmap, state, src_pad, dst_pad, params, relaxed, corridor, penalty, congestion,
                                   hard_cap, weight, starts)
        except MemoryError:
            pass  # search workspace too small for this one: do it in Python
        except Exception as error:  # the compiled kernel itself failed: never crash, fall back for good
            from weaveengine import accel
            accel.failed("search", error)
            pmap.__dict__.pop("_kernel_tables", None)  # its workspace may be half-used

    banned: set[int] = set()
    for _ in range(MAX_RETRIES):
        g: dict[int, float] = {}
        used: dict[int, int] = {}  # per node: bit set of the gates on the path that reached it
        parent: dict[int, tuple[int, int, int]] = {}
        heap: list[tuple[float, int, float, int]] = []
        counter = 0
        for e in starts:
            n = len(order[e])
            if n >= SLOT_CAP or (corridor is not None and e not in corridor):
                continue
            over = load[e] + weight - cap[e] if n else 0.0  # one wire always fits its own pad edge
            if over > 1e-9 and hard_cap:
                continue
            c = (pres * (over if over > 1.0 else 1.0) if over > 1e-9 else 0.0) + (hist[e] if use_hist else 0.0)
            if penalty:
                c += penalty.get(e, 0.0)
            h = hypot(mids[e][0] - tx, mids[e][1] - ty) - rad
            for p in range(n + 1):
                node = (2 * e) << 4 | p
                g[node] = c
                used[node] = 1 << e
                push(heap, (c + (h * hw if h > 0 else 0.0), counter, c, node))
                counter += 1

        pops = 0
        while heap:
            _, _, gn, node = pop(heap)
            if gn > g[node]:
                continue
            pops += 1
            if pops == PROBE_AFTER and not relaxed:
                # The search is taking long. A wire that cannot be reached is
                # usually fenced into a small region: look at the target's
                # side before exploring the rest of the board from this one.
                if _fenced_in(pmap, state, dst_pad, src_pad, weight, hard_cap, corridor):
                    return None
            he = node >> 4
            p = node & 15
            e = he >> 1
            if he & 1 and kind[e] == TERMINAL:
                found = _reconstruct(pmap, state, node, parent, gn, relaxed)
                if isinstance(found, Route):
                    return found
                # The cheapest path crossed a gate twice (7.3). Forbid the second
                # crossing and search again.
                banned.add(found)
                break
            ne = len(order[e])
            mask = used[node]
            for b, nxt, t, k, cu_e, cu_b, length in trans[he]:
                if kind[b] == TERMINAL and owner[b] != dst_pad:
                    continue
                if mask >> b & 1 and not relaxed:
                    continue  # a path must never cross the same gate twice (7.3)
                if corridor is not None and b not in corridor:
                    continue
                nb = len(order[b])
                if nb >= SLOT_CAP:
                    continue
                r = p if cu_e else ne - p
                lim = cnt[t][k]
                c = gn + length
                if r > lim:
                    # The wire would have to cross wires that cut this corner.
                    if not relaxed:
                        continue
                    c += cross_pen * (r - lim)
                    r = lim
                over = load[b] + weight - cap[b]
                if over > 1e-9 and (nb or nxt >= 0):  # one wire always fits its own pad edge
                    if hard_cap:
                        continue
                    c += pres * (over if over > 1.0 else 1.0)
                if use_hist:
                    c += hist[b]
                if penalty:
                    c += penalty.get(b, 0.0)
                nn = (nxt if nxt >= 0 else 2 * b + 1) << 4 | (r if cu_b else nb - r)
                if c < g.get(nn, inf) and nn not in banned:
                    g[nn] = c
                    parent[nn] = (node, t, k)
                    used[nn] = mask | 1 << b
                    h = hypot(mids[b][0] - tx, mids[b][1] - ty) - rad
                    counter += 1
                    push(heap, (c + (h * hw if h > 0 else 0.0), counter, c, nn))
        else:
            return None
    return None


def _route_compiled(pmap, state, src_pad, dst_pad, params, relaxed, corridor, penalty, congestion, hard_cap,
                    weight, starts) -> Route | None:
    """The same search on the compiled kernel."""
    tb = pmap.__dict__.get("_kernel_tables")
    if tb is None:
        tb = pmap.__dict__["_kernel_tables"] = kernel.Tables(pmap)
    if penalty:
        pen = np.zeros(pmap.num_edges)
        for e, v in penalty.items():
            pen[e] = v
    else:
        pen = tb.no_penalty
    if corridor is not None:
        cor = np.zeros(pmap.num_edges, dtype=np.uint8)
        cor[list(corridor)] = 1
    else:
        cor = tb.no_corridor
    tx, ty = pmap.pad_centre[dst_pad]
    start_arr = np.array(starts, dtype=np.int32)
    banned_nodes: list[int] = []
    try:
        for _ in range(MAX_RETRIES):
            goal, cost, touched = kernel.astar(
                start_arr, dst_pad, tb.b, tb.nxt, tb.t, tb.k, tb.cue, tb.cub, tb.length, tb.n, tb.kind, tb.owner, tb.mid,
                state.count, state.corner, state.load, state.cap, state.hist, pen, bool(penalty), cor, corridor is not None,
                relaxed, params.pres_fac if congestion else 0.0, congestion, params.cross_penalty, hard_cap, weight,
                tx, ty, pmap.pad_radius[dst_pad], params.h_weight, tb.g, tb.parent, tb.parent_tr, tb.banned, tb.touched,
                tb.heap_f, tb.heap_g, tb.heap_n)
            found = None
            if goal >= 0:
                parent: dict[int, tuple[int, int, int]] = {}
                node = goal
                while tb.parent[node] >= 0:
                    prev = int(tb.parent[node])
                    j = tb.parent_tr[node]
                    he = prev >> 4
                    parent[node] = (prev, int(tb.t[he, j]), int(tb.k[he, j]))
                    node = prev
                found = _reconstruct(pmap, state, int(goal), parent, float(cost), relaxed)
            idx = tb.touched[:touched]
            tb.g[idx] = np.inf
            tb.parent[idx] = -1
            if goal == -2:
                raise MemoryError("search heap overflow")
            if goal < 0:
                return None
            if isinstance(found, Route):
                return found
            tb.banned[found] = 1  # the cheapest path crossed a gate twice (7.3): forbid it, search again
            banned_nodes.append(found)
        return None
    finally:
        if banned_nodes:
            tb.banned[banned_nodes] = 0


def _fenced_in(pmap: PlanarMap, state: TopoState, pad: int, other: int, weight: float, hard_cap: bool,
               corridor: set[int] | None) -> bool:
    """True if everything reachable from ``pad`` without crossing a wire has
    been explored (within PROBE_LIMIT) and ``other`` is not there."""
    trans, kind, owner = pmap.trans, pmap.edge_kind_list, pmap.edge_owner_list
    order, cnt, load, cap = state.gate_order, state.corner_cnt, state.load, state.cap
    seen: set[int] = set()
    stack: list[int] = []
    for e in pmap.pad_edges.get(pad, ()):
        n = len(order[e])
        if n >= SLOT_CAP or (corridor is not None and e not in corridor):
            continue
        for p in range(n + 1):
            stack.append((2 * e) << 4 | p)
    seen.update(stack)
    while stack:
        if len(seen) > PROBE_LIMIT:
            return False
        node = stack.pop()
        he, p = node >> 4, node & 15
        ne = len(order[he >> 1])
        for b, nxt, t, k, cu_e, cu_b, _ in trans[he]:
            if kind[b] == TERMINAL:
                if owner[b] == other:
                    return False
                continue
            if corridor is not None and b not in corridor:
                continue
            nb = len(order[b])
            if nb >= SLOT_CAP or (hard_cap and load[b] + weight - cap[b] > 1e-9):
                continue
            r = p if cu_e else ne - p
            if r > cnt[t][k]:
                continue
            nn = nxt << 4 | (r if cu_b else nb - r)
            if nn not in seen:
                seen.add(nn)
                stack.append(nn)
    return True


def _reconstruct(pmap: PlanarMap, state: TopoState, goal: int, parent: dict, cost: float, relaxed: bool) -> "Route | int":
    """The route, or (when it crosses a gate twice) the node id of the repeat crossing."""
    rev: list[Step] = []
    nodes: list[int] = []
    blocking: set[int] = set()
    node = goal
    while node in parent:
        prev, t, k = parent[node]
        rev.append(((node >> 4) >> 1, t, k, node & 15))
        nodes.append(node)
        if relaxed:
            e, p = (prev >> 4) >> 1, prev & 15
            wires = state.gate_order[e]
            if pmap.tri_v_list[t][k] != pmap.edge_v_list[e][0]:
                wires = wires[::-1]
                p = len(wires) - p
            blocking.update(wires[state.corner_cnt[t][k]:p])
        node = prev
    rev.append(((node >> 4) >> 1, -1, -1, node & 15))
    nodes.append(node)
    steps = rev[::-1]
    gates = [s[0] for s in steps]
    if not relaxed and len(set(gates)) != len(gates):
        seen: set[int] = set()
        for gate, nid in zip(gates, nodes[::-1]):
            if gate in seen:
                return nid
            seen.add(gate)
    mids = pmap.edge_mid_list
    length = sum(math.hypot(mids[a][0] - mids[b][0], mids[a][1] - mids[b][1]) for a, b in zip(gates, gates[1:]))
    return Route(steps, cost, length, blocking)


_NONE_I = np.zeros(0, dtype=np.int64)
_NONE_F = np.zeros(0)


def flood(pmap: PlanarMap, state: TopoState, src_pad: int | None = None, weight: float = 1.0, seeds=None, dst_pad: int = -1):
    """Legal reach on one layer, for placing vias (design 12.4).

    Starts from the edges of ``src_pad`` and/or from ``seeds`` = (triangles,
    costs, x, y): points in the middle cells of those triangles, each at its
    cost so far. Returns (best, origin, goal): per triangle the cheapest legal
    length to its middle cell (inf = not reachable) and the index of the seed
    that way came from (-1 = the pad); and (cost, origin) of the cheapest
    arrival at ``dst_pad`` (cost inf = not reached). Nothing crosses a wire or
    over-fills a gate.
    """
    best = np.full(pmap.num_triangles, np.inf)
    origin = np.full(pmap.num_triangles, -1, dtype=np.int64)
    goal = np.array([np.inf, -1.0])
    starts = np.array(pmap.pad_edges.get(src_pad, ()) if src_pad is not None else (), dtype=np.int32)
    tris, costs, xs, ys = seeds if seeds is not None else (_NONE_I, _NONE_F, _NONE_F, _NONE_F)
    if not len(starts) and not len(tris):
        return best, origin, (math.inf, -1)
    for _ in range(2):
        tb = pmap.__dict__.get("_kernel_tables")
        if tb is None:
            tb = pmap.__dict__["_kernel_tables"] = kernel.Tables(pmap)
        touched = kernel.flood(starts, np.ascontiguousarray(tris, dtype=np.int64), np.ascontiguousarray(costs, dtype=np.float64),
                               np.ascontiguousarray(xs, dtype=np.float64), np.ascontiguousarray(ys, dtype=np.float64), dst_pad,
                               pmap.tri_e, pmap.tri_v, pmap.edge_v, pmap.edge_t,
                               tb.b, tb.nxt, tb.t, tb.k, tb.cue, tb.cub, tb.length, tb.n, tb.kind, tb.owner, tb.mid,
                               state.count, state.corner, state.load, state.cap, weight,
                               tb.g, tb.parent, tb.touched, tb.heap_f, tb.heap_n, best, origin, goal)
        if touched >= 0:
            idx = tb.touched[:touched]
            tb.g[idx] = np.inf
            tb.parent[idx] = -1
            return best, origin, (float(goal[0]), int(goal[1]))
        # The workspace was too small (many seeds): make a larger one and go again.
        tb.g[:] = np.inf
        tb.parent[:] = -1
        size = 4 * len(tb.heap_f)
        tb.heap_f, tb.heap_g, tb.heap_n = np.zeros(size), np.zeros(size), np.zeros(size, dtype=np.int32)
        best[:], origin[:], goal[:] = np.inf, -1, (np.inf, -1.0)
    return best, origin, (math.inf, -1)
