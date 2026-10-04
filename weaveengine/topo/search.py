"""Slot-aware topological A* (design section 8).

Hot loop: locals, lists, tuples, ints, floats and heapq only (section 3).
"""
import heapq
import math
from dataclasses import dataclass, field

from weaveengine.topo.costs import CostParams
from weaveengine.topo.planar_map import TERMINAL, PlanarMap
from weaveengine.topo.state import Step, TopoState

SLOT_CAP = 15  # node id is half_edge * 16 + slot; a gate with 15 wires is full
MAX_RETRIES = 12
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
    tx, ty = pmap.pad_centre[dst_pad]
    rad = pmap.pad_radius[dst_pad]
    hypot, push, pop, inf = math.hypot, heapq.heappush, heapq.heappop, math.inf

    banned: set[int] = set()
    for _ in range(MAX_RETRIES):
        g: dict[int, float] = {}
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
                push(heap, (c + (h if h > 0 else 0.0), counter, c, node))
                counter += 1

        while heap:
            _, _, gn, node = pop(heap)
            if gn > g[node]:
                continue
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
            for b, nxt, t, k, cu_e, cu_b, length in trans[he]:
                if kind[b] == TERMINAL and owner[b] != dst_pad:
                    continue
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
                    h = hypot(mids[b][0] - tx, mids[b][1] - ty) - rad
                    counter += 1
                    push(heap, (c + (h if h > 0 else 0.0), counter, c, nn))
        else:
            return None
    return None


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


def flood(pmap: PlanarMap, state: TopoState, src_pad: int, weight: float = 1.0) -> dict[int, float]:
    """Cheapest legal cost from ``src_pad`` to every triangle it can reach
    without crossing a wire or over-filling a gate. Used to place via sites."""
    trans, kind, load, cap = pmap.trans, pmap.edge_kind_list, state.load, state.cap
    order, cnt, tris = state.gate_order, state.corner_cnt, pmap.edge_t_list
    best: dict[int, float] = {}
    g: dict[int, float] = {}
    heap: list[tuple[float, int]] = []
    for e in pmap.pad_edges.get(src_pad, ()):
        n = len(order[e])
        if n >= SLOT_CAP or (n and load[e] + weight - cap[e] > 1e-9):
            continue
        for p in range(n + 1):
            node = (2 * e) << 4 | p
            g[node] = 0.0
            heapq.heappush(heap, (0.0, node))
    while heap:
        gn, node = heapq.heappop(heap)
        if gn > g[node]:
            continue
        he, p = node >> 4, node & 15
        e = he >> 1
        t = tris[e][he & 1]
        if gn < best.get(t, math.inf):
            best[t] = gn
        ne = len(order[e])
        for b, nxt, t, k, cu_e, cu_b, length in trans[he]:
            nb = len(order[b])
            if nxt < 0 or kind[b] == TERMINAL or nb >= SLOT_CAP or load[b] + weight - cap[b] > 1e-9:
                continue
            r = p if cu_e else ne - p
            if r > cnt[t][k]:
                continue
            nn = nxt << 4 | (r if cu_b else nb - r)
            c = gn + length
            if c < g.get(nn, math.inf):
                g[nn] = c
                heapq.heappush(heap, (c, nn))
    return best
