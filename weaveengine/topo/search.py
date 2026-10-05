"""The slot-aware topological search (design section 8): one A*, in ``kernel.py``.

This module prepares its arguments and reads its answer. A search runs on one
layer. It may start from a pad or from seeds, points a via could come through
from another layer, and it reports what it reached besides the goal, so that a
route across layers is a chain of these searches (``plan/path.py``).
"""
import math
from dataclasses import dataclass, field

import numpy as np

from weaveengine.topo import kernel
from weaveengine.topo.costs import CostParams
from weaveengine.topo.planar_map import PlanarMap
from weaveengine.topo.state import Step, TopoState

SLOT_CAP = 15  # node id is half_edge * 16 + slot; a gate with 15 wires is full
MAX_RETRIES = 12
_DEFAULT = CostParams()
_NO_SEEDS = (np.zeros(0, dtype=np.int64), np.zeros(0), np.zeros(0), np.zeros(0))


@dataclass
class Route:
    steps: list[Step]
    cost: float
    length: float                                    # midpoint-to-midpoint estimate
    blocking: set[int] = field(default_factory=set)  # relaxed mode: wires it would cross
    layer: int = 0                                   # set by the planner
    seed: int = -1                                   # the seed it starts from; -1 = it starts on the pad

    @property
    def gates(self) -> tuple[int, ...]:
        return tuple(s[0] for s in self.steps)


class Reach:
    """What a search reached on the way to its goal: per triangle, the cost of
    getting to its middle cell (the part no wire has cut off, where a via site
    would go; inf = not reached), and how to get there."""

    def __init__(self, pmap, state, relaxed, best, node, touched, parent, parent_tr):
        self.best, self._node = best, node
        self._pmap, self._state, self._relaxed = pmap, state, relaxed
        self._touched, self._parent, self._parent_tr = touched, parent, parent_tr
        self._lookup = None

    def route_to(self, t: int) -> Route | None:
        """The way to the middle of triangle ``t`` (its last step crosses into
        ``t``), or None if there is none that could be inserted."""
        node = int(self._node[t])
        if node < 0:
            return None
        if self._lookup is None:
            self._lookup = dict(zip(self._touched.tolist(), zip(self._parent.tolist(), self._parent_tr.tolist())))
        found = _read(self._pmap, self._state, node, self._lookup.__getitem__, float(self.best[t]), self._relaxed)
        return found if isinstance(found, Route) else None


def route(pmap: PlanarMap, state: TopoState, src_pad: int | None, dst_pad: int, params: CostParams | None = None,
          mode: str = "normal", corridor: set[int] | None = None, penalty: dict[int, float] | None = None,
          congestion: bool = True, hard_cap: bool = False, weight: float = 1.0, seeds=None, reach: bool = False,
          target: tuple[float, float] | None = None, bound: float = math.inf, net: int = -1):
    """Cheapest planar path to ``dst_pad`` on this layer, or None.

    It starts from ``src_pad`` and/or from ``seeds`` = (triangles, costs, x, y):
    points in the middle cells of those triangles, each at its cost so far.

    mode: "normal"   - only crossing-free slots are explored;
          "relaxed"  - crossings allowed at ``cross_penalty`` each, reported in
                       ``Route.blocking`` (the result cannot be inserted);
          "corridor" - like normal, restricted to the edge set ``corridor``.
    ``penalty`` adds a per-gate cost (candidate diversification). ``hard_cap``
    forbids exceeding capacity instead of pricing it. ``weight`` is the load
    the wire puts on a gate (1 for a base-width trace). ``net``: the wire's
    net; beside a wire of the same net it is the same trace and costs no room
    and next to no length (11).

    With ``reach``, returns (route or None, Reach). ``target``: where the
    goal is, for a search on a layer the pad itself is not on (it then cannot
    end there, only say what it reached). ``bound``: nothing dearer than this
    is of interest.
    """
    if mode not in ("normal", "relaxed", "corridor"):
        raise ValueError(f"unknown search mode {mode!r}")
    if mode == "corridor" and corridor is None:
        raise ValueError("corridor mode needs a corridor")
    params = params or _DEFAULT
    relaxed = mode == "relaxed"
    starts = np.array(pmap.pad_edges.get(src_pad, ()) if src_pad is not None else (), dtype=np.int32)
    tris, costs, xs, ys = seeds if seeds is not None else _NO_SEEDS
    here = dst_pad in pmap.pad_edges
    if not (here or (reach and target is not None)) or src_pad == dst_pad or not (len(starts) or len(tris)):
        return (None, None) if reach else None
    pen = no_pen = np.zeros(1)
    if penalty:
        pen = np.zeros(pmap.num_edges)
        for e, v in penalty.items():
            pen[e] = v
    cor = np.zeros(1, dtype=np.uint8)
    if mode == "corridor":
        cor = np.zeros(pmap.num_edges, dtype=np.uint8)
        cor[list(corridor)] = 1
    tx, ty = pmap.pad_centre[dst_pad] if here else target
    radius = pmap.pad_radius[dst_pad] if here else 0.0
    best = np.full(pmap.num_triangles, np.inf)
    best_node = np.full(pmap.num_triangles, -1, dtype=np.int64)
    search = kernel.astar if kernel.AVAILABLE else kernel.astar_plain
    banned: list[int] = []
    found, seen = None, None
    tb = None
    riding = state.beside(net) if net >= 0 else {}
    try:
        for _ in range(MAX_RETRIES):
            tb = pmap.__dict__.get("_kernel_tables")
            if tb is None:
                tb = pmap.__dict__["_kernel_tables"] = kernel.Tables(pmap)
                tb.banned[banned] = 1
            for e, places in riding.items():
                tb.ride[e] = places
            best[:], best_node[:] = np.inf, -1
            try:
                goal, cost, touched = search(
                    starts, np.ascontiguousarray(tris, dtype=np.int64), np.ascontiguousarray(costs, dtype=np.float64),
                    np.ascontiguousarray(xs, dtype=np.float64), np.ascontiguousarray(ys, dtype=np.float64), dst_pad,
                    pmap.tri_e, pmap.tri_v, pmap.edge_v, pmap.edge_t,
                    tb.b, tb.nxt, tb.t, tb.k, tb.cue, tb.cub, tb.length, tb.n, tb.kind, tb.owner, tb.mid,
                    state.count, state.corner, state.load, state.cap, state.hist, pen, pen is not no_pen, cor, mode == "corridor",
                    relaxed, params.pres_fac if congestion else 0.0, congestion, params.cross_penalty, hard_cap, weight, tb.ride,
                    tx, ty, radius, params.h_weight, bound, tb.g, tb.parent, tb.parent_tr, tb.banned, tb.touched,
                    tb.heap_f, tb.heap_g, tb.heap_n, best, best_node)
            except Exception as error:  # the compiled kernel itself failed: never crash, fall back for good
                from weaveengine import accel
                accel.failed("search", error)
                pmap.__dict__.pop("_kernel_tables", None)  # its workspace may be half-used
                search = kernel.astar_plain
                continue
            idx = tb.touched[:touched]
            found = None
            if goal >= 0:
                found = _read(pmap, state, int(goal), lambda n: (int(tb.parent[n]), int(tb.parent_tr[n])), float(cost), relaxed)
            if reach and not isinstance(found, int):
                seen = Reach(pmap, state, relaxed, best, best_node, idx.copy(), tb.parent[idx], tb.parent_tr[idx])
            tb.g[idx] = np.inf
            tb.parent[idx] = -1
            if goal == -2:  # the workspace was too small (many seeds): a larger one, and again
                size = 4 * len(tb.heap_f)
                tb.heap_f, tb.heap_g, tb.heap_n = np.zeros(size), np.zeros(size), np.zeros(size, dtype=np.int32)
                tb.g[:] = np.inf
                tb.parent[:] = -1
                continue
            if isinstance(found, int):
                tb.banned[found] = 1  # the cheapest path crossed a gate twice (7.3): forbid it, search again
                banned.append(found)
                continue
            break
        else:
            found = None
    finally:
        if tb is not None and pmap.__dict__.get("_kernel_tables") is tb:
            tb.banned[banned] = 0
            tb.ride[list(riding)] = 0
    return (found, seen) if reach else found


def _read(pmap: PlanarMap, state: TopoState, node: int, parent_of, cost: float, relaxed: bool) -> "Route | int":
    """The route that ends at ``node``, or (when it crosses a gate twice) the
    node id of the repeat crossing. ``parent_of(node)`` gives (parent, which
    transition led here); a parent below zero ends the walk: -1 at a pad edge,
    -2 - i at seed i."""
    tables = pmap.__dict__["_kernel_tables"]
    rev: list[Step] = []
    nodes: list[int] = []
    blocking: set[int] = set()
    while True:
        prev, j = parent_of(node)
        if prev < 0:
            break
        he = prev >> 4
        t, k = int(tables.t[he, j]), int(tables.k[he, j])
        rev.append(((node >> 4) >> 1, t, k, node & 15))
        nodes.append(node)
        if relaxed:
            e, p = he >> 1, prev & 15
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
    return Route(steps, cost, length, blocking, seed=-1 if prev == -1 else -2 - prev)
