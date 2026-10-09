"""The slot-aware topological search (design section 8): one A*, in ``kernel.py``.

This module prepares its arguments and reads its answer. A search runs on one
layer. It may start from a pad or from seeds, points a via could come through
from another layer, and it reports what it reached besides the goal, so that a
route across layers is a chain of these searches (``plan/path.py``).
"""
import math
from dataclasses import dataclass, field

import numpy as np
from scipy import ndimage

from weaveengine.topo import kernel
from weaveengine.topo.costs import CostParams
from weaveengine.topo.planar_map import PlanarMap
from weaveengine.topo.sites import FAR
from weaveengine.topo.state import Step, TopoState

SLOT_CAP = 15  # node id is half_edge * 16 + slot; a gate with 15 wires is full
_DEFAULT = CostParams()
_NO_SEEDS = (np.zeros(0, dtype=np.int64), np.zeros(0), np.zeros(0), np.zeros(0))


@dataclass
class Route:
    steps: list[Step]
    cost: float
    length: float                                    # midpoint-to-midpoint estimate
    blocking: set[int] = field(default_factory=set)  # relaxed mode: wires it would cross
    layer: int = 0                                   # set by the planner
    seed: int = -1                                   # the seed it starts from; -1 = it starts on a pad
    lead: list = field(default_factory=list)         # the pieces before this one: it came through pads of its net (11)
    pad: int = -1                                    # the pad its last step ends on (-1: it ends in a triangle)

    @property
    def gates(self) -> tuple[int, ...]:
        return tuple(s[0] for s in self.steps)


class Reach:
    """What a search reached on the way to its goal: per triangle, the cost of
    getting to its middle cell (the part no wire has cut off, where a via site
    would go; inf = not reached), and how to get there."""

    def __init__(self, pmap, state, relaxed, best, node, touched, parent, parent_tr, pads):
        self.best, self._node = best, node
        self.pads = pads  # pad of the wire's net that was reached -> (cost, node)
        self._pmap, self._state, self._relaxed = pmap, state, relaxed
        self._touched, self._parent, self._parent_tr = touched, parent, parent_tr
        self._at = None

    def route_to(self, t: int) -> Route | None:
        """The way to the middle of triangle ``t`` (its last step crosses into
        ``t``), or None if there is none that could be inserted."""
        return self._back(int(self._node[t]), float(self.best[t]))

    def route_to_pad(self, pad: int) -> Route | None:
        """The way to a pad of the wire's net that the search reached."""
        return self._back(self.pads[pad][1], self.pads[pad][0])

    def _back(self, node: int, cost: float) -> Route | None:
        if node < 0:
            return None
        if self._at is None:  # where each node the search touched stands in its record
            self._at = np.zeros(int(self._touched.max()) + 1, dtype=np.int64)
            self._at[self._touched] = np.arange(len(self._touched))
        at, parent, parent_tr = self._at, self._parent, self._parent_tr
        return _read(self._pmap, self._state, node, lambda n: (int(parent[at[n]]), int(parent_tr[at[n]])), cost, self._relaxed)


FAR_CELLS = 64  # along the board's longer side: the coarse map of how far a net's copper is


def _far(pmap: PlanarMap, edges: list[int]):
    """A coarse map of the board: how far each cell is, at the least, from the
    middle of the nearest of ``edges`` (``kernel.estimate``). Returns (map,
    x0, y0, cell)."""
    extent = pmap.__dict__.get("_extent")
    if extent is None:
        on_board = pmap.vx < 0.5 * FAR  # not the parked slots of via sites
        x0, x1, y0, y1 = pmap.vx[on_board].min(), pmap.vx[on_board].max(), pmap.vy[on_board].min(), pmap.vy[on_board].max()
        cell = max(x1 - x0, y1 - y0, 1e-6) / FAR_CELLS
        extent = pmap.__dict__["_extent"] = (float(x0), float(y0), float(cell), int((x1 - x0) / cell) + 1, int((y1 - y0) / cell) + 1)
    x0, y0, cell, nx, ny = extent
    clear = np.ones((nx, ny), dtype=bool)
    mids = pmap.edge_mid[edges]
    clear[np.clip(((mids[:, 0] - x0) / cell).astype(np.int64), 0, nx - 1), np.clip(((mids[:, 1] - y0) / cell).astype(np.int64), 0, ny - 1)] = False
    # Between two points in cells whose middles are d apart there is at least d less a cell's diagonal.
    return np.maximum(ndimage.distance_transform_edt(clear) * cell - math.sqrt(2.0) * cell, 0.0), x0, y0, cell


_NOWHERE = (np.full((1, 1), np.inf), 0.0, 0.0, 1.0)
_NO_RIDE = (np.zeros(0, dtype=np.int64), np.zeros(0, dtype=np.int64))


def _own(pmap: PlanarMap, state: TopoState, net: int, through: bool):
    """What a search for a wire of ``net`` reads about its net: the places
    beside the net's wires (gates, and the places on each), the net's pads on this layer (if the wire may go
    ``through`` them) and how far its wires are from everywhere. Kept while
    neither the wires nor the map change: a connection is searched several
    times over before anything moves."""
    key = (net, through, state.epoch, pmap.changes)
    kept = pmap.__dict__.get("_own")
    if kept is None or kept[0] != key:
        riding = state.beside(net)
        by_net = pmap.__dict__.get("_net_pads")
        if by_net is None or by_net[0] != pmap.changes:  # the pads of every net: they change only with the map (vias)
            by_net = pmap.__dict__["_net_pads"] = (pmap.changes, {})
            for p in sorted(pmap.pad_edges):
                by_net[1].setdefault(pmap.pad_net.get(p, -1), []).append(p)
        pads = by_net[1].get(net, []) if through else []
        at = np.fromiter(riding, dtype=np.int64, count=len(riding))
        kept = pmap.__dict__["_own"] = (key, (at, np.fromiter(riding.values(), dtype=np.int64, count=len(riding))), pads,
                                        _far(pmap, at) if riding else _NOWHERE)
    return kept[1:]


def route(pmap: PlanarMap, state: TopoState, src_pad: int | None, dst_pad: int, params: CostParams | None = None,
          mode: str = "normal", corridor: set[int] | None = None, penalty: dict[int, float] | None = None,
          congestion: bool = True, hard_cap: bool = False, weight: float = 1.0, seeds=None, reach: bool = False,
          target: tuple[float, float] | None = None, bound: float = math.inf, net: int = -1,
          through: bool = False, starts: dict[int, float] | None = None):
    """Cheapest planar path to ``dst_pad`` on this layer, or None.

    It starts from ``src_pad``, from the pads ``starts`` (pad -> cost so far)
    and from ``seeds`` = (triangles, costs, x, y): points in the middle cells
    of those triangles, each at its cost so far.

    mode: "normal"   - only crossing-free slots are explored;
          "relaxed"  - crossings allowed at ``cross_penalty`` each, reported in
                       ``Route.blocking`` (the result cannot be inserted);
          "corridor" - like normal, restricted to the edge set ``corridor``.
    ``penalty`` adds a per-gate cost (candidate diversification). ``hard_cap``
    forbids exceeding capacity instead of pricing it. ``weight`` is the load
    the wire puts on a gate (1 for a base-width trace). ``net``: the wire's
    net; beside a wire of the same net it is the same trace and costs no room
    and next to no length (11). ``through``: it may also run into a pad of its
    net that is on its way and go on from there; the route then has a
    ``lead``, the pieces before the last.

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
    start = {} if src_pad is None else {src_pad: 0.0}
    start.update(starts or {})
    from_edges = [e for pad in start for e in pmap.pad_edges.get(pad, ())]
    from_costs = [start[pad] for pad in start for _ in pmap.pad_edges.get(pad, ())]
    tris, costs, xs, ys = seeds if seeds is not None else _NO_SEEDS
    here = dst_pad in pmap.pad_edges
    if not (here or (reach and target is not None)) or src_pad == dst_pad or not (len(from_edges) or len(tris)):
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
    pmap.catch_up()
    found, seen = None, None
    tb = None
    (ride_at, ride_places), pads, far = _own(pmap, state, net, through) if net >= 0 else (_NO_RIDE, [], _NOWHERE)
    pads = [p for p in pads if p != dst_pad and p not in start]
    own_edges = [pmap.pad_edges[p] for p in pads]
    own_ptr = np.cumsum([0, *map(len, own_edges)]).astype(np.int64)
    own_edge = np.array([e for edges in own_edges for e in edges], dtype=np.int64)
    own_cost, own_node = np.full(len(pads), np.inf), np.full(len(pads), -1, dtype=np.int64)
    try:
        while True:
            tb = pmap.__dict__.get("_kernel_tables")
            if tb is None:
                tb = pmap.__dict__["_kernel_tables"] = kernel.Tables(pmap)
            tb.ride[ride_at] = ride_places
            for k, edges in enumerate(own_edges):
                tb.own[edges] = k + 1
            best[:], best_node[:] = np.inf, -1
            own_cost[:], own_node[:] = np.inf, -1
            try:
                goal, cost, touched = search(
                    np.array(from_edges, dtype=np.int32), np.array(from_costs, dtype=np.float64),
                    np.ascontiguousarray(tris, dtype=np.int64), np.ascontiguousarray(costs, dtype=np.float64),
                    np.ascontiguousarray(xs, dtype=np.float64), np.ascontiguousarray(ys, dtype=np.float64), dst_pad,
                    pmap.tri_e, pmap.tri_v, pmap.edge_v, pmap.edge_t,
                    tb.tr, tb.length, tb.n, pmap.edge_kind, pmap.edge_owner, pmap.edge_mid,
                    state.count, state.corner, state.load, state.cap, state.hist, pen, pen is not no_pen, cor, mode == "corridor",
                    relaxed, params.pres_fac if congestion else 0.0, congestion, params.cross_penalty, hard_cap, weight, tb.ride,
                    tb.own, own_ptr, own_edge, own_cost, own_node,
                    tx, ty, radius, params.h_weight, *far, bound, tb.g, tb.parent, tb.parent_tr, tb.touched,
                    tb.heap_f, tb.heap_g, tb.heap_n, best, best_node)
            except Exception as error:  # the compiled kernel itself failed: never crash, fall back for good
                if search is kernel.astar_plain:
                    raise
                from weaveengine import accel
                accel.failed("search", error)
                pmap.__dict__.pop("_kernel_tables", None)  # its workspace may be half-used
                search = kernel.astar_plain
                continue
            idx = tb.touched[:touched]
            if goal >= 0:
                chain = np.empty(8192, dtype=np.int64)
                n, root = (kernel.walk_back if search is kernel.astar else kernel.walk_back_plain)(tb.parent, int(goal), chain)
                chain = chain[:n]
                back = dict(zip(chain.tolist(), zip(chain[1:].tolist() + [int(root)], tb.parent_tr[chain].tolist())))
                found = _read(pmap, state, int(goal), back.__getitem__, float(cost), relaxed) if n < 8192 else None
            if reach:
                seen = Reach(pmap, state, relaxed, best, best_node, idx.copy(), tb.parent[idx], tb.parent_tr[idx],
                             {p: (float(c), int(n)) for p, c, n in zip(pads, own_cost, own_node) if n >= 0})
            tb.g[idx] = np.inf
            tb.parent[idx] = -1
            if goal != -2:
                break
            size = 4 * len(tb.heap_f)  # the workspace was too small (many seeds): a larger one, and again
            tb.heap_f, tb.heap_g, tb.heap_n = np.zeros(size), np.zeros(size), np.zeros(size, dtype=np.int32)
            tb.g[:] = np.inf
            tb.parent[:] = -1
    finally:
        if tb is not None and pmap.__dict__.get("_kernel_tables") is tb:
            tb.ride[ride_at] = 0
            tb.own[own_edge] = 0
    return (found, seen) if reach else found


def _read(pmap: PlanarMap, state: TopoState, node: int, parent_of, cost: float, relaxed: bool) -> Route | None:
    """The route that ends at ``node``. ``parent_of(node)`` gives (parent,
    how it led here: which transition, or ``kernel.THROUGH``); a parent below
    zero ends the walk: -1 at a pad edge, -2 - i at seed i. A route that came
    through pads of its net is returned as its last piece, with the pieces
    before it as its ``lead``.

    None if a piece crosses a gate twice. Such a route runs along one side of a
    wire, round its end and back along the other: it cannot be inserted (7.3),
    and what it says is that the wire is in the way. No other route is looked
    for: the next cheapest go round by the same wire, further out (8.4)."""
    tables = pmap.__dict__["_kernel_tables"]
    mids = pmap.edge_mid_list
    pieces: list[Route] = []
    rev: list[Step] = []
    blocking: set[int] = set()
    while True:
        prev, j = parent_of(node)
        if prev < 0 or j == kernel.THROUGH:  # where this piece starts: on the edge of a pad
            rev.append(((node >> 4) >> 1, -1, -1, node & 15))
            gates = [s[0] for s in rev]
            if not relaxed and len(set(gates)) != len(gates):
                return None
            pieces.append(Route(rev[::-1], 0.0, sum(math.dist(mids[a], mids[b]) for a, b in zip(gates, gates[1:])),
                                pad=pmap.edge_owner_list[gates[0]]))
            rev = []
            if prev < 0:
                break
        else:
            he = prev >> 4
            t, k = int(tables.tr[he, j, 2]), int(tables.tr[he, j, 3])
            rev.append(((node >> 4) >> 1, t, k, node & 15))
            if relaxed:
                e, p = he >> 1, prev & 15
                wires = state.gate_order[e]
                if pmap.tri_v_list[t][k] != pmap.edge_v_list[e][0]:
                    wires = wires[::-1]
                    p = len(wires) - p
                blocking.update(wires[state.corner_cnt[t][k]:p])
        node = prev
    last = pieces[0]
    last.cost, last.blocking, last.seed, last.lead = cost, blocking, -1 if prev == -1 else -2 - prev, pieces[:0:-1]
    return last
