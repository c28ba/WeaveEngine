"""Routes that may change layer (design section 12.3).

A route is a chain of searches, one per layer it runs on, joined by vias. The
first search starts at the connection's pad; each later one starts from every
point the search before could reach, at its cost so far plus the cost of a via.
A via is priced like anything else, so whether a connection takes one, and
where, falls out of the same cost that decides everything else about it.

Where the search before reached copper its net already has on both layers, a
through-hole pad or a via, the next one starts from that as well, at no cost:
the layer change is there already (11).
"""
import math
from dataclasses import dataclass, field

import numpy as np
import shapely

from weaveengine.topo import kernel, sites
from weaveengine.topo.planar_map import PlanarMap
from weaveengine.topo.search import Route, route

GRID = 0.25  # mm: cell of the coarse map of where a via may be


@dataclass
class Path:
    """A route from pad to pad, in pieces. Between consecutive pieces is a
    joint: a point (x, y) where a via is to be made, or the id of a pad or via
    of the connection's net that is there already, which the route runs
    through or changes layer at."""
    routes: list[Route]
    joints: list = field(default_factory=list)
    cost: float = 0.0

    @classmethod
    def on_one_layer(cls, r: Route) -> "Path":
        """What one search found: its pieces, if it came through pads of its net."""
        pieces = [*r.lead, r]
        for piece in pieces:
            piece.layer = r.layer
        return cls(pieces, [piece.pad for piece in r.lead], r.cost)

    @property
    def vias(self) -> list[tuple[float, float]]:
        """The vias to be made."""
        return [j for j in self.joints if not isinstance(j, int)]

    @property
    def blocking(self) -> set[int]:
        return set().union(*(r.blocking for r in self.routes))

    @property
    def length(self) -> float:
        return sum(r.length for r in self.routes)


def locate(pmap: PlanarMap, pts: np.ndarray) -> np.ndarray:
    """Triangle of the map containing each point (-1 = none)."""
    out = np.zeros(len(pts), dtype=np.int64)
    grid = pmap.__dict__.get("_locate_grid")
    if grid is None:
        # The triangles sorted into cells, about two cells to a triangle. The
        # board's extent, not the tables': unused slots are parked far away.
        x0, y0, x1, y1 = pmap.free_space.free.bounds
        cell = max(x1 - x0, y1 - y0, 1e-6) / max(16.0, math.sqrt(2.0 * pmap.num_triangles))
        nx, ny = int((x1 - x0) / cell) + 1, int((y1 - y0) / cell) + 1
        grid = pmap.__dict__["_locate_grid"] = [x0, y0, cell, nx, ny, np.zeros(nx * ny + 1, dtype=np.int64), np.zeros(0, dtype=np.int32)]
        pmap.moved = True
    if pmap.moved:
        need = kernel.cells(pmap.tri_v, pmap.vx, pmap.vy, *grid)
        if need > len(grid[6]):
            grid[6] = np.zeros(2 * need, dtype=np.int32)
            kernel.cells(pmap.tri_v, pmap.vx, pmap.vy, *grid)
        pmap.moved = False
    kernel.locate(pmap.tri_v, pmap.vx, pmap.vy, np.ascontiguousarray(pts[:, 0]), np.ascontiguousarray(pts[:, 1]), out, *grid)
    return out


class Legal:
    """Where a via's centre may be: inside every layer's free space, shrunk so
    that the via copper keeps its clearance from fixed copper and the board
    edge. The exact regions, and a coarse grid of them for testing many points
    at once. Other vias and the traces are checked separately."""

    def __init__(self, ctx):
        rules = ctx.board.rules
        margin = rules.via_diameter / 2.0 / math.cos(math.pi / 16.0) - rules.base_width / 2.0 + 0.01
        self.regions = []
        for layer in ctx.layers:
            region = layer.pmap.free_space.free.buffer(-margin)
            shapely.prepare(region)
            self.regions.append(region)
        x0, y0, x1, y1 = ctx.board.outline.bounds
        self.x0, self.y0 = x0, y0
        gx, gy = np.meshgrid(x0 + (np.arange(int((x1 - x0) / GRID) + 1) + 0.5) * GRID,
                             y0 + (np.arange(int((y1 - y0) / GRID) + 1) + 0.5) * GRID, indexing="ij")
        self.grid = np.ones(gx.shape, dtype=bool)
        for region in self.regions:
            self.grid &= shapely.contains_xy(region, gx, gy)

    def exactly(self, x: float, y: float) -> bool:
        return all(shapely.contains_xy(region, x, y) for region in self.regions)


def _taken(ctx, struck) -> np.ndarray:
    """The points a new via must keep its distance from: the vias there are,
    and ``struck``. In order of x."""
    pts = np.array([s.centre for s in ctx.layers[0].pmap.sites.values() if s.active]
                   + [p.centre for p in ctx.board.pads if p.is_via] + list(struck), dtype=float).reshape(-1, 2)
    return pts[np.argsort(pts[:, 0], kind="stable")]


def _via_points(ctx, la: int, seen, goal, bound: float, taken, via_cost: float, onward: float):
    """Where a route that the search ``seen`` brought this far on layer ``la``
    may change layer: the points it reached where a via is legal and fits,
    each at its cost so far plus a via. Points that cannot lead to a route
    cheaper than ``bound`` are left out. Returns (points, costs, where each
    came from as (layer, triangle)), or None. ``onward``: the part of the
    straight line from a point to the goal that the way on costs at least."""
    rules, legal, pmap = ctx.board.rules, ctx.legal, ctx.layers[la].pmap
    x, y, cost, tris = kernel.via_points(seen.best, via_cost, bound, goal[0], goal[1], onward, pmap.tri_v, pmap.vx, pmap.vy,
                                         ctx.layers[la].state.corner, pmap.pitch, sites.keep_off(rules), sites.MIN_ROOM * sites.SITE_RADIUS,
                                         legal.grid, legal.x0, legal.y0, GRID, np.ascontiguousarray(taken[:, 0]),
                                         np.ascontiguousarray(taken[:, 1]), rules.via_diameter + rules.clearance + 0.02)
    return (np.stack([x, y], axis=1), cost, np.stack([np.full(len(tris), la), tris], axis=1)) if len(x) else None


def _seeds(ctx, sources: dict, onto, goal, bound: float, onward: float):
    """Where the next search, on layer ``onto``, may start: the via points of
    the other layers (``sources``, per layer) that have room on this one too
    and can still lead to a route cheaper than ``bound``. Returns (triangles
    on ``onto``, costs, points, where each came from)."""
    parts = [found for la, found in sources.items() if la != onto.index and found is not None]
    if not parts:
        return None
    pts, cost, source = (np.concatenate(x) for x in zip(*parts))
    pmap = onto.pmap
    tris = locate(pmap, pts)
    at = np.nonzero((tris >= 0) & (cost + onward * np.hypot(pts[:, 0] - goal[0], pts[:, 1] - goal[1]) < bound))[0]
    at = at[kernel.rooms(pmap.tri_v, pmap.vx, pmap.vy, onto.state.corner, pmap.pitch, sites.keep_off(ctx.board.rules),
                         sites.MIN_ROOM * sites.SITE_RADIUS, tris[at], np.ascontiguousarray(pts[at, 0]), np.ascontiguousarray(pts[at, 1]))]
    return (tris[at], cost[at], pts[at], source[at]) if len(at) else None


def place(ctx, conn, path: "Path | None", **how) -> bool:
    """Commits ``path``. If it does not fit as planned, plans again (``how``:
    as the first plan was made), twice at most: without the point where a via
    did not fit, or, if it has no via, without going through pads of its net."""
    struck: list = []
    for _ in range(3):
        if path is None:
            return False
        if ctx.commit(conn, path):
            return True
        struck += path.vias
        if not path.vias:
            how["through"] = False
        path = find(ctx, conn, struck=struck, **how)
    return False


def find(ctx, conn, max_vias: int | None = None, struck=(), penalty: dict | None = None, first: dict | None = None,
         via_cost: float | None = None, bound: float = math.inf, through: bool = True, **how) -> Path | None:
    """Cheapest route for the connection, with at most ``max_vias`` new vias
    (default: as many as the parameters allow). It may run through pads and
    vias of its net and change layer at them (unless ``through`` is False:
    then it touches nothing but its own two pads). ``how`` goes to every
    search (``mode``, ``hard_cap``, ``congestion``); ``penalty`` is per layer.
    ``struck``: points where a via has been found not to fit. ``first``: per
    layer, the outcome of the search from the connection's own pad, where the
    caller has made it already. ``via_cost``: what a via costs, if not what
    the parameters say. ``bound``: only a route cheaper than this is of
    interest (the searches stop short of anything else)."""
    params = ctx.params
    via_cost = params.via_cost if via_cost is None else via_cost
    if max_vias is None:
        max_vias = params.max_vias if ctx.vias else 0
    changes = params.max_vias if len(ctx.layers) > 1 else 0  # of layer: at a new via, or at copper the net has on both
    goal = ctx.pad_centre(conn.dst)
    penalty = penalty or {}
    search = dict(weight=conn.weight, net=conn.net_id, reach=True, target=goal, through=through, **how)
    # What the way on from a via costs at least, of the straight line: all of it if the net has
    # no copper but this connection's; else next to nothing, since it may lead along that copper.
    onward = 1.0 if conn.net_id in ctx.pairs else kernel.RIDE
    best = None  # (cost, hop, layer, route)
    reached = {}  # per layer: what the search saw, the via points it started from, the layer each pad it started from was reached on
    for layer in ctx.layers:
        if conn.src not in layer.pmap.pad_edges:
            continue
        if first and layer.index in first and through:
            r, seen = first[layer.index]
        else:
            r, seen = route(layer.pmap, layer.state, conn.src, conn.dst, params, penalty=penalty.get(layer.index), bound=bound, **search)
        reached[layer.index] = (seen if changes else None, None, {})
        if r is not None and (best is None or r.cost < best[0]):
            best = (r.cost, 0, layer.index, r)
    hops = [reached]
    taken = _taken(ctx, struck) if max_vias else None
    had: dict[tuple[int, int], float] = {}  # (layer, pad of the net) -> the least cost at which a search has been at it there
    for hop in range(1, changes + 1):
        limit = best[0] if best else bound
        sources = {la: _via_points(ctx, la, seen, goal, limit, taken, via_cost, onward)
                   for la, (seen, _, _) in reached.items() if seen is not None} if hop <= max_vias else {}
        pads: dict[int, tuple[float, int]] = {}  # copper of the net that was reached: pad -> (cost, on which layer)
        for la, (seen, _, _) in reached.items():
            for pad, (cost, _) in (seen.pads.items() if seen is not None else ()):
                had[la, pad] = min(cost, had.get((la, pad), math.inf))
                if cost < limit and cost < pads.get(pad, (math.inf, 0))[0]:
                    pads[pad] = (cost, la)
        reached = {}
        for layer in ctx.layers:
            seeds = _seeds(ctx, sources, layer, goal, limit, onward)
            # A pad the net has on this layer too is where the route may go on here, unless a search has been at it here for less.
            starts = {pad: (cost, la) for pad, (cost, la) in pads.items()
                      if la != layer.index and pad in layer.pmap.pad_edges and cost < had.get((layer.index, pad), math.inf)}
            had.update({(layer.index, pad): cost for pad, (cost, _) in starts.items()})
            if seeds is None and not starts:
                continue
            tris, cost, pts, source = seeds if seeds is not None else (None, None, None, None)
            r, seen = route(layer.pmap, layer.state, None, conn.dst, params, starts={pad: c for pad, (c, _) in starts.items()},
                            seeds=None if seeds is None else (tris, cost, pts[:, 0], pts[:, 1]),
                            bound=limit, penalty=penalty.get(layer.index), **search)
            reached[layer.index] = (seen if hop < changes else None, (pts, source), {pad: la for pad, (_, la) in starts.items()})
            if r is not None and (best is None or r.cost < best[0]):
                best = (r.cost, hop, layer.index, r)
        if not reached:
            break
        hops.append(reached)
    if best is None:
        return None
    cost, hop, li, r = best
    chain, joints = [], []  # from the goal back to the connection's pad
    for level in range(hop, -1, -1):
        r.layer = li
        chain.append(Path.on_one_layer(r))
        if level == 0:
            break
        if r.seed >= 0:  # it starts at a new via
            pts, source = hops[level][li][1]
            joints.append((float(pts[r.seed, 0]), float(pts[r.seed, 1])))
            li, tri = int(source[r.seed, 0]), int(source[r.seed, 1])
            r = hops[level - 1][li][0].route_to(tri)
        else:            # it starts at copper of its net that the search before came to on another layer
            pad = ctx.layers[li].pmap.edge_owner_list[chain[-1].routes[0].steps[0][0]]
            joints.append(pad)
            li = hops[level][li][2][pad]
            r = hops[level - 1][li][0].route_to_pad(pad)
        if r is None:
            return None  # that way crosses a gate twice: it cannot be inserted
    routes, between = [], []
    for part, joint in zip(chain[::-1], [*joints[::-1], None]):
        routes += part.routes
        between += part.joints + ([joint] if joint is not None else [])
    return Path(routes, between, cost)

