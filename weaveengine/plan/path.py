"""Routes that may change layer (design section 12.3).

A route is a chain of searches, one per layer it runs on, joined by vias. The
first search starts at the connection's pad; each later one starts from every
point the search before could reach, at its cost so far plus the cost of a via.
A via is priced like anything else, so whether a connection takes one, and
where, falls out of the same cost that decides everything else about it.
"""
import math
from dataclasses import dataclass, field

import numpy as np
import shapely
from scipy.spatial import cKDTree

from weaveengine.topo import kernel, sites
from weaveengine.topo.planar_map import PlanarMap
from weaveengine.topo.search import Route, route

GRID = 0.25  # mm: cell of the coarse map of where a via may be


@dataclass
class Path:
    """A route from pad to pad: one piece per layer it runs on, in order, with a via between consecutive pieces."""
    routes: list[Route]
    vias: list[tuple[float, float]] = field(default_factory=list)
    cost: float = 0.0

    @classmethod
    def on_one_layer(cls, r: Route) -> "Path":
        return cls([r], [], r.cost)

    @property
    def blocking(self) -> set[int]:
        return set().union(*(r.blocking for r in self.routes))

    @property
    def length(self) -> float:
        return sum(r.length for r in self.routes)


def locate(pmap: PlanarMap, pts: np.ndarray) -> np.ndarray:
    """Triangle of the map containing each point (-1 = none)."""
    out = np.zeros(len(pts), dtype=np.int64)
    hint = pmap.__dict__.get("_locate_hint")
    if hint is None:
        # The board's extent, not the tables': unused slots are parked far away.
        x0, y0, x1, y1 = pmap.free_space.free.bounds
        cell = max(x1 - x0, y1 - y0, 1e-6) / 256.0
        hint = pmap.__dict__["_locate_hint"] = (np.zeros((int((x1 - x0) / cell) + 1, int((y1 - y0) / cell) + 1), dtype=np.int64), cell, x0, y0)
    kernel.locate(pmap.tri_v, pmap.tri_n, pmap.vx, pmap.vy, np.ascontiguousarray(pts[:, 0]), np.ascontiguousarray(pts[:, 1]), out,
                  hint[0], hint[2], hint[3], hint[1])
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

    def roughly(self, pts: np.ndarray) -> np.ndarray:
        ix = np.clip(((pts[:, 0] - self.x0) / GRID).astype(np.int64), 0, self.grid.shape[0] - 1)
        iy = np.clip(((pts[:, 1] - self.y0) / GRID).astype(np.int64), 0, self.grid.shape[1] - 1)
        return self.grid[ix, iy]

    def exactly(self, x: float, y: float) -> bool:
        return all(shapely.contains_xy(region, x, y) for region in self.regions)


def _points(pmap: PlanarMap, tris: np.ndarray) -> np.ndarray:
    """Points inside each triangle where a site might go: the incentre, the
    centroid, and one towards each corner. [len(tris), 5, 2]"""
    xy = np.stack([pmap.vx[pmap.tri_v[tris]], pmap.vy[pmap.tri_v[tris]]], axis=2)   # [n, 3, 2]
    cen = xy.mean(axis=1)
    side = np.linalg.norm(xy[:, [1, 2, 0]] - xy[:, [2, 0, 1]], axis=2)             # length opposite each corner
    inc = (xy * side[:, :, None]).sum(axis=1) / side.sum(axis=1)[:, None]
    return np.concatenate([inc[:, None], cen[:, None], (xy + inc[:, None]) / 2.0], axis=1)


def _has_room(layer, tris: np.ndarray, pts: np.ndarray, keep: float) -> np.ndarray:
    """Whether each point really lies in the middle cell of its triangle: clear
    of the triangle's edges, and far enough from each corner for the via's
    keep-off and the wires that cut that corner."""
    pmap, state = layer.pmap, layer.state
    xy = np.stack([pmap.vx[pmap.tri_v[tris]], pmap.vy[pmap.tri_v[tris]]], axis=2)       # [n, 3, 2]
    d = np.hypot(xy[:, :, 0] - pts[:, None, 0], xy[:, :, 1] - pts[:, None, 1])
    ok = (d - keep >= (state.corner[tris] - 1) * pmap.pitch).all(axis=1)
    a, b = xy, xy[:, [1, 2, 0]]
    cross = (b[:, :, 0] - a[:, :, 0]) * (pts[:, None, 1] - a[:, :, 1]) - (b[:, :, 1] - a[:, :, 1]) * (pts[:, None, 0] - a[:, :, 0])
    height = cross / np.maximum(np.hypot(b[:, :, 0] - a[:, :, 0], b[:, :, 1] - a[:, :, 1]), 1e-12)
    return ok & (height.min(axis=1) >= sites.MIN_ROOM * sites.SITE_RADIUS)


def _seeds(ctx, reached: dict, onto, goal, bound: float, struck):
    """Where the next search, on layer ``onto``, may start: the points the
    searches before it reached on other layers where a via is legal and fits,
    each at its cost so far plus a via. Points that cannot lead to a route
    cheaper than ``bound`` are left out. Returns (triangles on ``onto``, costs,
    points, where each came from as (layer, triangle))."""
    params, rules = ctx.params, ctx.board.rules
    keep = sites.keep_off(rules)
    spacing = rules.via_diameter + rules.clearance + 0.02
    taken = np.array([s.centre for s in ctx.layers[0].pmap.sites.values() if s.active]
                     + [p.centre for p in ctx.board.pads if p.is_via] + list(struck)).reshape(-1, 2)
    pts, cost, source = [], [], []
    for la, (seen, _) in reached.items():
        if la == onto.index or seen is None:
            continue
        tris = np.nonzero(np.isfinite(seen.best))[0]
        if not len(tris):
            continue
        p = _points(ctx.layers[la].pmap, tris).reshape(-1, 2)
        c = np.repeat(seen.best[tris], 5) + params.via_cost
        ok = ctx.legal.roughly(p) & (c + np.hypot(p[:, 0] - goal[0], p[:, 1] - goal[1]) < bound)
        if len(taken):
            ok &= cKDTree(taken).query(p, distance_upper_bound=spacing)[0] > spacing
        at = np.nonzero(ok)[0]
        at = at[_has_room(ctx.layers[la], np.repeat(tris, 5)[at], p[at], keep)]
        pts.append(p[at])
        cost.append(c[at])
        source.append(np.stack([np.full(len(at), la), np.repeat(tris, 5)[at]], axis=1))
    if not pts or not sum(len(p) for p in pts):
        return None
    pts, cost, source = np.concatenate(pts), np.concatenate(cost), np.concatenate(source)
    tris = locate(onto.pmap, pts)
    at = np.nonzero(tris >= 0)[0]
    at = at[_has_room(onto, tris[at], pts[at], keep)]
    return (tris[at], cost[at], pts[at], source[at]) if len(at) else None


def place(ctx, conn, path: "Path | None", **how) -> bool:
    """Commits ``path``. If one of its vias does not fit where it was planned,
    plans again without that point (``how``: as the first plan was made), twice at most."""
    struck: list = []
    for _ in range(3):
        if path is None:
            return False
        if ctx.commit(conn, path):
            return True
        struck += path.vias
        path = find(ctx, conn, struck=struck, **how)
    return False


def find(ctx, conn, max_vias: int | None = None, struck=(), penalty: dict | None = None, **how) -> Path | None:
    """Cheapest route for the connection, changing layer at most ``max_vias``
    times (default: as many as the parameters allow). ``how`` goes to every
    search (``mode``, ``hard_cap``, ``congestion``); ``penalty`` is per layer.
    ``struck``: points where a via has been found not to fit."""
    params = ctx.params
    if max_vias is None:
        max_vias = params.max_vias if ctx.vias else 0
    goal = ctx.pad_centre(conn.dst)
    penalty = penalty or {}
    best = None  # (cost, hop, layer, route)
    reached = {}
    for layer in ctx.layers:
        if conn.src not in layer.pmap.pad_edges:
            continue
        r, seen = route(layer.pmap, layer.state, conn.src, conn.dst, params, weight=conn.weight, net=conn.net_id, reach=True,
                        target=goal, penalty=penalty.get(layer.index), **how)
        reached[layer.index] = (seen if max_vias else None, None)
        if r is not None and (best is None or r.cost < best[0]):
            best = (r.cost, 0, layer.index, r)
    hops = [reached]
    for hop in range(1, max_vias + 1):
        reached = {}
        for layer in ctx.layers:
            seeds = _seeds(ctx, hops[-1], layer, goal, best[0] if best else math.inf, struck)
            if seeds is None:
                continue
            tris, cost, pts, source = seeds
            r, seen = route(layer.pmap, layer.state, None, conn.dst, params, weight=conn.weight, net=conn.net_id, reach=True, target=goal,
                            seeds=(tris, cost, pts[:, 0], pts[:, 1]), bound=best[0] if best else math.inf,
                            penalty=penalty.get(layer.index), **how)
            reached[layer.index] = (seen if hop < max_vias else None, (pts, source))
            if r is not None and (best is None or r.cost < best[0]):
                best = (r.cost, hop, layer.index, r)
        if not reached:
            break
        hops.append(reached)
    if best is None:
        return None
    cost, hop, li, r = best
    r.layer = li
    routes, vias = [r], []
    for level in range(hop, 0, -1):
        pts, source = hops[level][li][1]
        vias.append((float(pts[r.seed, 0]), float(pts[r.seed, 1])))
        li, tri = int(source[r.seed, 0]), int(source[r.seed, 1])
        r = hops[level - 1][li][0].route_to(tri)
        if r is None:
            return None  # that way crosses a gate twice: it cannot be inserted
        r.layer = li
        routes.append(r)
    return Path(routes[::-1], vias[::-1], cost)
