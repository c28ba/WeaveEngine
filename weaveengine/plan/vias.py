"""Via sites (design section 12, option 2, placed on demand).

A via is a small through pad of the net. When a connection cannot be completed
on any single layer, a site is chosen that one end can reach on one layer and
the other end on another; the connection is split there and the board is
routed again with the via as an ordinary pad. Only sites that are wanted
exist, so no unused site wastes routing space.
"""
import math

import numpy as np
import shapely
from scipy.spatial import cKDTree

from weaveengine.board import Board, Pad
from weaveengine.plan.context import Connection, Context
from weaveengine.topo import sites
from weaveengine.topo.planar_map import PlanarMap
from weaveengine.topo.search import flood

MAX_POINTS = 4000


def locate(pmap: PlanarMap, pts: np.ndarray) -> np.ndarray:
    """Index of the triangle containing each point (-1 if none)."""
    out = np.full(len(pts), -1, dtype=np.int64)
    a = np.stack([pmap.vx[pmap.tri_v[:, 0]], pmap.vy[pmap.tri_v[:, 0]]], axis=1)
    b = np.stack([pmap.vx[pmap.tri_v[:, 1]], pmap.vy[pmap.tri_v[:, 1]]], axis=1)
    c = np.stack([pmap.vx[pmap.tri_v[:, 2]], pmap.vy[pmap.tri_v[:, 2]]], axis=1)
    for start in range(0, len(pts), 512):
        p = pts[start:start + 512, None, :]
        d1 = (b[:, 0] - a[:, 0]) * (p[:, :, 1] - a[:, 1]) - (b[:, 1] - a[:, 1]) * (p[:, :, 0] - a[:, 0])
        d2 = (c[:, 0] - b[:, 0]) * (p[:, :, 1] - b[:, 1]) - (c[:, 1] - b[:, 1]) * (p[:, :, 0] - b[:, 0])
        d3 = (a[:, 0] - c[:, 0]) * (p[:, :, 1] - c[:, 1]) - (a[:, 1] - c[:, 1]) * (p[:, :, 0] - c[:, 0])
        inside = (d1 >= -1e-12) & (d2 >= -1e-12) & (d3 >= -1e-12)
        hit = inside.any(axis=1)
        out[start:start + 512][hit] = inside.argmax(axis=1)[hit]
    return out


def propose(ctx: Context, conn: Connection, taken: list[tuple[float, float]]) -> tuple[float, float] | None:
    """Best via position for an unrouted connection on the current state, or None."""
    rules = ctx.board.rules
    radius = rules.via_diameter / 2.0
    # The via's own keep-off ring should not merge with its neighbours'; fall
    # back to bare legality (copper clearance only) where space is tight.
    margins = [radius + rules.clearance + rules.base_width / 2.0 + 0.01, radius - rules.base_width / 2.0 + 0.01]
    src_layers = [l.index for l in ctx.layers if conn.src in l.pmap.pad_edges]
    dst_layers = [l.index for l in ctx.layers if conn.dst in l.pmap.pad_edges]
    reach = {}

    def reach_of(pad: int, li: int) -> dict[int, float]:
        if (pad, li) not in reach:
            layer = ctx.layers[li]
            found = flood(layer.pmap, layer.state, pad, conn.weight)[0]
            reach[(pad, li)] = {int(t): float(found[t]) for t in np.nonzero(np.isfinite(found))[0]}
        return reach[(pad, li)]

    spacing = 2 * radius + rules.clearance + rules.pitch
    for margin in margins:
        safe = [layer.pmap.free_space.free.buffer(-margin) for layer in ctx.layers]
        best = None
        for la in src_layers:
            for lb in dst_layers:
                if la == lb:
                    continue
                ra, rb = reach_of(conn.src, la), reach_of(conn.dst, lb)
                if not ra or not rb:
                    continue
                pa = ctx.layers[la].pmap
                tris = np.fromiter(ra, dtype=np.int64)
                cost_a = np.array([ra[t] for t in tris.tolist()])
                # Candidate points: centroid and edge midpoints of every reachable triangle.
                cen = np.array([pa.tri_cen[t] for t in tris.tolist()])
                mids = pa.edge_mid[pa.tri_e[tris]].reshape(-1, 2)
                pts = np.concatenate([cen, mids])
                cost = np.concatenate([cost_a, np.repeat(cost_a, 3)])  # midpoints follow tri_e row order
                ok = np.ones(len(pts), dtype=bool)
                for s in safe:
                    ok &= shapely.contains_xy(s, pts[:, 0], pts[:, 1])
                for x, y in taken:
                    ok &= np.hypot(pts[:, 0] - x, pts[:, 1] - y) >= spacing
                pts, cost = pts[ok], cost[ok]
                if not len(pts):
                    continue
                keep = np.argsort(cost)[:MAX_POINTS]
                pts, cost = pts[keep], cost[keep]
                tb = locate(ctx.layers[lb].pmap, pts)
                cost_b = np.array([rb.get(t, math.inf) for t in tb.tolist()])
                total = cost + cost_b
                i = int(np.argmin(total))
                if math.isfinite(total[i]) and (best is None or total[i] < best[0]):
                    best = (float(total[i]), (float(pts[i, 0]), float(pts[i, 1])))
        if best is not None:
            return best[1]
        # Both pads live on the same single layer (surface-mount parts): the
        # connection needs two vias. Place the first one where this end can
        # reach, as close to the other end as possible; the remaining
        # via-to-pad connection gets its own via in the next round.
        if set(src_layers) == set(dst_layers) and len(src_layers) == 1 and len(ctx.layers) > 1:
            la = src_layers[0]
            pa = ctx.layers[la].pmap
            target = pa.pad_centre[conn.dst]
            for pad, goal in ((conn.src, target), (conn.dst, pa.pad_centre[conn.src])):
                ra = reach_of(pad, la)
                if not ra:
                    continue
                tris = np.fromiter(ra, dtype=np.int64)
                pts = np.concatenate([np.array([pa.tri_cen[t] for t in tris.tolist()]), pa.edge_mid[pa.tri_e[tris]].reshape(-1, 2)])
                base = np.array([ra[t] for t in tris.tolist()])
                cost = np.concatenate([base, np.repeat(base, 3)])
                ok = np.ones(len(pts), dtype=bool)
                for s in safe:
                    ok &= shapely.contains_xy(s, pts[:, 0], pts[:, 1])
                for x, y in taken:
                    ok &= np.hypot(pts[:, 0] - x, pts[:, 1] - y) >= spacing
                if ok.any():
                    pts, cost = pts[ok], cost[ok]
                    i = int(np.argmin(cost + 2.0 * np.hypot(pts[:, 0] - goal[0], pts[:, 1] - goal[1])))
                    return (float(pts[i, 0]), float(pts[i, 1]))
    return None


def add_via(board: Board, net_id: int, at: tuple[float, float]) -> int:
    """Adds a via pad of ``net_id`` to the board and returns its pad id."""
    pad_id = max((p.pad_id for p in board.pads), default=-1) + 1
    board.pads.append(Pad.circle(pad_id, at[0], at[1], board.rules.via_diameter / 2.0, net_id,
                                 name=f"via{pad_id}", is_via=True))
    return pad_id


# -- vias placed during the pass (design 12.4) ---------------------------------

TRIES = 4      # searches per connection: a point that turns out not to fit is struck out and the search repeated
GRID = 0.25    # mm: cell of the coarse map of where a via may be


def _triangles_of(pmap: PlanarMap, pts: np.ndarray) -> np.ndarray:
    """Triangle of the map containing each point (-1 = none)."""
    from weaveengine.topo import kernel
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


class _Legal:
    """Where a via's centre may be: inside every layer's free space, shrunk so
    that the via copper keeps its clearance from fixed copper and the board
    edge. The exact regions, and a coarse grid of them for testing many points
    at once. Other vias and the traces are checked separately."""

    def __init__(self, ctx: Context):
        rules = ctx.board.rules
        margin = rules.via_diameter / 2.0 / math.cos(math.pi / 16.0) - rules.base_width / 2.0 + 0.01
        self.regions = []
        for layer in ctx.layers:
            region = layer.pmap.free_space.free.buffer(-margin)
            shapely.prepare(region)
            self.regions.append(region)
        x0, y0, x1, y1 = ctx.board.outline.bounds
        self.x0, self.y0 = x0, y0
        nx, ny = int((x1 - x0) / GRID) + 1, int((y1 - y0) / GRID) + 1
        gx, gy = np.meshgrid(x0 + (np.arange(nx) + 0.5) * GRID, y0 + (np.arange(ny) + 0.5) * GRID, indexing="ij")
        ok = np.ones(gx.shape, dtype=bool)
        for region in self.regions:
            ok &= shapely.contains_xy(region, gx, gy)
        self.grid = ok

    def roughly(self, pts: np.ndarray) -> np.ndarray:
        ix = np.clip(((pts[:, 0] - self.x0) / GRID).astype(np.int64), 0, self.grid.shape[0] - 1)
        iy = np.clip(((pts[:, 1] - self.y0) / GRID).astype(np.int64), 0, self.grid.shape[1] - 1)
        return self.grid[ix, iy]

    def exactly(self, x: float, y: float) -> bool:
        return all(shapely.contains_xy(region, x, y) for region in self.regions)


def _legal(ctx: Context) -> _Legal:
    if getattr(ctx, "_via_legal", None) is None:
        ctx._via_legal = _Legal(ctx)
    return ctx._via_legal


def _points(pmap: PlanarMap, tris: np.ndarray) -> np.ndarray:
    """Points inside each triangle where a site might go: the incentre, the
    centroid, and one towards each corner. [len(tris), 5, 2]"""
    xy = np.stack([pmap.vx[pmap.tri_v[tris]], pmap.vy[pmap.tri_v[tris]]], axis=2)   # [n, 3, 2]
    cen = xy.mean(axis=1)
    side = np.linalg.norm(xy[:, [1, 2, 0]] - xy[:, [2, 0, 1]], axis=2)             # length opposite each corner
    inc = (xy * side[:, :, None]).sum(axis=1) / side.sum(axis=1)[:, None]
    return np.concatenate([inc[:, None], cen[:, None], (xy + inc[:, None]) / 2.0], axis=1)


def _room_for_wires(layer, t: int, point, keep: float) -> bool:
    """Whether the point really lies in the middle cell of triangle ``t``: far
    enough from each corner for the via's keep-off and the wires cutting that corner."""
    pmap, state = layer.pmap, layer.state
    if sites.room(pmap, t, point) < sites.MIN_ROOM * sites.SITE_RADIUS:
        return False
    for k, v in enumerate(pmap.tri_v_list[t]):
        if math.dist(point, pmap.vxy[v]) - keep < (state.corner_cnt[t][k] - 1) * pmap.pitch:
            return False
    return True


def _room_many(layer, tris: np.ndarray, pts: np.ndarray, keep: float) -> np.ndarray:
    """``_room_for_wires`` for many points at once (``tris[i]`` holds ``pts[i]``)."""
    pmap, state = layer.pmap, layer.state
    xy = np.stack([pmap.vx[pmap.tri_v[tris]], pmap.vy[pmap.tri_v[tris]]], axis=2)       # [n, 3, 2]
    d = np.hypot(xy[:, :, 0] - pts[:, None, 0], xy[:, :, 1] - pts[:, None, 1])
    ok = (d - keep >= (state.corner[tris] - 1) * pmap.pitch).all(axis=1)
    a, b = xy, xy[:, [1, 2, 0]]
    cross = (b[:, :, 0] - a[:, :, 0]) * (pts[:, None, 1] - a[:, :, 1]) - (b[:, :, 1] - a[:, :, 1]) * (pts[:, None, 0] - a[:, :, 0])
    height = cross / np.maximum(np.hypot(b[:, :, 0] - a[:, :, 0], b[:, :, 1] - a[:, :, 1]), 1e-12)
    return ok & (height.min(axis=1) >= sites.MIN_ROOM * sites.SITE_RADIUS)


def _taken(ctx: Context) -> np.ndarray:
    return np.array([s.centre for s in ctx.layers[0].pmap.sites.values() if s.active]
                    + [p.centre for p in ctx.board.pads if p.is_via]).reshape(-1, 2)


def _search(ctx: Context, conn: Connection, max_vias: int, struck: set) -> list[tuple[float, float]] | None:
    """The cheapest legal way from one end of the connection to the other that
    changes layer at most ``max_vias`` times, as the list of points to put vias
    at. One flood per layer per via: each starts from every point the flood
    before could reach, at its cost so far plus the cost of a via."""
    params, rules = ctx.params, ctx.board.rules
    legal = _legal(ctx)
    keep_off = sites.keep_off(rules)
    spacing = rules.via_diameter + rules.clearance + 0.02
    taken = _taken(ctx)
    tree = cKDTree(taken) if len(taken) else None
    front = {}
    for layer in ctx.layers:
        if conn.src in layer.pmap.pad_edges:
            best, origin, goal = flood(layer.pmap, layer.state, conn.src, conn.weight + params.spare, dst_pad=conn.dst)
            if math.isfinite(goal[0]):
                return None  # a plain route exists: no via is needed
            front[layer.index] = (best, origin, None)
    hops = [front]
    reached = None  # (cost, hop, layer, seed) of the cheapest arrival so far
    for _ in range(max_vias):
        front = {}
        for layer in ctx.layers:
            pts, cost, source = [], [], []
            for la, (best, _, _) in hops[-1].items():
                if la == layer.index:
                    continue
                tris = np.nonzero(np.isfinite(best))[0]
                if not len(tris):
                    continue
                p = _points(ctx.layers[la].pmap, tris).reshape(-1, 2)
                ok = legal.roughly(p)
                if tree is not None:
                    ok &= tree.query(p, distance_upper_bound=spacing)[0] > spacing
                if struck:  # points near one that turned out not to fit
                    ok &= cKDTree(np.array(sorted(struck))).query(p, distance_upper_bound=keep_off)[0] > keep_off
                if reached is not None:
                    ok &= np.repeat(best[tris], 5) + params.via_cost < reached[0]  # cannot beat what has been found
                keep = np.nonzero(ok)[0]
                keep = keep[_room_many(ctx.layers[la], np.repeat(tris, 5)[keep], p[keep], keep_off)]
                pts.append(p[keep])
                cost.append(np.repeat(best[tris], 5)[keep] + params.via_cost)
                source.append(np.stack([np.full(len(keep), la), np.repeat(tris, 5)[keep]], axis=1))
            if not pts or not sum(len(p) for p in pts):
                continue
            pts, cost, source = np.concatenate(pts), np.concatenate(cost), np.concatenate(source)
            tb = _triangles_of(layer.pmap, pts)
            inside = np.nonzero(tb >= 0)[0]
            inside = inside[_room_many(layer, tb[inside], pts[inside], keep_off)]
            pts, cost, source, tb = pts[inside], cost[inside], source[inside], tb[inside]
            if not len(pts):
                continue
            best, origin, goal = flood(layer.pmap, layer.state, None, conn.weight + params.spare,
                                       seeds=(tb, cost, pts[:, 0], pts[:, 1]), dst_pad=conn.dst)
            front[layer.index] = (best, origin, (pts, source))
            if math.isfinite(goal[0]) and (reached is None or goal[0] < reached[0]):
                reached = (goal[0], len(hops), layer.index, goal[1])
        if not front:
            break
        hops.append(front)
    if reached is None:
        return None
    # More vias are tried even after a way is found: a via costs a few
    # millimetres, and the way with fewest vias is often a long way round.
    chain, (_, top, li, seed) = [], reached
    for level in range(top, 0, -1):
        pts, source = hops[level][li][2]
        chain.append((float(pts[seed, 0]), float(pts[seed, 1])))
        li, tri = int(source[seed, 0]), int(source[seed, 1])
        seed = int(hops[level - 1][li][1][tri])
    return chain[::-1]


def connect(ctx: Context, conn: Connection) -> bool:
    """Connects an open connection through vias (design 12.4), or leaves
    everything as it was and returns False.

    The points come from ``_search``. A via site is made at each on every
    layer; the connection is split there into children, and each child is
    routed legally on one layer. If a site does not fit among the wires, it is
    taken out again, its point struck out, and the search repeated.
    """
    from weaveengine.plan.candidates import plain_route
    params, rules = ctx.params, ctx.board.rules
    room = params.max_vias - ctx.via_count(conn.wire_id)
    if room <= 0 or len(ctx.layers) < 2 or conn.children:
        return False
    keep = sites.keep_off(rules)
    legal = _legal(ctx)
    spacing = rules.via_diameter + rules.clearance + 0.02
    struck: set = set()
    notes = ctx.via_notes
    for _ in range(TRIES):
        ctx.lift(conn, True)
        try:
            chain = _search(ctx, conn, room, struck)
        finally:
            ctx.lift(conn, False)
        if not chain:
            notes["no way found"] += 1
            return False
        ctx.split_count += 1
        mark, pads, bad = ctx.mark(), [], None
        taken = _taken(ctx)
        for n, (x, y) in enumerate(chain):
            near = [math.dist((x, y), q) for q in taken.tolist() + list(chain[:n])]
            where = [int(_triangles_of(layer.pmap, np.array([[x, y]]))[0]) for layer in ctx.layers]
            fits = (not near or min(near) >= spacing) and legal.exactly(x, y) and min(where) >= 0
            if not fits:
                notes["point not legal"] += 1
            elif not all(_room_for_wires(layer, t, (x, y), keep) for layer, t in zip(ctx.layers, where)):
                notes["no room in the triangle"] += 1
                fits = False
            if fits:
                try:
                    made = [sites.create(layer.pmap, layer.state, t, (x, y), pad=ctx.next_pad + n) for layer, t in zip(ctx.layers, where)]
                    fits = all(sites.fits(layer.pmap, layer.state, site, keep, params.spare) for layer, site in zip(ctx.layers, made))
                except ValueError:
                    fits = False
                if not fits:
                    notes["site does not fit"] += 1
            if not fits:
                bad = (round(x, 4), round(y, 4))
                break
            for layer, site in zip(ctx.layers, made):
                sites.set_net(layer.pmap, layer.state, site, conn.net_id, keep)
            pads.append(ctx.next_pad + n)
        if bad is not None:
            ctx.rollback(mark)  # no room among the wires for a via here: leave no trace of the attempt
            struck.add(bad)
            continue
        ctx.maps_changed()
        first_pad, first_wire = ctx.next_pad, ctx.next_wire
        ctx.next_pad += len(pads)
        piece = conn
        for pad in pads:
            _, piece = ctx.split(piece, pad)
        for w in range(first_wire, ctx.next_wire):
            if w in ctx.unrouted:
                r = plain_route(ctx, ctx.conns[w], params.spare, hard_cap=True)
                if r is not None:
                    ctx.commit(ctx.conns[w], r)
        if ctx.connected(conn.wire_id):
            ctx.keep()
            notes["connected"] += 1
            notes["vias"] += len(pads)
            return True
        # A piece could not be routed after all (the search allows ways that
        # cross a gate twice; a route may not). Take everything out again.
        notes["piece not routable"] += 1
        ctx.abandon(conn, delete=False)
        ctx.rollback(mark)
        ctx.next_pad, ctx.next_wire = first_pad, first_wire
        struck.update((round(x, 4), round(y, 4)) for x, y in chain)
    return False
