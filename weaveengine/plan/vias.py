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

from weaveengine.board import Board, Pad
from weaveengine.plan.context import Connection, Context
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
            reach[(pad, li)] = flood(layer.pmap, layer.state, pad, conn.weight)
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
