"""Where a trace may leave a pad (design section 13.1, step 4).

A trace ends on its pad's keep-off ring and a straight stub joins that point to
the pad centre. The ring only guarantees that the point itself is clear of
foreign copper. Where the rings of neighbouring pads have merged (fine-pitch
parts), a stub from the part of the ring over the gap cuts across towards the
neighbour and comes too close to it. So every pad edge gets a window: the part
of it from which the stub keeps its clearance. An edge with no such part is not
a way out of the pad.
"""
import numpy as np
import shapely

from weaveengine.board import Board

SAMPLES = 17   # points tried along each pad edge
SLACK = 1e-4   # mm: a point on the ring is at the required distance to within rounding


def exit_windows(board: Board, layer: int, pmap) -> tuple[dict[int, tuple[float, float]], list[int]]:
    """(windows, dead): for the pad edges that are restricted, the legal part as
    (from, to) in mm along the edge from its u end; and the pad edges from which
    no stub is legal. Edges not mentioned are legal along their whole length."""
    rules = board.rules
    pads = {p.pad_id: p for p in board.pads_on(layer)}
    copper = [(p.shape, p.net_id, p.pad_id) for p in pads.values()]
    copper += [(o.shape, o.net_id, -1) for o in board.obstacles_on(layer)]
    edges = [e for e, kind in enumerate(pmap.edge_kind_list) if kind == 2 and pmap.edge_owner_list[e] in pads]
    if not edges or not copper:
        return {}, []
    tree = shapely.STRtree([c[0] for c in copper])
    fractions = np.linspace(0.0, 1.0, SAMPLES)
    stubs, need = [], []
    for e in edges:
        pad = pads[pmap.edge_owner_list[e]]
        (ux, uy), (vx, vy) = (pmap.vxy[v] for v in pmap.edge_v_list[e])
        cx, cy = pad.centre
        for f in fractions:
            stubs.append(((cx, cy), (ux + f * (vx - ux), uy + f * (vy - uy))))
        need.append(rules.clearance + rules.width(pad.net_id) / 2.0)
    lines = shapely.linestrings(np.array(stubs))
    legal = np.ones(len(lines), dtype=bool)
    reach = np.repeat(need, SAMPLES)
    li, ci = tree.query(lines, predicate="dwithin", distance=float(max(need)))
    for i, c in zip(li.tolist(), ci.tolist()):
        if not legal[i]:
            continue
        pad = pads[pmap.edge_owner_list[edges[i // SAMPLES]]]
        shape, net, pad_id = copper[c]
        if pad_id == pad.pad_id or (net >= 0 and net == pad.net_id):
            continue
        if lines[i].distance(shape) < reach[i] - SLACK:
            legal[i] = False
    windows: dict[int, tuple[float, float]] = {}
    dead: list[int] = []
    for n, e in enumerate(edges):
        ok = legal[n * SAMPLES:(n + 1) * SAMPLES]
        if ok.all():
            continue
        if not ok.any():
            dead.append(e)
            continue
        # The longest run of legal points (one run, except in odd corners).
        best, start = (0, 0), None
        for i, good in enumerate(list(ok) + [False]):
            if good and start is None:
                start = i
            elif not good and start is not None:
                if i - start > best[1] - best[0]:
                    best = (start, i)
                start = None
        length = pmap.edge_len_list[e]
        windows[e] = (float(fractions[best[0]]) * length, float(fractions[best[1] - 1]) * length)
    return windows, dead
