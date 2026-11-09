"""Pours that stay whole (design 11.3).

A pour is left to KiCad to fill round the traces, and traces of other nets
that cross it, or close a ring round part of it, cut it into pieces. Here
each pour (a net and a layer) is given a *skeleton*: a tree joining the pads
of its net that lie inside its outline, routed on that layer only, held to
the gates inside the outline, as a wire as wide as a fill may be narrow
(``SKELETON``) with the trace clearance either side. It is a set of ordinary
connections, negotiated like every other (not placed first or last), so it
takes its room in the gates by its weight in the load. Wires never cross, so
no trace cuts it, and the pour round it is one piece that reaches every pad
it joins. It is not written to the session: it is the fill's, not copper of
its own. Where it loses, its pad is left to a via down to the net's plane on
another layer (``plan.fanout``).
"""
import numpy as np
import shapely
import shapely.prepared

from weaveengine.plan.fanout import FILL
from weaveengine.topo.planar_map import TERMINAL

# How wide a skeleton is drawn: a fill's minimum width and a little more, so that
# the strip it holds open is not lost to the fill's own rounding (``pours.estimate``).
SKELETON = 1.4 * FILL


def width(rules) -> float:
    """The width a skeleton is routed and drawn at: never below the narrowest
    trace, which the maps were built for (``Rules.base_width``)."""
    return max(SKELETON, rules.base_width)


def plan(board, layers) -> list[tuple[int, int, int, int]]:
    """The skeletons' connections, as (net, pad, pad, layer): per pour on a
    routed layer, a spanning tree over the pads of its net inside its outline
    that a trace reaches on that layer (pads whose copper meets are joined
    already)."""
    from weaveengine.plan.context import _touching, spanning_pairs
    by_id = {p.pad_id: p for p in board.pads}
    centre = {p.pad_id: p.centre for p in board.pads}
    out = []
    for net, at, outline in board.pours:
        if at >= len(layers):
            continue  # a plane on a layer left out of the routing: no trace there to cut it
        edges = layers[at].pmap.pad_edges
        inside = shapely.prepared.prep(outline)
        pads = sorted(p.pad_id for p in board.pads if p.net_id == net and p.on(at) and p.pad_id in edges and inside.intersects(p.shape))
        if len(pads) < 2:
            continue
        touching = _touching([by_id[p] for p in pads])
        out += [(net, a, b, at) for a, b, _ in spanning_pairs(pads, centre, touching)]
    return out


def hold(board, layers, nets: set[int]) -> None:
    """Holds every search for a wire of ``nets`` (the skeletons') to the gates
    inside its net's pours on each layer, and the edges of its net's pads
    (``topo.search.route`` reads this). On a layer where the net has no pour
    it is held to nothing."""
    for layer in layers:
        pmap = layer.pmap
        held = pmap.__dict__["_held"] = {}
        for net in nets:
            shapes = [shape for n, at, shape in board.pours if n == net and at == layer.index]
            area = shapely.union_all(shapes) if shapes else None
            if area is not None:
                shapely.prepare(area)
            held[net] = [area, np.zeros(0, dtype=np.uint8), -1]
            region(pmap, net)


def region(pmap, net: int) -> np.ndarray | None:
    """The edges a search for a wire of ``net`` may cross (1) on this map, or
    None if it is not held. Brought up to date as via sites add and change edges."""
    held = pmap.__dict__.get("_held")
    if not held or net not in held:
        return None
    area, mask, seen = held[net]
    if len(mask) == pmap.num_edges and seen == pmap.changes:
        return mask
    first = len(mask)
    mask = np.concatenate([mask, np.zeros(pmap.num_edges - first, dtype=np.uint8)])
    todo = np.concatenate([np.flatnonzero(pmap.edge_changed[:first] > seen), np.arange(first, pmap.num_edges)]).astype(np.int64)
    if area is not None and len(todo):
        mids = np.asarray([pmap.edge_mid_list[e] for e in todo.tolist()], dtype=np.float64).reshape(-1, 2)
        mask[todo] = shapely.contains_xy(area, mids[:, 0], mids[:, 1])
        kind, owner = pmap.edge_kind_list, pmap.edge_owner_list
        for e in todo.tolist():
            if kind[e] == TERMINAL:
                mask[e] = pmap.pad_net.get(owner[e], -1) == net
    held[net] = [area, mask, pmap.changes]
    return mask
