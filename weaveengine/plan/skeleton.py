"""Pours that stay whole (design 11.3).

A pour is left to KiCad to fill round the traces, and traces of other nets
that cross it, or close a ring round part of it, cut it into pieces. Here
each pour (a net and a layer) is given a *skeleton*: a tree joining the pads
of its net that lie inside its outline, routed on that layer only, held to
the gates inside the outline, as a wire wide enough to hold open a strip of
fill (``width``) with the trace clearance either side. It is a set of ordinary
connections, negotiated like every other (not placed first or last), so it
takes its room in the gates by its weight in the load. Wires never cross, so
no trace cuts it, and the pour round it is one piece that reaches every pad
it joins. It is not written to the session: it is the fill's, not copper of
its own. Where it loses, its pad is left to a via down to the net's plane on
another layer (``plan.fanout``).
"""
import math

import numpy as np
import shapely
import shapely.prepared

from weaveengine.plan.fanout import FILL
from weaveengine.topo.planar_map import TERMINAL

# How much wider than a fill's minimum width the strip a skeleton holds open is,
# so that it is not lost to the fill's own rounding (``pours.estimate``).
SLACK = 0.05


def width(board, net: int) -> float:
    """The width the skeletons of ``net`` are routed at: wide enough that the fill keeps a strip of its minimum
    width along it (its zones' min_thickness, or the setting, and ``SLACK``) between traces of other nets at the
    board's clearance either side, where the zone keeps a larger clearance than that; for the widest of its pours.
    Never below the narrowest trace, which the maps were built for (``Rules.base_width``)."""
    rules = board.rules
    need = [board.pour_width(n, at) + SLACK + 2.0 * max(0.0, board.pour_clearance(n, at) - rules.clearance)
            for n, at, _ in board.pours if n == net]
    return max(max(need, default=FILL + SLACK), rules.base_width)


def area(board, k: int):
    """Where the skeleton of pour ``k`` may run: its outline, less the outlines of other nets' pours on its
    layer of higher priority (they fill first, and its fill keeps clear of theirs)."""
    net, at, outline = board.pours[k]
    rank = board.zone(net, at).priority
    above = [shape for n, a, shape in board.pours if a == at and n != net and board.zone(n, a).priority > rank]
    return outline.difference(shapely.union_all(above)) if above else outline


def plan(board, layers) -> list[tuple[int, int, int, int]]:
    """The skeletons' connections, as (net, pad, pad, layer): per net with
    pours on routed layers, a spanning tree over its pads inside them that a
    trace reaches there and that the zone joins, each join through one pour.
    A pad through the board is the same pad on every layer, so it joins the
    trees of the net's pours on each: the tree is the shortest of the joins
    each pour's own spanning tree would make (pads whose copper meets are
    joined already)."""
    from weaveengine.plan.context import _touching, spanning_pairs
    from weaveengine.pours import joined
    by_id = {p.pad_id: p for p in board.pads}
    centre = {p.pad_id: p.centre for p in board.pads}
    joins: dict[int, list] = {}  # net -> (length, pad, pad, layer) the pours' own trees would make
    for k, (net, at, _) in enumerate(board.pours):
        if at >= len(layers):
            continue  # a plane on a layer left out of the routing: no trace there to cut it
        edges = layers[at].pmap.pad_edges
        zone = board.zone(net, at)
        inside = shapely.prepared.prep(area(board, k))
        pads = sorted(p.pad_id for p in board.pads if p.net_id == net and p.on(at) and p.pad_id in edges
                      and inside.intersects(p.shape) and joined(p, zone) != "none")
        if len(pads) < 2:
            continue
        touching = _touching([by_id[p] for p in pads])
        joins.setdefault(net, []).extend((math.dist(centre[a], centre[b]), a, b, at) for a, b, _ in spanning_pairs(pads, centre, touching))
    out = []
    for net, found in sorted(joins.items()):
        pads = sorted({p for _, a, b, _ in found for p in (a, b)})
        group = {p: p for p in pads}

        def root(p: int) -> int:
            while group[p] != p:
                group[p] = p = group[group[p]]
            return p

        for i, j in _touching([by_id[p] for p in pads]):
            group[root(pads[i])] = root(pads[j])
        for _, a, b, at in sorted(found):
            if root(a) != root(b):
                group[root(a)] = root(b)
                out.append((net, a, b, at))
    return out


def hold(board, layers, nets: set[int]) -> None:
    """Holds every search for a wire of ``nets`` (the skeletons') to the gates
    inside its net's pours on each layer (less those of other nets of higher
    priority: ``area``), and the edges of its net's pads (``topo.search.route``
    reads this). On a layer where the net has no pour it is held to nothing."""
    for layer in layers:
        pmap = layer.pmap
        held = pmap.__dict__["_held"] = {}
        for net in nets:
            shapes = [area(board, k) for k, (n, at, _) in enumerate(board.pours) if n == net and at == layer.index]
            room = shapely.union_all(shapes) if shapes else None
            if room is not None:
                shapely.prepare(room)
            held[net] = [room, np.zeros(0, dtype=np.uint8), -1]
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
