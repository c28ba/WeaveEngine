"""Preprocessing (design section 5): inflate copper so a wire is a zero-width curve."""
from dataclasses import dataclass

import numpy as np
import shapely
from shapely.geometry import LinearRing, MultiPolygon, Polygon
from shapely.ops import unary_union

from weaveengine.board import EPS, Board

WALL = -1
MITRE_LIMIT = 1.1


@dataclass
class FreeSpace:
    """Routable region and its classified boundary."""
    free: Polygon | MultiPolygon
    rings: list[np.ndarray]        # per obstacle ring: closed coordinate array [N+1, 2]
    ring_obs: list[int]            # obstacle id per ring (0 = outline)
    ring_owner: list[np.ndarray]   # per ring segment: owning pad id, or WALL
    obs_geom: dict[int, object]    # obstacle id -> boundary geometry (for capacity)
    inflated_pads: dict[int, Polygon]


def _inflate(shape: Polygon, delta: float) -> Polygon:
    # Limited mitre joins: the result always contains the true round offset, so
    # no clearance is lost at convex corners (13.1.3), and it adds few vertices.
    return shape.buffer(delta, join_style="mitre", mitre_limit=MITRE_LIMIT)


def preprocess(board: Board, layer: int = 0, max_segment: float = 2.0) -> FreeSpace:
    rules = board.rules
    delta = rules.inflation

    inflated_pads = {p.pad_id: _inflate(p.shape, delta) for p in board.pads_on(layer)}
    inflated = list(inflated_pads.values())
    inflated += [_inflate(o.shape, delta) for o in board.obstacles_on(layer)]

    region = board.outline.buffer(-rules.outline_inset, join_style="mitre")
    free = region.difference(unary_union(inflated)) if inflated else region
    free = shapely.set_precision(free, EPS)
    # Long walls get intermediate vertices (still on the obstacle, so every
    # gate stays a choke between two boundaries). Without them the only gates
    # from a pad to a long wall run to its far-away corners, and neither the
    # capacity estimate nor the realisation sees the real pad-to-wall gap.
    free = shapely.segmentize(free, max_segment * rules.pitch)
    polys = [g for g in getattr(free, "geoms", [free]) if isinstance(g, Polygon) and g.area > EPS]
    if not polys:
        raise ValueError("board has no routable free space")
    free = polys[0] if len(polys) == 1 else MultiPolygon(polys)

    # Merged obstacle pieces: every exterior ring is the outline group (id 0),
    # every interior ring is one merged piece with its own id.
    rings: list[np.ndarray] = []
    ring_obs: list[int] = []
    next_id = 1
    for poly in polys:
        rings.append(np.asarray(poly.exterior.coords))
        ring_obs.append(0)
        for hole in poly.interiors:
            rings.append(np.asarray(hole.coords))
            ring_obs.append(next_id)
            next_id += 1

    # Terminal ownership (5.4): a boundary segment belongs to pad P when it
    # lies on P's own inflated boundary.
    pad_ids = list(inflated_pads)
    tree = shapely.STRtree([inflated_pads[i].exterior for i in pad_ids]) if pad_ids else None
    ring_owner = []
    for ring in rings:
        owner = np.full(len(ring) - 1, WALL, dtype=np.int64)
        if tree is not None:
            mids = shapely.points((ring[:-1] + ring[1:]) / 2.0)
            seg_idx, pad_idx = tree.query(mids, predicate="dwithin", distance=10 * EPS)
            for s, p in zip(seg_idx.tolist(), pad_idx.tolist()):
                if owner[s] == WALL:
                    owner[s] = pad_ids[p]
        ring_owner.append(owner)

    by_obs: dict[int, list] = {}
    for ring, obs in zip(rings, ring_obs):
        by_obs.setdefault(obs, []).append(LinearRing(ring))
    obs_geom = {obs: (rs[0] if len(rs) == 1 else shapely.multilinestrings(rs)) for obs, rs in by_obs.items()}

    return FreeSpace(free, rings, ring_obs, ring_owner, obs_geom, inflated_pads)
