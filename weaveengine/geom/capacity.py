"""Gate width estimate and capacity (design section 6.3)."""
import numpy as np
import shapely


def edge_width(verts, edge_v, edge_len, edge_kind, v_obs, obs_geom) -> np.ndarray:
    """Width estimate w(e) of every edge (6.3).

    w(e) = min(|e|, dist(u, obstacle(v)), dist(v, obstacle(u))). The distance
    terms stop slanted edges between parallel obstacles being overestimated.
    They are skipped when both ends lie on the same obstacle (they would be 0).
    """
    u, v = edge_v[:, 0], edge_v[:, 1]
    width = edge_len.copy()
    differ = (v_obs[u] != v_obs[v]) & (v_obs[u] >= 0) & (v_obs[v] >= 0) & (edge_kind == 0)
    if differ.any():
        iu, iv = u[differ], v[differ]
        geom_u = np.array([obs_geom[o] for o in v_obs[iu].tolist()], dtype=object)
        geom_v = np.array([obs_geom[o] for o in v_obs[iv].tolist()], dtype=object)
        d_uv = shapely.distance(shapely.points(verts[iu]), geom_v)
        d_vu = shapely.distance(shapely.points(verts[iv]), geom_u)
        width[differ] = np.minimum(width[differ], np.minimum(d_uv, d_vu))
    return width


def edge_capacity(verts, edge_v, edge_len, edge_kind, v_obs, obs_geom, pitch: float) -> np.ndarray:
    """cap(e) = floor(w(e) / (t + s)) + 1, with walls at 0."""
    return capacity_from_width(edge_width(verts, edge_v, edge_len, edge_kind, v_obs, obs_geom), edge_kind, pitch)


def capacity_from_width(width: np.ndarray, edge_kind: np.ndarray, pitch: float) -> np.ndarray:
    cap = np.floor(width / pitch + 1e-9).astype(np.int64) + 1
    cap[edge_kind == 1] = 0
    return cap
