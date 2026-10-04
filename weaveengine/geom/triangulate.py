"""Constrained Delaunay triangulation of free space (design section 6.1)."""
import numpy as np
import shapely

from weaveengine.geom.inflate import WALL, FreeSpace

try:
    import triangle as _triangle
except ImportError:  # pragma: no cover - exercised via force_fallback
    _triangle = None


def _collect_pslg(fs: FreeSpace):
    """Vertices, constrained segments, per-vertex obstacle id, per-segment owner."""
    index: dict[tuple[float, float], int] = {}
    verts: list[tuple[float, float]] = []
    v_obs: list[int] = []
    segs: dict[tuple[int, int], int] = {}
    v_nbr: dict[int, tuple[int, int]] = {}
    for ring, obs, owners in zip(fs.rings, fs.ring_obs, fs.ring_owner):
        ids = []
        for x, y in ring[:-1].tolist():
            key = (x, y)
            if key not in index:
                index[key] = len(verts)
                verts.append(key)
                v_obs.append(obs)
            ids.append(index[key])
        n = len(ids)
        for i in range(n):
            a, b = ids[i], ids[(i + 1) % n]
            v_nbr.setdefault(a, (ids[i - 1], b))
            if a != b:
                segs[(min(a, b), max(a, b))] = int(owners[i])
    return np.array(verts, dtype=np.float64), v_obs, segs, v_nbr


def _keep_inside(fs: FreeSpace, verts: np.ndarray, tris: np.ndarray) -> np.ndarray:
    """Drop triangles in holes / outside the outline, and make the rest CCW."""
    p = verts[tris]
    cen = p.mean(axis=1)
    tris = tris[shapely.contains_xy(fs.free, cen[:, 0], cen[:, 1])]
    p = verts[tris]
    area2 = ((p[:, 1, 0] - p[:, 0, 0]) * (p[:, 2, 1] - p[:, 0, 1])
             - (p[:, 1, 1] - p[:, 0, 1]) * (p[:, 2, 0] - p[:, 0, 0]))
    tris = tris[np.abs(area2) > 1e-12]
    area2 = area2[np.abs(area2) > 1e-12]
    flip = area2 < 0
    tris[flip] = tris[flip][:, [0, 2, 1]]
    return tris


def _scipy_fallback(verts: np.ndarray, v_obs: list[int], segs: dict[tuple[int, int], int], v_nbr: dict):
    """Plain Delaunay, splitting constrained segments at midpoints until all survive."""
    from scipy.spatial import Delaunay

    verts = verts.tolist()
    for _ in range(40):
        tri = Delaunay(np.array(verts), qhull_options="QJ Pp")
        present = set()
        for a, b, c in tri.simplices.tolist():
            present.update(((min(a, b), max(a, b)), (min(b, c), max(b, c)), (min(a, c), max(a, c))))
        missing = [s for s in segs if s not in present]
        if not missing:
            return np.array(verts), v_obs, segs, tri.simplices.astype(np.int64)
        for a, b in missing:
            owner = segs.pop((a, b))
            m = len(verts)
            verts.append(((verts[a][0] + verts[b][0]) / 2.0, (verts[a][1] + verts[b][1]) / 2.0))
            v_obs.append(v_obs[a])
            v_nbr[m] = (a, b)
            v_nbr[a] = tuple(m if x == b else x for x in v_nbr[a])
            v_nbr[b] = tuple(m if x == a else x for x in v_nbr[b])
            segs[(a, m)] = owner
            segs[(min(b, m), max(b, m))] = owner
    raise RuntimeError("fallback triangulation could not recover all constrained segments")


def triangulate(fs: FreeSpace, force_fallback: bool = False):
    """Returns (verts [V,2], tris [T,3] CCW, v_obs [V], segs {(u,v): owner pad or WALL},
    v_nbr {v: (previous, next) vertex along its obstacle boundary})."""
    verts, v_obs, segs, v_nbr = _collect_pslg(fs)
    if _triangle is not None and not force_fallback:
        pslg = {"vertices": verts, "segments": np.array(list(segs), dtype=np.int32)}
        out = _triangle.triangulate(pslg, "p")
        if len(out["vertices"]) != len(verts):
            raise RuntimeError("triangulation added vertices: constrained segments intersect")
        tris = np.asarray(out["triangles"], dtype=np.int64)
    else:
        verts, v_obs, segs, tris = _scipy_fallback(verts, v_obs, segs, v_nbr)
    tris = _keep_inside(fs, verts, tris)
    return verts, tris, np.array(v_obs, dtype=np.int64), segs, v_nbr


def check_triangulation(fs: FreeSpace, verts, tris, segs) -> list[str]:
    """Triangulation checks from design 17.1. Returns a list of problems (empty = OK)."""
    problems = []
    edges = set()
    for a, b, c in tris.tolist():
        edges.update(((min(a, b), max(a, b)), (min(b, c), max(b, c)), (min(a, c), max(a, c))))
    missing = [s for s in segs if s not in edges]
    if missing:
        problems.append(f"{len(missing)} boundary segments are not triangulation edges")
    p = verts[tris]
    area = 0.5 * ((p[:, 1, 0] - p[:, 0, 0]) * (p[:, 2, 1] - p[:, 0, 1])
                  - (p[:, 1, 1] - p[:, 0, 1]) * (p[:, 2, 0] - p[:, 0, 0]))
    if (area <= 0).any():
        problems.append("degenerate or clockwise triangle")
    if abs(area.sum() - fs.free.area) > 1e-6 * max(1.0, fs.free.area):
        problems.append(f"triangle area {area.sum():.6f} != free area {fs.free.area:.6f}")
    return problems
