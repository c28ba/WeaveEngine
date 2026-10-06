"""Gate-sequence paths and topological crossing counts (design section 9.4)."""
import numpy as np

from weaveengine.topo.planar_map import PlanarMap
from weaveengine.topo.state import Step


class GatePath:
    """A path as the ordered gates it crosses. ``u_left[i]`` says whether the
    u end of gate i lies to the left of the path's direction of travel."""
    __slots__ = ("gates", "u_left", "pos")

    def __init__(self, gates, u_left):
        self.gates = tuple(gates)
        self.u_left = tuple(u_left)
        self.pos = {g: i for i, g in enumerate(self.gates)}


def path_from_steps(pmap: PlanarMap, steps: list[Step]) -> GatePath:
    mids, cen, vxy, edge_v = pmap.edge_mid_list, pmap.tri_cen, pmap.vxy, pmap.edge_v_list
    u_left = []
    for i, (e, t, _, _) in enumerate(steps):
        mx, my = mids[e]
        if i == 0:
            cx, cy = cen[steps[1][1]]
            dx, dy = cx - mx, cy - my      # heading into the first triangle
        else:
            cx, cy = cen[t]
            dx, dy = mx - cx, my - cy      # heading out of the triangle just crossed
        ux, uy = vxy[edge_v[e][0]]
        u_left.append(dx * (uy - my) - dy * (ux - mx) > 0)
    return GatePath([s[0] for s in steps], u_left)


def airwire_path(pmap: PlanarMap, p, q) -> GatePath:
    """All edges (gates, walls and terminals) crossed by the straight segment p -> q, in order."""
    dx, dy = q[0] - p[0], q[1] - p[1]
    u, v = pmap.edge_v[:, 0], pmap.edge_v[:, 1]
    ux, uy = pmap.vx[u] - p[0], pmap.vy[u] - p[1]
    vx, vy = pmap.vx[v] - p[0], pmap.vy[v] - p[1]
    side_u = dx * uy - dy * ux
    side_v = dx * vy - dy * vx
    ex, ey = vx - ux, vy - uy
    den = dx * ey - dy * ex
    with np.errstate(divide="ignore", invalid="ignore"):
        t = (ux * ey - uy * ex) / den
    hit = (side_u * side_v < 0) & (t > 0) & (t < 1)
    idx = np.nonzero(hit)[0]
    idx = idx[np.argsort(t[idx])]
    return GatePath(idx.tolist(), (side_u[idx] > 0).tolist())


def cross_count(pmap: PlanarMap, a: GatePath, b: GatePath) -> int:
    """Number of forced crossings between two paths, from gate sequences alone.

    For each maximal shared run of gates, the pair's order is read at both ends
    (whichever path leaves through the edge adjacent to an endpoint of the last
    shared gate is nearer that endpoint). The run is one crossing when both
    ends are determined and the orders are opposite.
    """
    ga, posa, posb = a.gates, a.pos, b.pos
    na = len(ga)
    count = 0
    done = -1
    for i in sorted(posa[g] for g in posa.keys() & posb.keys()):
        if i <= done:
            continue  # within the run before
        j = posb[ga[i]]
        i_end, j_end, step = i, j, 0
        while i_end + 1 < na:
            jn = posb.get(ga[i_end + 1])
            if jn is None or abs(jn - j_end) != 1 or (step and jn - j_end != step):
                break
            step = jn - j_end
            i_end, j_end = i_end + 1, jn
        first = _b_left(pmap, a, b, i, j, -1)
        last = _b_left(pmap, a, b, i_end, j_end, +1)
        if first is not None and last is not None and first != last:
            count += 1
        done = i_end
    return count


def _b_left(pmap: PlanarMap, a: GatePath, b: GatePath, i: int, j: int, direction: int):
    """Is b to the left of a (in a's direction of travel) where the run ends? None = undetermined."""
    ga, gb = a.gates, b.gates
    ia = i + direction
    if not 0 <= ia < len(ga):
        return None
    g, a_x = ga[i], ga[ia]
    t = pmap.common_triangle(g, a_x)
    if t < 0:
        return None
    tri_edges = pmap.tri_e_list[t]
    b_x = -1
    for jb in (j - 1, j + 1):
        if 0 <= jb < len(gb) and gb[jb] != g and gb[jb] in tri_edges:
            b_x = gb[jb]
    if b_x < 0 or b_x == a_x:
        return None
    a_near_u = pmap.shared_vertex(g, a_x) == pmap.edge_v_list[g][0]
    return a_near_u != a.u_left[i]
