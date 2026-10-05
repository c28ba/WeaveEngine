"""Via sites created inside the planar map (design section 12.2).

A site is a tiny triangular hole put into a triangle of the map. Its three
edges are terminal edges of the site's own pad id. While the site is dormant
(net -1) it takes no room and wires pass it on either side. Made active, it is
a via of a net: wires of other nets are kept ``keep`` away from it, and wires
of its net may start and end on it.

Creating a site in triangle T, with corners a0 a1 a2 and hole p0 p1 p2,
replaces T by six triangles (j = i + 1, all indices mod 3):

    A_i = (a_i, a_j, p_j)   holds the outer edge a_i a_j
    S_i = (a_i, p_j, p_i)   holds the hole edge p_i p_j

Each corner a_i gets two new gates ("spokes"), a_i p_i and a_i p_j. The site is
declared to lie in the middle cell of T, the part no wire has cut off, so every
wire cutting corner a_i now also crosses the two spokes at a_i, in its nesting
order, and nothing else about any wire changes.

The triangle that was split is usually long and thin, so the site's first
spokes say little about what is really near it. Edges round the site are then
flipped until they are Delaunay again (``legalise``): the site gains a spoke
to every vertex that is truly its neighbour, and each spoke is a gate whose
capacity and wire order hold the via's surroundings in the same terms as the
rest of the map.
"""
import math
from dataclasses import dataclass

import numpy as np
from shapely.geometry import LinearRing, Point

from weaveengine.board import Pad, Rules
from weaveengine.topo.planar_map import GATE, TERMINAL, PlanarMap
from weaveengine.topo.state import TopoState

SITE_RADIUS = 0.002   # mm: size of the hole in the map; inside the routing margin (design section 4)
MIN_ROOM = 10.0       # a site keeps this many hole radii from the edges of the triangle it splits


@dataclass
class Site:
    pad: int                          # pad id owning the hole's edges (the same on every layer)
    centre: tuple[float, float]
    obstacle: int                     # obstacle id of the hole
    outer: tuple[int, int, int]       # a_i: corners of the triangle that was split
    outer_edges: tuple[int, int, int] # edge opposite a_i
    verts: tuple[int, int, int]       # p_i
    spokes: list[int]                 # gates from a hole vertex outwards (kept up to date as edges are flipped)
    hole: tuple[int, int, int]        # [i] = p_i p_(i+1)
    tris: tuple[int, ...]             # A_0 (the id of the triangle that was split), A_1, A_2, S_0, S_1, S_2
    net: int = -1                     # -1 = dormant
    keep: float = 0.0                 # centreline keep-off from the hole's vertices while active

    @property
    def active(self) -> bool:
        return self.net >= 0


def keep_off(rules: Rules) -> float:
    """Distance a foreign base-width centreline keeps from a vertex of an active
    site: via copper (as the polygon it is checked as), clearance, half a trace,
    and the hole's own radius, since the vertices are not at the centre."""
    return rules.via_diameter / 2.0 / math.cos(math.pi / 16.0) + rules.clearance + rules.base_width / 2.0 + SITE_RADIUS


def room(pmap: PlanarMap, t: int, point: tuple[float, float]) -> float:
    """Distance from ``point`` to the nearest edge of triangle ``t`` (negative outside it)."""
    pts = [pmap.vxy[v] for v in pmap.tri_v_list[t]]
    best = math.inf
    for i in range(3):
        (ax, ay), (bx, by) = pts[i], pts[(i + 1) % 3]
        best = min(best, ((bx - ax) * (point[1] - ay) - (by - ay) * (point[0] - ax)) / math.hypot(bx - ax, by - ay))
    return best


def incentre(pmap: PlanarMap, t: int) -> tuple[float, float]:
    """The point of triangle ``t`` furthest from its edges."""
    (ax, ay), (bx, by), (cx, cy) = (pmap.vxy[v] for v in pmap.tri_v_list[t])
    la, lb, lc = math.hypot(bx - cx, by - cy), math.hypot(ax - cx, ay - cy), math.hypot(ax - bx, ay - by)
    s = la + lb + lc
    return ((la * ax + lb * bx + lc * cx) / s, (la * ay + lb * by + lc * cy) / s)


def create(pmap: PlanarMap, state: TopoState, t: int, point: tuple[float, float], pad: int | None = None,
           radius: float = SITE_RADIUS) -> Site:
    """Puts a dormant site at ``point`` inside triangle ``t`` and returns it.

    ``state`` is the topological state on this map; its wires through ``t``
    are carried over. Raises ValueError if the point is too close to an edge
    of the triangle to hold the hole.
    """
    site = _create_map(pmap, t, point, pad, radius)
    state.resize()
    for e in site.spokes:
        _set_capacity(pmap, state, e)
    _carry_wires(pmap, state, site)
    legalise(pmap, state, site)
    return site


def _create_map(pmap: PlanarMap, t: int, point: tuple[float, float], pad: int | None, radius: float = SITE_RADIUS) -> Site:
    """The map's side of ``create``: splits the triangle and logs how to undo it."""
    if room(pmap, t, point) < MIN_ROOM * radius:
        raise ValueError("no room for a via site at this point of the triangle")
    a = pmap.tri_v_list[t]
    outer_edges = pmap.tri_e_list[t]
    outer_nbr = [int(n) for n in pmap.tri_n[t]]
    px, py = point

    # Turn the hole so that each corner a_i looks squarely at the hole edge
    # p_i p_(i+1): its outward normal is at phi + 60 + 120 i degrees. Taking the
    # middle of the three wanted angles leaves every corner within 60 degrees
    # of its normal, so all six triangles are the right way up.
    want = [math.atan2(pmap.vxy[v][1] - py, pmap.vxy[v][0] - px) - math.radians(60 + 120 * i) for i, v in enumerate(a)]
    off = [(w - want[0] + math.pi) % (2 * math.pi) - math.pi for w in want]
    phi = want[0] + (min(off) + max(off)) / 2.0
    hole_xy = [(px + radius * math.cos(phi + i * 2 * math.pi / 3), py + radius * math.sin(phi + i * 2 * math.pi / 3))
               for i in range(3)]

    v0, e0, t0 = pmap.num_vertices, pmap.num_edges, pmap.num_triangles
    p = (v0, v0 + 1, v0 + 2)
    spokes = [e0 + i for i in range(6)]
    hole = tuple(e0 + 6 + i for i in range(3))
    big = (t, t0, t0 + 1)            # A_i
    small = (t0 + 2, t0 + 3, t0 + 4)  # S_i
    xy = pmap.vxy + hole_xy

    tri_v, tri_e, tri_n = {}, {}, {}
    for i in range(3):
        j, k = (i + 1) % 3, (i + 2) % 3
        tri_v[big[i]] = (a[i], a[j], p[j])
        tri_e[big[i]] = (spokes[2 * j], spokes[2 * i + 1], outer_edges[k])
        tri_n[big[i]] = (small[j], small[i], outer_nbr[k])
        tri_v[small[i]] = (a[i], p[j], p[i])
        tri_e[small[i]] = (hole[i], spokes[2 * i], spokes[2 * i + 1])
        tri_n[small[i]] = (-1, big[k], big[i])
    for tri in tri_v.values():
        (ax, ay), (bx, by), (cx, cy) = (xy[v] for v in tri)
        if (bx - ax) * (cy - ay) - (by - ay) * (cx - ax) <= 0.0:
            raise ValueError("no room for a via site at this point of the triangle")

    edge_v = [(a[i // 2], p[(i // 2 + i % 2) % 3]) for i in range(6)] + [tuple(sorted((p[i], p[(i + 1) % 3]))) for i in range(3)]
    edge_t = []
    for i in range(3):
        edge_t += [(big[(i + 2) % 3], small[i]), (small[i], big[i])]
    edge_t += [(small[i], -1) for i in range(3)]
    if pad is None:
        pad = max(max(pmap.pad_centre, default=-1), max(pmap.pad_edges, default=-1)) + 1
    obstacle = int(pmap.v_obs.max()) + 1
    lengths = [math.dist(xy[u], xy[v]) for u, v in edge_v]
    mids = [((xy[u][0] + xy[v][0]) / 2.0, (xy[u][1] + xy[v][1]) / 2.0) for u, v in edge_v]
    # Width estimate of a spoke (6.3): its length, or the distance from the
    # site to the corner's whole obstacle if that is nearer.
    geoms = pmap.free_space.obs_geom if pmap.free_space is not None else {}
    here = Point(px, py)
    widths = list(lengths)
    for i in range(6):
        geom = geoms.get(pmap.v_obs_list[a[i // 2]])
        if geom is not None:
            widths[i] = min(widths[i], here.distance(geom) + radius)

    entry = {"kind": "site", "t": t, "point": (px, py), "pad": pad, "radius": radius, "sizes": (v0, e0, t0),
             "row": (pmap.tri_v_list[t], pmap.tri_e_list[t], tuple(outer_nbr), pmap.tri_cen[t]),
             "outer": [(e, pmap.edge_t_list[e]) for e in outer_edges],
             "nbrs": [(n, tuple(int(x) for x in pmap.tri_n[n])) for n in outer_nbr if n >= 0], "claimed": []}

    # -- the map: arrays, then their list mirrors -----------------------------
    order = [big[1], big[2], *small]  # appended triangles, by id
    pmap.tri_v = np.concatenate([pmap.tri_v, np.array([tri_v[x] for x in order], dtype=pmap.tri_v.dtype)])
    pmap.tri_e = np.concatenate([pmap.tri_e, np.array([tri_e[x] for x in order], dtype=pmap.tri_e.dtype)])
    pmap.tri_n = np.concatenate([pmap.tri_n, np.array([tri_n[x] for x in order], dtype=pmap.tri_n.dtype)])
    pmap.tri_v[t], pmap.tri_e[t], pmap.tri_n[t] = tri_v[t], tri_e[t], tri_n[t]
    pmap.vx = np.concatenate([pmap.vx, [q[0] for q in hole_xy]])
    pmap.vy = np.concatenate([pmap.vy, [q[1] for q in hole_xy]])
    pmap.v_obs = np.concatenate([pmap.v_obs, np.full(3, obstacle, dtype=pmap.v_obs.dtype)])
    pmap.edge_v = np.concatenate([pmap.edge_v, np.array(edge_v, dtype=pmap.edge_v.dtype)])
    pmap.edge_t = np.concatenate([pmap.edge_t, np.array(edge_t, dtype=pmap.edge_t.dtype)])
    pmap.edge_len = np.concatenate([pmap.edge_len, lengths])
    pmap.edge_mid = np.concatenate([pmap.edge_mid, np.array(mids)])
    kinds = [GATE] * 6 + [TERMINAL] * 3
    owners = [-1] * 6 + [pad] * 3
    pmap.edge_kind = np.concatenate([pmap.edge_kind, np.array(kinds, dtype=pmap.edge_kind.dtype)])
    pmap.edge_owner = np.concatenate([pmap.edge_owner, np.array(owners, dtype=pmap.edge_owner.dtype)])
    caps = [int(math.floor(w / pmap.pitch + 1e-9)) + 1 for w in widths]
    pmap.edge_cap = np.concatenate([pmap.edge_cap, np.array(caps, dtype=pmap.edge_cap.dtype)])
    if pmap.edge_width is not None:
        pmap.edge_width = np.concatenate([pmap.edge_width, widths])
    # The outer edges and the triangles beyond them now border A_i instead of T.
    for k in range(3):
        new = big[(k + 1) % 3]
        e, n = outer_edges[k], outer_nbr[k]
        pmap.edge_t[e] = [new if x == t else x for x in pmap.edge_t_list[e]]
        pmap.edge_t_list[e] = tuple(int(x) for x in pmap.edge_t[e])
        if n >= 0:
            pmap.tri_n[n] = [new if x == t else x for x in pmap.tri_n[n].tolist()]

    pmap.num_vertices, pmap.num_edges, pmap.num_triangles = v0 + 3, e0 + 9, t0 + 5
    pmap.tri_v_list[t], pmap.tri_e_list[t] = tri_v[t], tri_e[t]
    pmap.tri_v_list += [tri_v[x] for x in order]
    pmap.tri_e_list += [tri_e[x] for x in order]
    pmap.tri_cen[t] = _centroid(xy, tri_v[t])
    pmap.tri_cen += [_centroid(xy, tri_v[x]) for x in order]
    pmap.vxy += hole_xy
    pmap.v_obs_list += [obstacle] * 3
    pmap.edge_v_list += edge_v
    pmap.edge_t_list += edge_t
    pmap.edge_len_list += lengths
    pmap.edge_mid_list += mids
    pmap.edge_kind_list += kinds
    pmap.edge_owner_list += owners
    pmap.edge_cap_list += caps
    pmap.pad_edges[pad] = list(hole)
    pmap.pad_centre[pad] = (px, py)
    pmap.pad_net[pad] = -1
    pmap.pad_radius[pad] = radius
    pmap.pad_obs[pad] = obstacle
    for i in range(3):
        pmap.v_nbr[p[i]] = (p[(i + 2) % 3], p[(i + 1) % 3])
        pmap.vertex_terminals[p[i]] = [hole[(i + 2) % 3], hole[i]]
    if pmap.free_space is not None:
        pmap.free_space.obs_geom[obstacle] = LinearRing(hole_xy)
    touched = [2 * e + side for e in (*outer_edges, *spokes, *hole) for side in (0, 1)]
    pmap.trans += [()] * 18
    for h in touched:
        pmap.trans[h] = pmap.transitions(h)
    _sync_tables(pmap, touched)

    site = Site(pad, (px, py), obstacle, tuple(a), tuple(outer_edges), p, spokes, hole, (*big, *small))
    pmap.sites[pad] = site
    owner = pmap.__dict__.setdefault("_site_of_vertex", {})
    for v in p:
        owner[v] = site
    for e, (u, v) in zip(spokes, edge_v):  # a corner of the triangle may itself belong to an older site
        if u in owner:
            owner[u].spokes.append(e)
            entry["claimed"].append((owner[u].pad, e))
    log(pmap).append(entry)
    return site


def log(pmap: PlanarMap) -> list:
    """Everything done to the map since it was built, in order: one entry per
    site created and per edge flipped, each holding what is needed to undo it
    (``rewind``) or to do it again on another copy of the map (``replay``)."""
    return pmap.__dict__.setdefault("_site_log", [])


def _sync_tables(pmap: PlanarMap, half_edges) -> None:
    tables = pmap.__dict__.get("_kernel_tables")
    if tables is not None and not tables.update(pmap, half_edges):
        del pmap.__dict__["_kernel_tables"]  # outgrown: built again, with room, at the next search


def rewind(pmap: PlanarMap, length: int) -> None:
    """Undoes the map's changes back to when its log had ``length`` entries.
    The wires are not carried back: the caller restores a state of that time."""
    entries = log(pmap)
    owner = pmap.__dict__.get("_site_of_vertex", {})
    cache = pmap.__dict__.get("_wall_need_cache")
    while len(entries) > length:
        entry = entries.pop()
        if entry["kind"] == "flip":
            e = entry["e"]
            for site_pad, added in reversed(entry["spokes"]):
                (pmap.sites[site_pad].spokes.remove if added else pmap.sites[site_pad].spokes.append)(e)
            for t, verts, edges, nbrs, cen in entry["tris"]:
                pmap.tri_v[t], pmap.tri_e[t], pmap.tri_n[t] = verts, edges, nbrs
                pmap.tri_v_list[t], pmap.tri_e_list[t], pmap.tri_cen[t] = verts, edges, cen
            ends, tris, length_, mid, cap, width = entry["edge"]
            pmap.edge_v[e], pmap.edge_t[e], pmap.edge_len[e], pmap.edge_mid[e], pmap.edge_cap[e] = ends, tris, length_, mid, cap
            pmap.edge_v_list[e], pmap.edge_t_list[e], pmap.edge_len_list[e] = ends, tris, length_
            pmap.edge_mid_list[e], pmap.edge_cap_list[e] = mid, cap
            if pmap.edge_width is not None:
                pmap.edge_width[e] = width
            for edge, tris in entry["moved"]:
                pmap.edge_t[edge], pmap.edge_t_list[edge] = tris, tris
            for n, row in entry["nbrs"]:
                pmap.tri_n[n] = row
            touched = [2 * x + side for x in (e, *(m[0] for m in entry["moved"]), *entry["kept"]) for side in (0, 1)]
            if cache:
                for key in [key for key in cache if key[0] == e]:
                    del cache[key]
        else:
            v0, e0, t0 = entry["sizes"]
            t, site = entry["t"], pmap.sites.pop(entry["pad"])
            for site_pad, e in entry["claimed"]:
                pmap.sites[site_pad].spokes.remove(e)
            for name in ("tri_v", "tri_e", "tri_n"):
                setattr(pmap, name, getattr(pmap, name)[:t0])
            for name in ("vx", "vy", "v_obs"):
                setattr(pmap, name, getattr(pmap, name)[:v0])
            for name in ("edge_v", "edge_t", "edge_len", "edge_mid", "edge_kind", "edge_owner", "edge_cap", "edge_width"):
                if getattr(pmap, name) is not None:
                    setattr(pmap, name, getattr(pmap, name)[:e0])
            for name in ("tri_v_list", "tri_e_list", "tri_cen"):
                del getattr(pmap, name)[t0:]
            for name in ("vxy", "v_obs_list"):
                del getattr(pmap, name)[v0:]
            for name in ("edge_v_list", "edge_t_list", "edge_len_list", "edge_mid_list", "edge_kind_list", "edge_owner_list", "edge_cap_list"):
                del getattr(pmap, name)[e0:]
            del pmap.trans[2 * e0:]
            verts, edges, nbrs, cen = entry["row"]
            pmap.tri_v[t], pmap.tri_e[t], pmap.tri_n[t] = verts, edges, nbrs
            pmap.tri_v_list[t], pmap.tri_e_list[t], pmap.tri_cen[t] = verts, edges, cen
            for edge, tris in entry["outer"]:
                pmap.edge_t[edge], pmap.edge_t_list[edge] = tris, tris
            for n, row in entry["nbrs"]:
                pmap.tri_n[n] = row
            pmap.num_vertices, pmap.num_edges, pmap.num_triangles = v0, e0, t0
            for table in (pmap.pad_edges, pmap.pad_centre, pmap.pad_net, pmap.pad_radius, pmap.pad_obs):
                del table[site.pad]
            for v in site.verts:
                del pmap.v_nbr[v], pmap.vertex_terminals[v], owner[v]
            if pmap.free_space is not None:
                del pmap.free_space.obs_geom[site.obstacle]
            touched = [2 * x[0] + side for x in entry["outer"] for side in (0, 1)]
            if cache:
                for key in [key for key in cache if key[0] >= e0]:
                    del cache[key]
        for h in touched:
            pmap.trans[h] = pmap.transitions(h)
        _sync_tables(pmap, touched)


def journal(state: TopoState) -> list:
    """Notes of what site creation changed in the state, so that a site that
    turns out not to fit can be taken out again (``undo``)."""
    return state.__dict__.setdefault("_journal", [])


def undo(pmap: PlanarMap, state: TopoState, log_length: int, journal_length: int) -> set[int]:
    """Takes the map and the state back to a mark (lengths of ``log`` and
    ``journal`` at the time). Returns the wires whose paths were put back."""
    notes = journal(state)
    wires: set[int] = set()
    while len(notes) > journal_length:
        note = notes.pop()
        if note[0] == "wires":
            for w, steps in note[1].items():
                state.wire_path[w] = steps
            wires.update(note[1])
        elif note[0] == "gate":
            _, e, row, load, cap = note
            state.gate_order[e][:] = row
            state.load[e], state.cap[e], state.count[e] = load, cap, len(row)
        else:
            _, t, cnt = note
            state.corner_cnt[t][:] = cnt
            state.corner[t] = cnt
    rewind(pmap, log_length)
    state.resize()
    pmap.__dict__.pop("_moved_wires", None)
    return wires


def replay(pmap: PlanarMap, entries: list) -> None:
    """Does the logged changes again on this map (a copy that has not had
    them), without wires: the caller restores a state that already has them."""
    for entry in entries:
        if entry["kind"] == "site":
            _create_map(pmap, entry["t"], entry["point"], entry["pad"], entry["radius"])
        else:
            _flip_map(pmap, _quad(pmap, entry["e"]))


def _centroid(xy, tri) -> tuple[float, float]:
    return (sum(xy[v][0] for v in tri) / 3.0, sum(xy[v][1] for v in tri) / 3.0)


def _carry_wires(pmap: PlanarMap, state: TopoState, site: Site) -> None:
    """Moves the wires that crossed the split triangle onto the new triangles.
    A wire that cut corner a_i crosses the two spokes at a_i, in the same
    nesting order; its place on every other gate is untouched."""
    t = site.tris[0]
    big, small = site.tris[:3], site.tris[3:]
    order, a, outer = state.gate_order, site.outer, site.outer_edges
    old = list(state.corner_cnt[t])
    notes = state.__dict__.get("_journal")
    if notes is not None:
        notes.append(("cnt", t, old))
    nested = []
    for i in range(3):
        along = order[outer[(i + 2) % 3]]  # the outer edge a_i a_(i+1), which every wire cutting a_i crosses
        nested.append(along[:old[i]] if pmap.edge_v_list[outer[(i + 2) % 3]][0] == a[i] else along[::-1][:old[i]])
    for i in range(3):
        j, k = (i + 1) % 3, (i + 2) % 3
        first, second = site.spokes[2 * i], site.spokes[2 * i + 1]
        for e in (first, second):  # a_i is the u end of both spokes: nearest a_i first
            order[e][:] = nested[i]
            state.load[e] = sum(state.weight[w] for w in nested[i])
            state.count[e] = len(nested[i])
        state.corner_cnt[big[i]][:] = [old[i], old[j], 0]
        state.corner[big[i]] = state.corner_cnt[big[i]]
        state.corner_cnt[small[i]][:] = [old[i], 0, 0]
        state.corner[small[i]] = state.corner_cnt[small[i]]
        pmap.__dict__.setdefault("_moved_wires", set()).update(nested[i])
        if notes is not None:
            notes.append(("wires", {w: list(state.wire_path[w]) for w in nested[i]}))
        for pos, w in enumerate(nested[i]):
            steps = state.wire_path[w]
            m = next(n for n, s in enumerate(steps) if s[1] == t)
            leave, slot = steps[m][0], steps[m][3]
            if leave == outer[k]:   # came in over a_(i-1) a_i, goes out over a_i a_(i+1)
                steps[m:m + 1] = [(first, big[k], 1, pos), (second, small[i], 0, pos), (leave, big[i], 0, slot)]
            else:
                steps[m:m + 1] = [(second, big[i], 0, pos), (first, small[i], 0, pos), (leave, big[k], 1, slot)]


def _apex(pmap: PlanarMap, t: int, e: int) -> int:
    """Local index in triangle ``t`` of the vertex opposite edge ``e``."""
    return pmap.tri_e_list[t].index(e)


def _cross(o, a, b) -> float:
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def _in_circle(a, b, c, d) -> bool:
    """Whether d lies inside the circle through a, b, c (counter-clockwise)."""
    ax, ay, bx, by, cx, cy = a[0] - d[0], a[1] - d[1], b[0] - d[0], b[1] - d[1], c[0] - d[0], c[1] - d[1]
    det = ((ax * ax + ay * ay) * (bx * cy - cx * by) - (bx * bx + by * by) * (ax * cy - cx * ay)
           + (cx * cx + cy * cy) * (ax * by - bx * ay))
    return det > 1e-12 * max(1.0, abs(ax * by - bx * ay)) ** 2


def _quad(pmap: PlanarMap, e: int):
    """The quadrilateral b p d q round gate ``e`` = p q, with its outer edges and
    the triangles beyond them, or None if the edge cannot be flipped."""
    if pmap.edge_kind_list[e] != GATE:
        return None
    t1, t2 = pmap.edge_t_list[e]
    if t1 < 0 or t2 < 0:
        return None
    i1, i2 = _apex(pmap, t1, e), _apex(pmap, t2, e)
    v1, v2 = pmap.tri_v_list[t1], pmap.tri_v_list[t2]
    b, p, q, d = v1[i1], v1[(i1 + 1) % 3], v1[(i1 + 2) % 3], v2[i2]
    xy = pmap.vxy
    if _cross(xy[b], xy[p], xy[d]) <= 1e-12 or _cross(xy[b], xy[d], xy[q]) <= 1e-12:
        return None  # not convex
    e_bp, e_qb = pmap.tri_e_list[t1][(i1 + 2) % 3], pmap.tri_e_list[t1][(i1 + 1) % 3]
    e_pd, e_dq = pmap.tri_e_list[t2][(i2 + 1) % 3], pmap.tri_e_list[t2][(i2 + 2) % 3]
    n_bp, n_qb = int(pmap.tri_n[t1][(i1 + 2) % 3]), int(pmap.tri_n[t1][(i1 + 1) % 3])
    n_pd, n_dq = int(pmap.tri_n[t2][(i2 + 1) % 3]), int(pmap.tri_n[t2][(i2 + 2) % 3])
    return e, t1, t2, i1, i2, (b, p, q, d), (e_bp, e_qb, e_pd, e_dq), (n_bp, n_qb, n_pd, n_dq)


def _flip_map(pmap: PlanarMap, quad) -> None:
    """The map's side of ``flip``; logs how to undo it."""
    e, t1, t2, _, _, (b, p, q, d), (e_bp, e_qb, e_pd, e_dq), (n_bp, n_qb, n_pd, n_dq) = quad
    xy = pmap.vxy
    entry = {"kind": "flip", "e": e,
             "tris": [(t, pmap.tri_v_list[t], pmap.tri_e_list[t], tuple(int(x) for x in pmap.tri_n[t]), pmap.tri_cen[t]) for t in (t1, t2)],
             "edge": (pmap.edge_v_list[e], pmap.edge_t_list[e], pmap.edge_len_list[e], pmap.edge_mid_list[e], pmap.edge_cap_list[e],
                      float(pmap.edge_width[e]) if pmap.edge_width is not None else 0.0),
             "moved": [(edge, pmap.edge_t_list[edge]) for edge in (e_pd, e_qb)], "kept": (e_bp, e_dq),
             "nbrs": [(n, tuple(int(x) for x in pmap.tri_n[n])) for n in (n_pd, n_qb) if n >= 0], "spokes": []}
    tri_v = {t1: (b, p, d), t2: (b, d, q)}
    tri_e = {t1: (e_pd, e, e_bp), t2: (e_dq, e_qb, e)}
    tri_n = {t1: (n_pd, t2, n_bp), t2: (n_dq, n_qb, t1)}
    for t in (t1, t2):
        pmap.tri_v[t], pmap.tri_e[t], pmap.tri_n[t] = tri_v[t], tri_e[t], tri_n[t]
        pmap.tri_v_list[t], pmap.tri_e_list[t] = tri_v[t], tri_e[t]
        pmap.tri_cen[t] = _centroid(xy, tri_v[t])
    for edge, nbr, was, now in ((e_pd, n_pd, t2, t1), (e_qb, n_qb, t1, t2)):
        pmap.edge_t[edge] = [now if x == was else x for x in pmap.edge_t_list[edge]]
        pmap.edge_t_list[edge] = tuple(int(x) for x in pmap.edge_t[edge])
        if nbr >= 0:
            pmap.tri_n[nbr] = [now if x == was else x for x in pmap.tri_n[nbr].tolist()]
    u, v = min(b, d), max(b, d)
    length = math.dist(xy[b], xy[d])
    mid = ((xy[b][0] + xy[d][0]) / 2.0, (xy[b][1] + xy[d][1]) / 2.0)
    width = _width(pmap, u, v, length)
    pmap.edge_v[e], pmap.edge_t[e], pmap.edge_len[e], pmap.edge_mid[e] = (u, v), (t1, t2), length, mid
    pmap.edge_v_list[e], pmap.edge_t_list[e], pmap.edge_len_list[e], pmap.edge_mid_list[e] = (u, v), (t1, t2), length, mid
    pmap.edge_cap[e] = pmap.edge_cap_list[e] = int(math.floor(width / pmap.pitch + 1e-9)) + 1
    if pmap.edge_width is not None:
        pmap.edge_width[e] = width
    touched = [2 * x + side for x in (e, e_bp, e_qb, e_pd, e_dq) for side in (0, 1)]
    for h in touched:
        pmap.trans[h] = pmap.transitions(h)
    _sync_tables(pmap, touched)
    cache = pmap.__dict__.get("_wall_need_cache")
    if cache:
        for key in [key for key in cache if key[0] == e]:
            del cache[key]
    owner = pmap.__dict__.get("_site_of_vertex", {})
    for x in (p, q):
        if x in owner:
            owner[x].spokes.remove(e)
            entry["spokes"].append((owner[x].pad, False))
    for x in (b, d):
        if x in owner:
            owner[x].spokes.append(e)
            entry["spokes"].append((owner[x].pad, True))
    log(pmap).append(entry)


def flip(pmap: PlanarMap, state: TopoState, e: int) -> bool:
    """Replaces gate ``e``, the diagonal p q of the quadrilateral b p d q made by
    its two triangles, by the other diagonal b d, keeping its edge id. Every
    wire keeps the edges of the quadrilateral it used, so no wire's relation to
    another changes. Returns False, changing nothing, if the edge is not a gate,
    the quadrilateral is not convex, or a wire would cross the new gate twice.
    """
    quad = _quad(pmap, e)
    if quad is None:
        return False
    _, t1, t2, i1, i2, (b, p, q, d), (e_bp, e_qb, e_pd, e_dq), _ = quad

    # The wires, by which corner they cut (7.1). Along the old gate from p,
    # those cutting p come first, in each triangle.
    c1, c2, order = state.corner_cnt[t1], state.corner_cnt[t2], state.gate_order
    c1b, c1p = c1[i1], c1[(i1 + 1) % 3]
    c2d, c2p = c2[i2], c2[(i2 + 2) % 3]
    along = order[e] if pmap.edge_v_list[e][0] == p else order[e][::-1]
    n = len(along)
    lo, hi = min(c1p, c2p), max(c1p, c2p)
    round_b, round_d = state._near(e_bp, b, c1b), state._near(e_dq, d, c2d)
    # Wires that went straight through: b p to d q, or q b to p d. From b they
    # lie in the order they have on their b-side edge.
    through = along[lo:hi][::-1] if c1p > c2p else along[lo:hi]
    new_row = round_b + through + round_d[::-1]  # from b to d
    if len(set(new_row)) != len(new_row) or len(new_row) >= 15:
        return False  # a wire passing both b and d would cross the new gate twice; or no slots left
    bp_dq, qb_pd = (hi - lo, 0) if c1p > c2p else (0, hi - lo)
    wires = set(new_row) | set(along)
    for edge in (e_bp, e_qb, e_pd, e_dq):
        wires.update(order[edge])

    notes = state.__dict__.get("_journal")
    if notes is not None:
        notes.append(("wires", {w: list(state.wire_path[w]) for w in wires}))
        notes.append(("gate", e, list(order[e]), float(state.load[e]), float(state.cap[e])))
        notes.append(("cnt", t1, list(c1)))
        notes.append(("cnt", t2, list(c2)))
    _flip_map(pmap, quad)

    tri_v = {t1: (b, p, d), t2: (b, d, q)}
    home = {e_bp: t1, e_pd: t1, e_dq: t2, e_qb: t2}
    row = new_row if b < d else new_row[::-1]
    for w in wires:
        steps, out, m = state.wire_path[w], [], 0
        while m < len(steps):
            if steps[m][1] not in (t1, t2):
                out.append(steps[m])
                m += 1
                continue
            came = steps[m - 1][0]
            while m + 1 < len(steps) and steps[m + 1][1] in (t1, t2) and steps[m][0] == e:
                m += 1
            leave, slot = steps[m][0], steps[m][3]
            ta, tb = home[came], home[leave]
            if ta == tb:
                out.append((leave, ta, tri_v[ta].index(pmap.shared_vertex(came, leave)), slot))
            else:
                out.append((e, ta, tri_v[ta].index(pmap.shared_vertex(came, e)), row.index(w)))
                out.append((leave, tb, tri_v[tb].index(pmap.shared_vertex(e, leave)), slot))
            m += 1
        steps[:] = out
    pmap.__dict__.setdefault("_moved_wires", set()).update(wires)
    order[e][:] = row
    state.load[e] = sum(state.weight[w] for w in row)
    state.count[e] = len(row)
    _set_capacity(pmap, state, e)
    state.corner_cnt[t1][:] = [c1b + bp_dq, lo, c2d + qb_pd]
    state.corner_cnt[t2][:] = [c1b + qb_pd, c2d + bp_dq, n - hi]
    state.corner[t1], state.corner[t2] = state.corner_cnt[t1], state.corner_cnt[t2]
    return True


def _width(pmap: PlanarMap, u: int, v: int, length: float) -> float:
    """Width estimate of a gate between vertices u and v (6.3)."""
    geoms = pmap.free_space.obs_geom if pmap.free_space is not None else {}
    ou, ov = pmap.v_obs_list[u], pmap.v_obs_list[v]
    width = length
    if ou != ov and ou in geoms and ov in geoms:
        width = min(width, Point(pmap.vxy[u]).distance(geoms[ov]), Point(pmap.vxy[v]).distance(geoms[ou]))
    return width


def legalise(pmap: PlanarMap, state: TopoState, site: Site) -> int:
    """Flips the edges facing the site until each is Delaunay (or cannot be
    flipped: a wall, a pad edge, a quadrilateral that is not convex). Returns
    the number of flips. Every flip gives the site one more spoke."""
    mine = set(site.verts)
    xy = pmap.vxy
    pending = []
    for e in site.spokes:
        for t in pmap.edge_t_list[e]:
            if t >= 0:
                pending += [x for x in pmap.tri_e_list[t] if not mine.intersection(pmap.edge_v_list[x])]
    flips = 0
    while pending and flips < 10_000:
        e = pending.pop()
        if pmap.edge_kind_list[e] != GATE:
            continue
        t1, t2 = pmap.edge_t_list[e]
        if t1 < 0 or t2 < 0:
            continue
        if pmap.tri_v_list[t2][_apex(pmap, t2, e)] in mine:
            t1, t2 = t2, t1
        top = pmap.tri_v_list[t1][_apex(pmap, t1, e)]
        if top not in mine:
            continue  # no longer faces the site
        far = pmap.tri_v_list[t2][_apex(pmap, t2, e)]
        if far in mine or not _in_circle(*(xy[v] for v in pmap.tri_v_list[t1]), xy[far]):
            continue
        if not flip(pmap, state, e):
            continue
        flips += 1
        for t in pmap.edge_t_list[e]:
            pending += [x for x in pmap.tri_e_list[t] if not mine.intersection(pmap.edge_v_list[x])]
    return flips


def set_net(pmap: PlanarMap, state: TopoState, site: Site, net: int, keep: float = 0.0) -> None:
    """Makes the site a via of ``net`` keeping foreign centrelines ``keep`` away
    (``keep_off``), or puts it back to sleep with net -1. The capacity of its
    spokes follows: the rule of 6.3 with the keep-off taken off the width."""
    if net < 0:
        if any(state.gate_order[e] for e in site.hole):
            raise ValueError("a via with traces on it cannot be put to sleep")
        keep = 0.0
    site.net, site.keep = net, keep
    pmap.pad_net[site.pad] = net
    for e in site.spokes:
        _set_capacity(pmap, state, e)


def _set_capacity(pmap: PlanarMap, state: TopoState, e: int) -> None:
    """Capacity of a gate from its width estimate (6.3), less the keep-off of
    an active site at either end. A trace of the via's own net needs no room
    beyond that keep-off (it runs into the via), so it is not counted against
    the gate: its weight is added back."""
    owner = pmap.__dict__.get("_site_of_vertex", {})
    keep, own = 0.0, 0.0
    for v in pmap.edge_v_list[e]:
        site = owner.get(v)
        if site is None:
            continue
        keep += site.keep if site.active else asleep_keep(pmap)
        if site.active:
            for w in state.gate_order[e]:
                if site.pad in wire_pads(pmap, state, w):
                    own += state.weight[w]
    state.cap[e] = (pmap.edge_width[e] - keep) / pmap.pitch + 1.0 + own


def wire_pads(pmap: PlanarMap, state: TopoState, w: int) -> tuple[int, int]:
    """The two pads a wire joins."""
    steps = state.wire_path[w]
    return pmap.edge_owner_list[steps[0][0]], pmap.edge_owner_list[steps[-1][0]]


def refresh(pmap: PlanarMap, state: TopoState, pad: int) -> None:
    """Recomputes the capacity of a site's spokes (a trace of its own has come or gone)."""
    site = pmap.sites.get(pad)
    if site is not None:
        for e in site.spokes:
            _set_capacity(pmap, state, e)


def fits(pmap: PlanarMap, state: TopoState, site: Site, keep: float, spare: float = 0.0) -> bool:
    """Whether the site can be made active with this keep-off without putting
    any of its spokes over capacity (or within ``spare`` pitches of it, where wires cross)."""
    was = site.net, site.keep
    site.net, site.keep = max(site.net, 0), keep  # as if it were awake, with no trace of its own yet
    try:
        for e in site.spokes:
            before = state.cap[e]
            _set_capacity(pmap, state, e)
            ok = state.load[e] <= state.cap[e] - (spare if state.load[e] > 0 else 0.0) + 1e-9
            state.cap[e] = before
            if not ok:
                return False
        return True
    finally:
        site.net, site.keep = was


def asleep_keep(pmap: PlanarMap) -> float:
    """What a sleeping site keeps wires away from its hole. It is a free point,
    with wires passing on both sides (no other vertex of the map has that): half
    a pitch each way keeps the two sides a pitch apart."""
    return pmap.pitch / 2.0 + SITE_RADIUS


def keep_at(pmap: PlanarMap) -> dict[int, tuple[float, int]]:
    """Vertex -> (keep-off, net) for the vertices of the sites (net -1 for one that is asleep)."""
    quiet = asleep_keep(pmap)
    return {v: (site.keep if site.active else quiet, site.net) for site in pmap.sites.values() for v in site.verts}


def via_pads(pmap: PlanarMap, rules: Rules) -> list[Pad]:
    """The active sites as via pads, for the design-rule check and for output."""
    return [Pad.circle(site.pad, site.centre[0], site.centre[1], rules.via_diameter / 2.0, site.net,
                       name=f"via{site.pad}", is_via=True)
            for site in pmap.sites.values() if site.active]
