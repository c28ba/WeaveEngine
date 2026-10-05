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
from weaveengine.topo.planar_map import GATE, TERMINAL, WALL_EDGE, PlanarMap
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
    for e in site.hole:
        state.cap[e] = pmap.edge_width[e] / pmap.pitch + 1.0
    _carry_wires(pmap, state, site)
    legalise(pmap, state, site)
    return site


FAR = 1.0e7  # mm: where the vertices of unused slots are parked, away from every board


def free_slots(pmap: PlanarMap) -> list:
    """Slots of the map's tables that hold no site at present: (vertices,
    edges, triangles) = 3, 9 and 5 ids. A site that is deleted leaves its slot
    here and the next site made takes it, so the tables do not grow without
    end and no id of anything else ever changes."""
    return pmap.__dict__.setdefault("_free_slots", [])


def _slot(pmap: PlanarMap):
    """A slot for a new site: a free one, or new rows at the end of every table."""
    free = free_slots(pmap)
    if free:
        return free.pop()
    v0, e0, t0 = pmap.num_vertices, pmap.num_edges, pmap.num_triangles
    slot = (tuple(range(v0, v0 + 3)), tuple(range(e0, e0 + 9)), tuple(range(t0, t0 + 5)))

    def more(a, n):
        return np.concatenate([a, np.zeros((n,) + a.shape[1:], dtype=a.dtype)])

    pmap.tri_v, pmap.tri_e, pmap.tri_n = more(pmap.tri_v, 5), more(pmap.tri_e, 5), more(pmap.tri_n, 5)
    pmap.vx, pmap.vy, pmap.v_obs = more(pmap.vx, 3), more(pmap.vy, 3), more(pmap.v_obs, 3)
    for name in ("edge_v", "edge_t", "edge_len", "edge_mid", "edge_kind", "edge_owner", "edge_cap", "edge_width"):
        if getattr(pmap, name) is not None:
            setattr(pmap, name, more(getattr(pmap, name), 9))
    pmap.tri_v_list += [(0, 0, 0)] * 5
    pmap.tri_e_list += [(0, 0, 0)] * 5
    pmap.tri_cen += [(0.0, 0.0)] * 5
    pmap.vxy += [(0.0, 0.0)] * 3
    pmap.v_obs_list += [-1] * 3
    pmap.edge_v_list += [(0, 0)] * 9
    pmap.edge_t_list += [(-1, -1)] * 9
    pmap.edge_len_list += [0.0] * 9
    pmap.edge_mid_list += [(0.0, 0.0)] * 9
    pmap.edge_kind_list += [WALL_EDGE] * 9
    pmap.edge_owner_list += [-1] * 9
    pmap.edge_cap_list += [0] * 9
    pmap.trans += [()] * 18
    pmap.num_vertices, pmap.num_edges, pmap.num_triangles = v0 + 3, e0 + 9, t0 + 5
    _park(pmap, slot)
    return slot


def _park(pmap: PlanarMap, slot) -> None:
    """Fills a slot's rows with a small triangle far from the board, joined to nothing."""
    verts, edges, tris = slot
    xy = [(FAR + 4.0 * verts[0], FAR), (FAR + 4.0 * verts[0] + 1.0, FAR), (FAR + 4.0 * verts[0], FAR + 1.0)]
    for v, q in zip(verts, xy):
        pmap.vx[v], pmap.vy[v], pmap.v_obs[v] = q[0], q[1], -1
        pmap.vxy[v], pmap.v_obs_list[v] = q, -1
    ends = [(verts[1], verts[2]), (verts[0], verts[2]), (verts[0], verts[1])] + [(verts[0], verts[1])] * 6
    for e, (u, v) in zip(edges, ends):
        _write_edge(pmap, e, (u, v), (-1, -1), WALL_EDGE, -1, 0.0)
    for t in tris:
        _write_triangle(pmap, t, verts, edges[:3], (-1, -1, -1))


def _write_triangle(pmap: PlanarMap, t: int, verts, edges, nbrs) -> None:
    verts, edges, nbrs = tuple(int(x) for x in verts), tuple(int(x) for x in edges), tuple(int(x) for x in nbrs)
    pmap.tri_v[t], pmap.tri_e[t], pmap.tri_n[t] = verts, edges, nbrs
    pmap.tri_v_list[t], pmap.tri_e_list[t] = verts, edges
    pmap.tri_cen[t] = _centroid(pmap.vxy, verts)


def _write_edge(pmap: PlanarMap, e: int, ends, tris, kind: int, owner: int, width: float | None = None) -> None:
    """Sets every table of edge ``e``. ``width`` None = estimate it (6.3)."""
    u, v = int(min(ends)), int(max(ends))
    length = math.dist(pmap.vxy[u], pmap.vxy[v])
    mid = ((pmap.vxy[u][0] + pmap.vxy[v][0]) / 2.0, (pmap.vxy[u][1] + pmap.vxy[v][1]) / 2.0)
    if width is None:
        width = _width(pmap, u, v, length)
    cap = 0 if kind == WALL_EDGE else int(math.floor(width / pmap.pitch + 1e-9)) + 1
    tris = (int(tris[0]), int(tris[1]))
    pmap.edge_v[e], pmap.edge_t[e], pmap.edge_len[e], pmap.edge_mid[e] = (u, v), tris, length, mid
    pmap.edge_kind[e], pmap.edge_owner[e], pmap.edge_cap[e] = kind, owner, cap
    pmap.edge_v_list[e], pmap.edge_t_list[e], pmap.edge_len_list[e], pmap.edge_mid_list[e] = (u, v), tris, length, mid
    pmap.edge_kind_list[e], pmap.edge_owner_list[e], pmap.edge_cap_list[e] = kind, owner, cap
    if pmap.edge_width is not None:
        pmap.edge_width[e] = width


def _edge_row(pmap: PlanarMap, e: int):
    return (pmap.edge_v_list[e], pmap.edge_t_list[e], pmap.edge_kind_list[e], pmap.edge_owner_list[e],
            float(pmap.edge_width[e]) if pmap.edge_width is not None else None)


def _triangle_row(pmap: PlanarMap, t: int):
    return (pmap.tri_v_list[t], pmap.tri_e_list[t], tuple(int(x) for x in pmap.tri_n[t]))


def _repoint(pmap: PlanarMap, edge: int, nbr: int, was: int, now: int) -> None:
    """Edge ``edge`` and the triangle ``nbr`` beyond it now border triangle ``now`` instead of ``was``."""
    tris = tuple(now if x == was else x for x in pmap.edge_t_list[edge])
    pmap.edge_t[edge], pmap.edge_t_list[edge] = tris, tris
    if nbr >= 0:
        pmap.tri_n[nbr] = [now if x == was else x for x in pmap.tri_n[nbr].tolist()]


def _refresh(pmap: PlanarMap, edges) -> None:
    """Transitions (and the compiled search's copy) of both sides of these edges."""
    touched = [2 * e + side for e in edges for side in (0, 1)]
    for h in touched:
        pmap.trans[h] = pmap.transitions(h)
    _sync_tables(pmap, touched)
    cache = pmap.__dict__.get("_wall_need_cache")
    if cache:
        gone = set(edges)
        for key in [key for key in cache if key[0] in gone]:
            del cache[key]


def _register(pmap: PlanarMap, site: Site, hole_xy) -> None:
    """Enters a site in the map's tables of pads, obstacles and site vertices."""
    p, hole = site.verts, site.hole
    pmap.sites[site.pad] = site
    pmap.pad_edges[site.pad] = list(hole)
    pmap.pad_centre[site.pad] = site.centre
    pmap.pad_net[site.pad] = site.net
    pmap.pad_radius[site.pad] = math.dist(site.centre, hole_xy[0])
    pmap.pad_obs[site.pad] = site.obstacle
    owner = pmap.__dict__.setdefault("_site_of_vertex", {})
    for i in range(3):
        pmap.v_nbr[p[i]] = (p[(i + 2) % 3], p[(i + 1) % 3])
        pmap.vertex_terminals[p[i]] = [hole[(i + 2) % 3], hole[i]]
        owner[p[i]] = site
    if pmap.free_space is not None:
        pmap.free_space.obs_geom[site.obstacle] = LinearRing(hole_xy)


def _unregister(pmap: PlanarMap, site: Site) -> None:
    del pmap.sites[site.pad]
    for table in (pmap.pad_edges, pmap.pad_centre, pmap.pad_net, pmap.pad_radius, pmap.pad_obs):
        del table[site.pad]
    owner = pmap.__dict__["_site_of_vertex"]
    for v in site.verts:
        del pmap.v_nbr[v], pmap.vertex_terminals[v], owner[v]
    if pmap.free_space is not None:
        pmap.free_space.obs_geom.pop(site.obstacle, None)


def _create_map(pmap: PlanarMap, t: int, point: tuple[float, float], pad: int | None, radius: float = SITE_RADIUS) -> Site:
    """The map's side of ``create``: splits the triangle and logs how to undo it."""
    if pmap.vxy[pmap.tri_v_list[t][0]][0] >= FAR:
        raise ValueError("not a triangle of the map (an unused slot)")
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
    corners = [pmap.vxy[v] for v in a]
    for i in range(3):
        j = (i + 1) % 3
        if _cross(corners[i], corners[j], hole_xy[j]) <= 0.0 or _cross(corners[i], hole_xy[j], hole_xy[i]) <= 0.0:
            raise ValueError("no room for a via site at this point of the triangle")

    entry = {"kind": "site", "t": t, "point": (px, py), "pad": pad, "radius": radius, "row": _triangle_row(pmap, t),
             "outer": [(e, pmap.edge_t_list[e]) for e in outer_edges],
             "nbrs": [(n, tuple(int(x) for x in pmap.tri_n[n])) for n in outer_nbr if n >= 0], "claimed": []}
    slot = entry["slot"] = _slot(pmap)
    p, edges, tris = slot
    spokes, hole = list(edges[:6]), tuple(edges[6:])
    big, small = (t, tris[0], tris[1]), tuple(tris[2:])
    if pad is None:
        pad = entry["pad"] = max(max(pmap.pad_centre, default=-1), max(pmap.pad_edges, default=-1)) + 1
    obstacle = int(pmap.v_obs.max()) + 1
    for v, q in zip(p, hole_xy):
        pmap.vx[v], pmap.vy[v], pmap.v_obs[v] = q[0], q[1], obstacle
        pmap.vxy[v], pmap.v_obs_list[v] = q, obstacle
    here = Point(px, py)
    geoms = pmap.free_space.obs_geom if pmap.free_space is not None else {}
    for i in range(3):
        j, k = (i + 1) % 3, (i + 2) % 3
        # Width estimate of a spoke (6.3): its length, or the distance from the
        # site to the corner's whole obstacle if that is nearer.
        geom = geoms.get(pmap.v_obs_list[a[i]])
        for e, top, tris_of in ((spokes[2 * i], p[i], (big[k], small[i])), (spokes[2 * i + 1], p[j], (small[i], big[i]))):
            width = math.dist(pmap.vxy[a[i]], pmap.vxy[top])
            if geom is not None:
                width = min(width, here.distance(geom) + radius)
            _write_edge(pmap, e, (a[i], top), tris_of, GATE, -1, width)
        _write_edge(pmap, hole[i], (p[i], p[j]), (small[i], -1), TERMINAL, pad)
        _write_triangle(pmap, big[i], (a[i], a[j], p[j]), (spokes[2 * j], spokes[2 * i + 1], outer_edges[k]), (small[j], small[i], outer_nbr[k]))
        _write_triangle(pmap, small[i], (a[i], p[j], p[i]), (hole[i], spokes[2 * i], spokes[2 * i + 1]), (-1, big[k], big[i]))
    # The outer edges and the triangles beyond them now border A_i instead of T.
    for k in range(3):
        if big[(k + 1) % 3] != t:
            _repoint(pmap, outer_edges[k], outer_nbr[k], t, big[(k + 1) % 3])

    site = Site(pad, (px, py), obstacle, tuple(a), tuple(outer_edges), p, spokes, hole, (*big, *small))
    _register(pmap, site, hole_xy)
    owner = pmap.__dict__["_site_of_vertex"]
    for i in range(3):  # a corner of the triangle may itself belong to an older site
        if a[i] in owner:
            for e in (spokes[2 * i], spokes[2 * i + 1]):
                owner[a[i]].spokes.append(e)
                entry["claimed"].append((owner[a[i]].pad, e))
    _refresh(pmap, (*outer_edges, *edges))
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
    The wires are not carried back: the caller restores a state of that time.
    The tables keep their size; what is undone leaves its slot free."""
    entries = log(pmap)
    while len(entries) > length:
        entry = entries.pop()
        if entry["kind"] == "flip":
            e = entry["e"]
            for site_pad, added in reversed(entry["spokes"]):
                (pmap.sites[site_pad].spokes.remove if added else pmap.sites[site_pad].spokes.append)(e)
            for t, row in entry["tris"]:
                _write_triangle(pmap, t, *row)
            _write_edge(pmap, e, *entry["edge"])
            for edge, tris in entry["moved"]:
                pmap.edge_t[edge], pmap.edge_t_list[edge] = tris, tris
            for n, row in entry["nbrs"]:
                pmap.tri_n[n] = row
            _refresh(pmap, (e, *(m[0] for m in entry["moved"]), *entry["kept"]))
        elif entry["kind"] == "site":
            site = pmap.sites[entry["pad"]]
            for site_pad, e in entry["claimed"]:
                pmap.sites[site_pad].spokes.remove(e)
            _unregister(pmap, site)
            _park(pmap, entry["slot"])
            _write_triangle(pmap, entry["t"], *entry["row"])
            for edge, tris in entry["outer"]:
                pmap.edge_t[edge], pmap.edge_t_list[edge] = tris, tris
            for n, row in entry["nbrs"]:
                pmap.tri_n[n] = row
            free_slots(pmap).append(entry["slot"])
            _refresh(pmap, (*(x[0] for x in entry["outer"]), *entry["slot"][1]))
        else:  # a site was deleted: put it back
            slot = free_slots(pmap).pop()
            assert slot == entry["slot"], "the map's log and its free slots disagree"
            site = entry["site"]
            for v, q in zip(site.verts, entry["hole_xy"]):
                pmap.vx[v], pmap.vy[v], pmap.v_obs[v] = q[0], q[1], site.obstacle
                pmap.vxy[v], pmap.v_obs_list[v] = q, site.obstacle
            for e, row in entry["edges"]:
                _write_edge(pmap, e, *row)
            for t, row in entry["tris"]:
                _write_triangle(pmap, t, *row)
            for edge, tris in entry["outer"]:
                pmap.edge_t[edge], pmap.edge_t_list[edge] = tris, tris
            for n, row in entry["nbrs"]:
                pmap.tri_n[n] = row
            for site_pad, e in entry["gained"]:
                pmap.sites[site_pad].spokes.remove(e)
            site.spokes[:] = entry["spokes"]
            _register(pmap, site, entry["hole_xy"])
            for site_pad, e in entry["claimed"]:
                pmap.sites[site_pad].spokes.append(e)
            _refresh(pmap, (*(x[0] for x in entry["outer"]), *(e for e, _ in entry["edges"])))


def journal(state: TopoState) -> list:
    """Notes of what changes to the map did to the state, so that a site that
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
    clear_free(pmap, state)
    pmap.__dict__.pop("_moved_wires", None)
    return wires


def clear_free(pmap: PlanarMap, state: TopoState) -> None:
    """Nothing is routed through a slot that holds no site."""
    for _, edges, tris in free_slots(pmap):
        for e in edges:
            state.gate_order[e].clear()
            state.load[e] = state.cap[e] = 0.0
            state.count[e] = 0
        for t in tris:
            state.corner_cnt[t][:] = [0, 0, 0]
            state.corner[t] = 0


def replay(pmap: PlanarMap, entries: list) -> None:
    """Does the logged changes again on this map (a copy that has not had
    them), without wires: the caller restores a state that already has them."""
    for entry in entries:
        if entry["kind"] == "site":
            _create_map(pmap, entry["t"], entry["point"], entry["pad"], entry["radius"])
        elif entry["kind"] == "flip":
            _flip_map(pmap, _quad(pmap, entry["e"]))
        else:
            _delete_map(pmap, pmap.sites[entry["site"].pad])


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
        for e in (first, second):  # nearest a_i first, whichever end of the spoke a_i is
            order[e][:] = nested[i] if pmap.edge_v_list[e][0] == a[i] else nested[i][::-1]
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
    entry = {"kind": "flip", "e": e, "tris": [(t, _triangle_row(pmap, t)) for t in (t1, t2)], "edge": _edge_row(pmap, e),
             "moved": [(edge, pmap.edge_t_list[edge]) for edge in (e_pd, e_qb)], "kept": (e_bp, e_dq),
             "nbrs": [(n, tuple(int(x) for x in pmap.tri_n[n])) for n in (n_pd, n_qb) if n >= 0], "spokes": []}
    _write_triangle(pmap, t1, (b, p, d), (e_pd, e, e_bp), (n_pd, t2, n_bp))
    _write_triangle(pmap, t2, (b, d, q), (e_dq, e_qb, e), (n_dq, n_qb, t1))
    _repoint(pmap, e_pd, n_pd, t2, t1)
    _repoint(pmap, e_qb, n_qb, t1, t2)
    _write_edge(pmap, e, (b, d), (t1, t2), GATE, -1)
    owner = pmap.__dict__.get("_site_of_vertex", {})
    for x in (p, q):
        if x in owner:
            owner[x].spokes.remove(e)
            entry["spokes"].append((owner[x].pad, False))
    for x in (b, d):
        if x in owner:
            owner[x].spokes.append(e)
            entry["spokes"].append((owner[x].pad, True))
    _refresh(pmap, (e, e_bp, e_qb, e_pd, e_dq))
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


def _ring(pmap: PlanarMap, site: Site):
    """The polygon round a site: its outer vertices counter-clockwise, and for
    each the edge to the next one with the triangle beyond it and the triangle
    of the site's surroundings it belongs to. Also those surroundings: every
    triangle that touches the hole. None if they do not form a ring."""
    mine = set(site.verts)
    round_hole = {t for e in (*site.spokes, *site.hole) for t in pmap.edge_t_list[e] if t >= 0}
    after = {}
    for t in round_hole:
        verts = pmap.tri_v_list[t]
        inside = [i for i in range(3) if verts[i] in mine]
        if len(inside) == 1:  # (hole vertex, x, y): the ring runs from x to y
            i = inside[0]
            after[verts[(i + 1) % 3]] = (verts[(i + 2) % 3], pmap.tri_e_list[t][i], int(pmap.tri_n[t][i]), t)
    if len(after) < 3 or len(round_hole) != len(after) + 3:
        return None
    ring, edges = [min(after)], []
    while True:
        nxt, e, n, t = after.get(ring[-1], (None, 0, 0, 0))
        if nxt is None or len(edges) > len(after):
            return None
        edges.append((e, n, t))
        if nxt == ring[0]:
            break
        ring.append(nxt)
    return (ring, edges, round_hole) if len(ring) == len(after) else None


def _fill(pmap: PlanarMap, ring: list[int]):
    """Triangles filling the polygon ``ring`` (vertex ids, counter-clockwise),
    by cutting ears off it, as triples of positions in the ring. None if that fails."""
    xy = pmap.vxy
    left = list(range(len(ring)))
    out = []
    while len(left) > 3:
        for n in range(len(left)):
            i, j, k = left[n - 1], left[n], left[(n + 1) % len(left)]
            a, b, c = xy[ring[i]], xy[ring[j]], xy[ring[k]]
            if _cross(a, b, c) <= 1e-12:
                continue
            if any(m not in (i, j, k) and _cross(a, b, xy[ring[m]]) >= 0 and _cross(b, c, xy[ring[m]]) >= 0
                   and _cross(c, a, xy[ring[m]]) >= 0 for m in left):
                continue
            out.append((i, j, k))
            del left[n]
            break
        else:
            return None
    if _cross(*(xy[ring[m]] for m in left)) <= 1e-12:
        return None
    return out + [tuple(left)]


def _plan_delete(pmap: PlanarMap, site: Site):
    """How the map looks once the site is gone: its surroundings, a polygon,
    are filled with triangles again. Returns the ring, the new triangles and
    edges with the ids they take, and the slot that is freed; or None."""
    found = _ring(pmap, site)
    if found is None:
        return None
    ring, ring_edges, round_hole = found
    fill = _fill(pmap, ring)
    if fill is None:
        return None
    n = len(ring)
    tri_ids = sorted(round_hole)
    edge_ids = sorted(site.spokes)
    if len(tri_ids) != n + 3 or len(edge_ids) != n + 3:
        return None
    side = {}  # (ring position i, ring position j) with j after i on a triangle's boundary -> that triangle
    for t, tri in zip(tri_ids, fill):
        for m in range(3):
            side[(tri[m], tri[(m + 1) % 3])] = t
    diagonals = {}  # (i, j) with i < j, not neighbours on the ring -> edge id
    for (i, j) in sorted(side):
        if (j - i) % n not in (1, n - 1) and i < j:
            diagonals[(i, j)] = edge_ids[len(diagonals)]
    slot = (site.verts, (*edge_ids[len(diagonals):], *site.hole), tuple(tri_ids[n - 2:]))
    return ring, ring_edges, round_hole, list(zip(tri_ids, fill)), side, diagonals, slot


def _delete_map(pmap: PlanarMap, site: Site, plan=None) -> None:
    """The map's side of ``delete``: logs how to undo it."""
    ring, ring_edges, round_hole, triangles, side, diagonals, slot = plan or _plan_delete(pmap, site)
    n = len(ring)
    entry = {"kind": "delete", "site": site, "slot": slot, "spokes": list(site.spokes),
             "hole_xy": [pmap.vxy[v] for v in site.verts],
             "edges": [(e, _edge_row(pmap, e)) for e in (*sorted(site.spokes), *site.hole)],
             "tris": [(t, _triangle_row(pmap, t)) for t in sorted(round_hole)],
             "outer": [(e, pmap.edge_t_list[e]) for e, _, _ in ring_edges],
             "nbrs": [(nb, tuple(int(v) for v in pmap.tri_n[nb])) for _, nb, _ in ring_edges if nb >= 0],
             "claimed": [], "gained": []}
    owner = pmap.__dict__["_site_of_vertex"]
    for e in site.spokes:  # a spoke may also be a spoke of the site at its other end
        for v in pmap.edge_v_list[e]:
            if v in owner and owner[v] is not site:
                owner[v].spokes.remove(e)
                entry["claimed"].append((owner[v].pad, e))
    _unregister(pmap, site)
    _park(pmap, slot)

    def edge_between(i: int, j: int) -> int:
        if (j - i) % n == 1:
            return ring_edges[i][0]
        if (i - j) % n == 1:
            return ring_edges[j][0]
        return diagonals[(min(i, j), max(i, j))]

    for (i, j), e in diagonals.items():
        _write_edge(pmap, e, (ring[i], ring[j]), (side[(i, j)], side[(j, i)]), GATE, -1)
        for v in (ring[i], ring[j]):
            if v in owner:
                owner[v].spokes.append(e)
                entry["gained"].append((owner[v].pad, e))
    for t, tri in triangles:
        nbrs = []
        for m in range(3):  # across the edge opposite tri[m], which runs from tri[m + 1] to tri[m + 2]
            i, j = tri[(m + 1) % 3], tri[(m + 2) % 3]
            nbrs.append(ring_edges[i][1] if (j - i) % n == 1 else side[(j, i)])
        _write_triangle(pmap, t, [ring[m] for m in tri], [edge_between(tri[(m + 1) % 3], tri[(m + 2) % 3]) for m in range(3)], nbrs)
    for i, (e, nb, was) in enumerate(ring_edges):
        _repoint(pmap, e, nb, was, side[(i, (i + 1) % n)])
    free_slots(pmap).append(slot)
    _refresh(pmap, (*(e for e, _, _ in ring_edges), *sorted(entry["spokes"]), *site.hole))
    log(pmap).append(entry)


def delete(pmap: PlanarMap, state: TopoState, site: Site) -> bool:
    """Takes a sleeping site out of the map, wherever it is and whenever it was
    made. The triangles round the hole form a polygon; it is filled with
    triangles again, without the hole. Every wire that crossed the polygon
    keeps the two edges of it that it came in and went out by, and crosses
    whatever new edges lie between them, in the order the wires have on the
    polygon's boundary. With the hole gone, which way round it a wire went no
    longer matters.

    Returns False, changing nothing, if the site is a via, has a trace on it,
    or a wire that passes the polygon twice would have to cross one of the new
    edges twice.
    """
    if site.active or any(state.gate_order[e] for e in site.hole):
        return False
    plan = _plan_delete(pmap, site)
    if plan is None:
        return False
    ring, ring_edges, round_hole, triangles, side, diagonals, slot = plan
    n = len(ring)
    order = state.gate_order
    at = {e: i for i, (e, _, _) in enumerate(ring_edges)}  # ring edge -> its position (it runs from ring[i] to ring[i + 1])

    def along(e: int, w: int) -> int:
        """Place of wire w on ring edge e, counted from the edge's first vertex on the ring."""
        k = order[e].index(w)
        return k if pmap.edge_v_list[e][0] == ring[at[e]] else len(order[e]) - 1 - k

    # Every passage of a wire through the polygon: (wire, where in its path, edge in, edge out).
    wires = sorted({w for e in site.spokes for w in order[e]})
    runs = []
    for w in wires:
        steps, m = state.wire_path[w], 0
        while m < len(steps):
            if steps[m][1] not in round_hole:
                m += 1
                continue
            first = m
            while m + 1 < len(steps) and steps[m + 1][1] in round_hole:
                m += 1
            runs.append((w, first, m, steps[first - 1][0], steps[m][0]))
            m += 1
    # The new edges each passage crosses: those with its two ring edges on opposite sides.
    crossing: dict[tuple[int, int], list] = {d: [] for d in diagonals}
    for run in runs:
        w, _, _, came, leave = run
        for (i, j) in diagonals:
            a_in, b_in = i <= at[came] < j, i <= at[leave] < j
            if a_in != b_in:
                e = came if a_in else leave  # its end on the arc from ring[i] round to ring[j]
                crossing[(i, j)].append(((at[e] - i, along(e, w)), run))
    rows = {}
    for d, found in crossing.items():
        found.sort(key=lambda x: x[0])
        row = [run[0] for _, run in found]
        if len(set(row)) != len(row) or len(row) >= 15:
            return False  # a wire would cross this edge twice; or no slots left
        rows[d] = row

    notes = journal(state)
    outermost = not notes
    notes.append(("wires", {w: list(state.wire_path[w]) for w in wires}))
    for e in site.spokes:
        notes.append(("gate", e, list(order[e]), float(state.load[e]), float(state.cap[e])))
    for t in round_hole:
        notes.append(("cnt", t, list(state.corner_cnt[t])))
    _delete_map(pmap, site, plan)
    clear_free(pmap, state)

    home = {}  # edge of the filled polygon -> the triangles on its two sides, as (from, to) when crossed leaving ``from``
    for (i, j), t in side.items():
        e = ring_edges[i][0] if (j - i) % n == 1 else diagonals.get((min(i, j), max(i, j)))
        home.setdefault(e, []).append(t)
    inside = {t: tri for t, tri in triangles}
    count = {t: [0, 0, 0] for t in inside}
    for e in diagonals.values():
        order[e].clear()
    for d, e in diagonals.items():
        row = rows[d]
        order[e][:] = row if pmap.edge_v_list[e][0] == ring[d[0]] else row[::-1]
        state.load[e] = sum(state.weight[w] for w in row)
        state.count[e] = len(row)
        _set_capacity(pmap, state, e)
    new_steps: dict[int, list] = {}
    for w, first, last, came, leave in sorted(runs, key=lambda r: (r[0], -r[1])):  # later passages first: indices stay valid
        steps = state.wire_path[w]
        slot_out = steps[last][3]
        # Walk from the triangle behind the edge it came in by to the one behind the edge it leaves by.
        t = home[came][0]
        prev, out = came, []
        while leave not in pmap.tri_e_list[t]:
            # the one edge of t, other than the one just crossed, that this passage crosses
            nxt = next(e for e in pmap.tri_e_list[t] if e != prev and e in rows_by_edge(diagonals, rows) and w in rows_by_edge(diagonals, rows)[e]
                       and _separates(diagonals, e, at[came], at[leave]))
            k = pmap.tri_v_list[t].index(pmap.shared_vertex(prev, nxt))
            out.append((nxt, t, k, order[nxt].index(w)))
            count[t][k] += 1
            a, b = pmap.edge_t_list[nxt]
            t, prev = (b if a == t else a), nxt
        k = pmap.tri_v_list[t].index(pmap.shared_vertex(prev, leave))
        out.append((leave, t, k, slot_out))
        count[t][k] += 1
        steps[first:last + 1] = out
    for t, c in count.items():
        state.corner_cnt[t][:] = c
        state.corner[t] = c
    pmap.__dict__.setdefault("_moved_wires", set()).update(wires)
    state.resize()
    # The fill is whatever cutting ears gave. Make its inner edges Delaunay,
    # as the map's edges were before the site came.
    inner = set(diagonals.values())
    pending = sorted(inner)
    for _ in range(20 * len(inner) + 1):
        if not pending:
            break
        e = pending.pop()
        t1, t2 = pmap.edge_t_list[e]
        far = pmap.tri_v_list[t2][_apex(pmap, t2, e)]
        if _in_circle(*(pmap.vxy[v] for v in pmap.tri_v_list[t1]), pmap.vxy[far]) and flip(pmap, state, e):
            pending += [x for t in pmap.edge_t_list[e] for x in pmap.tri_e_list[t] if x in inner and x != e]
    if outermost:
        notes.clear()  # nothing else is waiting to be undone
    return True


def rows_by_edge(diagonals, rows) -> dict:
    return {diagonals[d]: rows[d] for d in diagonals}


def _separates(diagonals, e: int, a: int, b: int) -> bool:
    """Whether new edge ``e`` has ring edges number a and b on opposite sides."""
    (i, j) = next(d for d, x in diagonals.items() if x == e)
    return (i <= a < j) != (i <= b < j)


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
