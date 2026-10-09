"""The planar map (design section 6): arrays built once, then list/tuple tables for hot loops."""
import math
from dataclasses import dataclass, field

import numpy as np

from weaveengine.board import Board
from weaveengine.geom.capacity import capacity_from_width, edge_width
from weaveengine.geom.exits import exit_windows
from weaveengine.geom.inflate import WALL, FreeSpace, preprocess
from weaveengine.geom.triangulate import triangulate

GATE, WALL_EDGE, TERMINAL = 0, 1, 2


@dataclass
class PlanarMap:
    """Built once. The only thing that changes it afterwards is the creation of
    via sites (``topo/sites.py``, design section 12.2), which appends to it."""
    tri_v: np.ndarray      # [T,3] vertex ids, CCW
    tri_n: np.ndarray      # [T,3] neighbour across the edge opposite local vertex i (-1 = wall)
    tri_e: np.ndarray      # [T,3] edge id of the edge opposite local vertex i
    vx: np.ndarray
    vy: np.ndarray
    v_obs: np.ndarray      # [V] obstacle id (0 = outline, -1 = free)
    edge_v: np.ndarray     # [E,2] endpoints (u, v), u < v
    edge_t: np.ndarray     # [E,2] incident triangles; boundary edges hold theirs in column 0
    edge_len: np.ndarray
    edge_mid: np.ndarray   # [E,2]
    edge_cap: np.ndarray
    edge_kind: np.ndarray  # 0 = gate, 1 = wall, 2 = pad terminal
    edge_owner: np.ndarray # owning pad id for terminals, else -1
    pad_centre: dict[int, tuple[float, float]] = field(default_factory=dict)
    pad_net: dict[int, int] = field(default_factory=dict)
    v_nbr: dict[int, tuple[int, int]] = field(default_factory=dict)  # boundary neighbours of a vertex
    free_space: FreeSpace | None = None
    edge_width: np.ndarray | None = None  # [E] width estimate behind edge_cap
    pitch: float = 1.0                    # base trace width + clearance
    layer: int = 0

    def __post_init__(self):
        self.num_vertices = len(self.vx)
        self.num_triangles = len(self.tri_v)
        self.num_edges = len(self.edge_v)
        # Tier 2 (design section 3): plain lists/tuples for inner loops.
        self.tri_v_list = [tuple(r) for r in self.tri_v.tolist()]
        self.tri_e_list = [tuple(r) for r in self.tri_e.tolist()]
        self.edge_v_list = [tuple(r) for r in self.edge_v.tolist()]
        self.edge_t_list = [tuple(r) for r in self.edge_t.tolist()]
        self.edge_mid_list = [tuple(r) for r in self.edge_mid.tolist()]
        self.edge_len_list = self.edge_len.tolist()
        self.edge_cap_list = self.edge_cap.tolist()
        self.edge_kind_list = self.edge_kind.tolist()
        self.edge_owner_list = self.edge_owner.tolist()
        self.v_obs_list = self.v_obs.tolist()
        self.vxy = list(zip(self.vx.tolist(), self.vy.tolist()))
        cen = np.stack([self.vx[self.tri_v].mean(axis=1), self.vy[self.tri_v].mean(axis=1)], axis=1)
        self.tri_cen = [tuple(r) for r in cen.tolist()]

        self.pad_edges: dict[int, list[int]] = {}
        for e, (kind, owner) in enumerate(zip(self.edge_kind_list, self.edge_owner_list)):
            if kind == TERMINAL:
                self.pad_edges.setdefault(owner, []).append(e)
        # Heuristic radius and obstacle group per pad.
        self.pad_radius: dict[int, float] = {}
        self.pad_obs: dict[int, int] = {}
        for pad, edges in self.pad_edges.items():
            cx, cy = self.pad_centre.get(pad, self.edge_mid_list[edges[0]])
            self.pad_centre.setdefault(pad, (cx, cy))
            self.pad_radius[pad] = max(
                math.hypot(self.vxy[v][0] - cx, self.vxy[v][1] - cy)
                for e in edges for v in self.edge_v_list[e])
            self.pad_obs[pad] = self.v_obs_list[self.edge_v_list[edges[0]][0]]
        # Terminal edges touching each vertex (used by realisation keep-away).
        self.vertex_terminals: dict[int, list[int]] = {}
        for edges in self.pad_edges.values():
            for e in edges:
                for v in self.edge_v_list[e]:
                    self.vertex_terminals.setdefault(v, []).append(e)
        self.sites: dict[int, object] = {}  # via sites by pad id (topo/sites.py)
        # Pad edge -> the part of it (from, to; mm from its u end) a trace may leave through (geom/exits.py).
        self.exit_window: dict[int, tuple[float, float]] = self.__dict__.get("exit_window", {})
        self._build_transitions()

    def _build_transitions(self) -> None:
        """Per-half-edge successor tuples (design section 3, rule 3).

        Half-edge ``h = 2*e + side`` means "just crossed e into edge_t[e][side]".
        trans[h] holds, for each usable exit edge b of that triangle:
          (b, next half-edge or -1 at a boundary, triangle, corner local index,
           corner is u-end of e, corner is u-end of b, midpoint-to-midpoint length)
        """
        self.trans = [self.transitions(h) for h in range(2 * self.num_edges)]
        self.stale: set[int] = set()  # edges changed since the transitions were last brought up to date
        self.moved = True             # triangles changed since the grid for locating points was filled
        self.changes = 0              # counts the changes made to the map since it was built
        self.edge_changed = np.zeros(self.num_edges, dtype=np.int64)  # per edge: ``changes`` when it, or a triangle at it, last changed

    def catch_up(self) -> None:
        """Brings the transitions, and the compiled search's copy of them, up
        to date with the edges changed since (``stale``: a via site coming or
        going changes a dozen edges several times over, and nothing reads the
        transitions in between)."""
        if not self.stale:
            return
        touched = [2 * e + side for e in self.stale for side in (0, 1)]
        for h in touched:
            self.trans[h] = self.transitions(h)
        tables = self.__dict__.get("_kernel_tables")
        if tables is not None and not tables.update(self, touched):
            del self.__dict__["_kernel_tables"]  # outgrown: built again, with room, at the next search
        self.stale.clear()

    def transitions(self, h: int) -> tuple:
        """The successor tuples of one half-edge, from the tables as they stand."""
        e, side = h >> 1, h & 1
        t = self.edge_t_list[e][side]
        if t < 0:
            return ()
        kind, mids = self.edge_kind_list, self.edge_mid_list
        verts, edges = self.tri_v_list[t], self.tri_e_list[t]
        entry_local = edges.index(e)
        out = []
        for out_local in range(3):
            b = edges[out_local]
            if out_local == entry_local or kind[b] == WALL_EDGE:
                continue
            k = 3 - entry_local - out_local
            c = verts[k]
            tb = self.edge_t_list[b]
            if tb[1] < 0:
                nxt = -1
            else:
                nxt = 2 * b + (1 if tb[0] == t else 0)
            length = math.hypot(mids[b][0] - mids[e][0], mids[b][1] - mids[e][1])
            out.append((b, nxt, t, k, c == self.edge_v_list[e][0], c == self.edge_v_list[b][0], length))
        return tuple(out)

    def shared_vertex(self, e1: int, e2: int) -> int:
        a, b = self.edge_v_list[e1]
        return a if a in self.edge_v_list[e2] else (b if b in self.edge_v_list[e2] else -1)

    def common_triangle(self, e1: int, e2: int) -> int:
        for t in self.edge_t_list[e1]:
            if t >= 0 and e2 in self.tri_e_list[t]:
                return t
        return -1


def from_triangles(verts: np.ndarray, tris: np.ndarray, v_obs: np.ndarray, segs: dict[tuple[int, int], int],
                   pitch: float, obs_geom: dict | None = None, **extra) -> PlanarMap:
    """Assemble the section 6.2 tables from a triangle soup and its boundary classification."""
    edge_id: dict[tuple[int, int], int] = {}
    edge_t: list[list[int]] = []
    tri_e = np.zeros((len(tris), 3), dtype=np.int64)
    for t, tri in enumerate(tris.tolist()):
        for k in range(3):
            a, b = tri[(k + 1) % 3], tri[(k + 2) % 3]
            key = (min(a, b), max(a, b))
            e = edge_id.get(key)
            if e is None:
                e = edge_id[key] = len(edge_t)
                edge_t.append([t, -1])
            else:
                edge_t[e][1] = t
            tri_e[t, k] = e
    edge_v = np.array(list(edge_id), dtype=np.int64).reshape(-1, 2)
    edge_t_arr = np.array(edge_t, dtype=np.int64).reshape(-1, 2)
    tri_n = np.full((len(tris), 3), -1, dtype=np.int64)
    for t in range(len(tris)):
        for k in range(3):
            a, b = edge_t[tri_e[t, k]]
            tri_n[t, k] = b if a == t else a

    edge_kind = np.zeros(len(edge_v), dtype=np.int64)
    edge_owner = np.full(len(edge_v), -1, dtype=np.int64)
    for key, e in edge_id.items():
        if edge_t[e][1] < 0:
            owner = segs.get(key, WALL)
            if owner == WALL:
                edge_kind[e] = WALL_EDGE
            else:
                edge_kind[e] = TERMINAL
                edge_owner[e] = owner
    pu, pv = verts[edge_v[:, 0]], verts[edge_v[:, 1]]
    edge_len = np.hypot(*(pv - pu).T)
    width = edge_len.copy() if obs_geom is None else edge_width(verts, edge_v, edge_len, edge_kind, v_obs, obs_geom)
    edge_cap = capacity_from_width(width, edge_kind, pitch)
    return PlanarMap(
        tri_v=tris, tri_n=tri_n, tri_e=tri_e, vx=verts[:, 0].copy(), vy=verts[:, 1].copy(), v_obs=v_obs,
        edge_v=edge_v, edge_t=edge_t_arr, edge_len=edge_len, edge_mid=(pu + pv) / 2.0,
        edge_cap=edge_cap, edge_kind=edge_kind, edge_owner=edge_owner, edge_width=width, pitch=pitch, **extra)


def build(board: Board, layer: int = 0, force_fallback: bool = False) -> PlanarMap:
    fs = preprocess(board, layer)
    verts, tris, v_obs, segs, v_nbr = triangulate(fs, force_fallback=force_fallback)
    pmap = from_triangles(
        verts, tris, v_obs, segs, board.rules.pitch, fs.obs_geom,
        pad_centre={p.pad_id: p.centre for p in board.pads_on(layer)},
        pad_net={p.pad_id: p.net_id for p in board.pads_on(layer)}, layer=layer,
        v_nbr=v_nbr, free_space=fs)
    # A pad edge from which no trace can legally leave is a wall (13.1, step 4).
    windows, dead = exit_windows(board, layer, pmap)
    if dead:
        pmap.edge_kind[dead], pmap.edge_owner[dead], pmap.edge_cap[dead] = WALL_EDGE, -1, 0
        pmap.__post_init__()
    pmap.exit_window = windows
    return pmap
