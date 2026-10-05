"""Topological state (design section 7): ordered wires per gate, corner counts per triangle."""
import math

import numpy as np

from weaveengine.topo.planar_map import PlanarMap

# A step is (edge_id, tri_id, corner_k, slot): the wire crossed ``edge_id`` at
# position ``slot`` after traversing ``tri_id``, cutting off its local corner
# ``corner_k``. The first step (the start terminal) has tri_id = corner_k = -1.
Step = tuple[int, int, int, int]


class TopoState:
    def __init__(self, planar_map: PlanarMap):
        self.map = planar_map
        self.gate_order: list[list[int]] = [[] for _ in range(planar_map.num_edges)]
        self.corner_cnt: list[list[int]] = [[0, 0, 0] for _ in range(planar_map.num_triangles)]
        self.wire_path: dict[int, list[Step]] = {}
        # Capacity in units of the base pitch, un-floored: width / pitch + 1.
        # A base-width wire weighs 1, a wider one 1 + extra width / pitch, so
        # "load + weight <= cap" is the floor(w / (t + s)) + 1 rule of 6.3 for
        # uniform wires and stays correct for mixed widths. It is an estimate;
        # DRC feedback may lower it, so the working copy lives here.
        kinds = planar_map.edge_kind_list
        self.cap = np.where(planar_map.edge_kind == 1, 0.0, planar_map.edge_width / planar_map.pitch + 1.0)
        # A pad edge always lets one trace leave its own pad, however wide.
        self._terminal = [k == 2 for k in kinds]
        self.load = np.zeros(planar_map.num_edges)
        self.weight: dict[int, float] = {}
        # Wires of one net side by side on a gate are one trace there (11):
        # they take the room of one. The net of each wire (-1: of no net), and
        # the wires of each net that are in.
        self.net: dict[int, int] = {}
        self.by_net: dict[int, set[int]] = {}
        # Flat mirrors of the usage and corner counts, for the compiled search kernel.
        self.count = np.zeros(planar_map.num_edges, dtype=np.int32)
        self.corner = np.zeros((planar_map.num_triangles, 3), dtype=np.int32)
        self.hist = np.zeros(planar_map.num_edges)

    def resize(self) -> None:
        """Brings every table up to the map's present size (the map has gained
        room for via sites; it never shrinks) and takes the kinds of its edges
        from the map again. New gates and triangles start empty."""
        m = self.map
        edges, tris = m.num_edges - len(self.gate_order), m.num_triangles - len(self.corner_cnt)
        if edges > 0:
            first = len(self.gate_order)
            self.gate_order += [[] for _ in range(edges)]
            cap = np.where(m.edge_kind[first:] == 1, 0.0, m.edge_width[first:] / m.pitch + 1.0)
            self.cap = np.concatenate([self.cap, cap])
            self.load = np.concatenate([self.load, np.zeros(edges)])
            self.hist = np.concatenate([self.hist, np.zeros(edges)])
            self.count = np.concatenate([self.count, np.zeros(edges, dtype=np.int32)])
        if tris > 0:
            self.corner_cnt += [[0, 0, 0] for _ in range(tris)]
            self.corner = np.concatenate([self.corner, np.zeros((tris, 3), dtype=np.int32)])
        if m.sites or edges > 0:
            self._terminal = [k == 2 for k in m.edge_kind_list]

    def usage(self, edge_id: int) -> int:
        return len(self.gate_order[edge_id])

    def tally(self, edge_id: int) -> float:
        """The load on a gate, from the wires on it: every run of neighbours of
        one net counts once, as its widest wire."""
        total, run, last = 0.0, 0.0, -1
        for w in self.gate_order[edge_id]:
            net = self.net.get(w, -1)
            if net < 0 or net != last:
                total += run
                run = 0.0
            run = max(run, self.weight[w])
            last = net
        return total + run

    def beside(self, net: int) -> dict[int, int]:
        """Gate -> the places on it that are beside a wire of ``net`` (bit p set:
        a wire put in at place p has one as its neighbour)."""
        places: dict[int, int] = {}
        for w in self.by_net.get(net, ()):
            for e, _, _, _ in self.wire_path[w]:
                p = self.gate_order[e].index(w)
                places[e] = places.get(e, 0) | (3 << p)
        return places

    def overflow(self, edge_id: int) -> int:
        """Whole base-width wires by which the gate is over capacity."""
        if self._terminal[edge_id] and len(self.gate_order[edge_id]) <= 1:
            return 0
        return max(0, math.ceil(self.load[edge_id] - self.cap[edge_id] - 1e-9))

    def overflowed_gates(self) -> list[int]:
        order, terminal = self.gate_order, self._terminal
        return [e for e in np.nonzero(self.load > self.cap + 1e-9)[0].tolist()
                if not (terminal[e] and len(order[e]) <= 1)]

    def fits(self, edge_id: int, weight: float) -> bool:
        """Whether one more wire of this weight fits on the edge."""
        if self._terminal[edge_id] and not self.gate_order[edge_id]:
            return True
        return self.load[edge_id] + weight <= self.cap[edge_id] + 1e-9

    def full(self, edge_id: int, extra: float = 0.0) -> bool:
        """No further base-width wire fits (after adding ``extra`` load)."""
        return self.cap[edge_id] - self.load[edge_id] - extra < 1.0 - 1e-9

    def insert(self, wire_id: int, steps: list[Step], weight: float = 1.0, net: int | None = None) -> None:
        """``net`` None: the net this wire had when it was last in (or none)."""
        if wire_id in self.wire_path:
            raise ValueError(f"wire {wire_id} already exists")
        if net is not None:
            self.net[wire_id] = net
        self.weight[wire_id] = weight
        if self.net.get(wire_id, -1) >= 0:
            self.by_net.setdefault(self.net[wire_id], set()).add(wire_id)
        gates = [s[0] for s in steps]
        if len(set(gates)) != len(gates):
            raise ValueError("a path must never cross the same gate twice")
        # Slots come from one pre-insertion snapshot; gates are distinct, so
        # the inserts are independent of each other.
        for edge_id, tri_id, corner_k, slot in steps:
            self.gate_order[edge_id].insert(slot, wire_id)
            self.load[edge_id] = self.tally(edge_id)
            self.count[edge_id] += 1
            if tri_id >= 0:
                self.corner_cnt[tri_id][corner_k] += 1
                self.corner[tri_id, corner_k] += 1
        self.wire_path[wire_id] = list(steps)

    def remove(self, wire_id: int) -> None:
        self.by_net.get(self.net.get(wire_id, -1), set()).discard(wire_id)
        for edge_id, tri_id, corner_k, _ in self.wire_path.pop(wire_id):
            self.gate_order[edge_id].remove(wire_id)
            self.load[edge_id] = self.tally(edge_id)
            self.count[edge_id] -= 1
            if tri_id >= 0:
                self.corner_cnt[tri_id][corner_k] -= 1
                self.corner[tri_id, corner_k] -= 1
        del self.weight[wire_id]

    def snapshot(self):
        return ([list(o) for o in self.gate_order], [list(c) for c in self.corner_cnt],
                {w: list(p) for w, p in self.wire_path.items()}, self.load.copy(), dict(self.weight))

    def restore(self, snap) -> None:
        order, cnt, paths, load, weight = snap
        self.load, self.weight = load.copy(), dict(weight)
        self.count = np.array([len(o) for o in order], dtype=np.int32)
        self.corner = np.array(cnt, dtype=np.int32).reshape(-1, 3)
        self.gate_order = [list(o) for o in order]
        self.corner_cnt = [list(c) for c in cnt]
        self.wire_path = {w: list(p) for w, p in paths.items()}
        self.by_net = {}
        for w in self.wire_path:
            if self.net.get(w, -1) >= 0:
                self.by_net.setdefault(self.net[w], set()).add(w)

    def check_invariants(self) -> bool:
        return not self.invariant_errors(limit=1)

    def invariant_errors(self, limit: int = 20) -> list[str]:
        """Section 7.2: around every corner the two adjacent edges agree on the
        nested wires, and every wire on an edge cuts one of its two end corners."""
        errors: list[str] = []
        m = self.map
        for t in range(m.num_triangles):
            verts, edges, cnt = m.tri_v_list[t], m.tri_e_list[t], self.corner_cnt[t]
            for k in range(3):
                a, b = edges[(k + 1) % 3], edges[(k + 2) % 3]
                if self._near(a, verts[k], cnt[k]) != self._near(b, verts[k], cnt[k]):
                    errors.append(f"triangle {t} corner {k}: edges {a} and {b} disagree")
                opposite = edges[k]
                if self.usage(opposite) != cnt[(k + 1) % 3] + cnt[(k + 2) % 3]:
                    errors.append(f"triangle {t}: usage of edge {opposite} != corner counts at its ends")
                if len(errors) >= limit:
                    return errors
        return errors

    def _near(self, edge_id: int, corner_v: int, count: int) -> list[int]:
        order = self.gate_order[edge_id]
        if corner_v == self.map.edge_v_list[edge_id][0]:
            return order[:count]
        return order[::-1][:count]
