"""Topological state (design section 7): ordered wires per gate, corner counts per triangle."""
import math

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
        self.cap: list[float] = [0.0 if kinds[e] == 1 else w / planar_map.pitch + 1.0
                                 for e, w in enumerate(planar_map.edge_width.tolist())]
        # A pad edge always lets one trace leave its own pad, however wide:
        # give every pad edge room for the widest trace that ends on it alone.
        self._terminal = [k == 2 for k in kinds]
        self.load: list[float] = [0.0] * planar_map.num_edges
        self.weight: dict[int, float] = {}
        self.hist: list[float] = [0.0] * planar_map.num_edges

    def usage(self, edge_id: int) -> int:
        return len(self.gate_order[edge_id])

    def overflow(self, edge_id: int) -> int:
        """Whole base-width wires by which the gate is over capacity."""
        if self._terminal[edge_id] and len(self.gate_order[edge_id]) <= 1:
            return 0
        return max(0, math.ceil(self.load[edge_id] - self.cap[edge_id] - 1e-9))

    def overflowed_gates(self) -> list[int]:
        cap, order, terminal = self.cap, self.gate_order, self._terminal
        return [e for e, load in enumerate(self.load)
                if load > cap[e] + 1e-9 and not (terminal[e] and len(order[e]) <= 1)]

    def fits(self, edge_id: int, weight: float) -> bool:
        """Whether one more wire of this weight fits on the edge."""
        if self._terminal[edge_id] and not self.gate_order[edge_id]:
            return True
        return self.load[edge_id] + weight <= self.cap[edge_id] + 1e-9

    def full(self, edge_id: int, extra: float = 0.0) -> bool:
        """No further base-width wire fits (after adding ``extra`` load)."""
        return self.cap[edge_id] - self.load[edge_id] - extra < 1.0 - 1e-9

    def insert(self, wire_id: int, steps: list[Step], weight: float = 1.0) -> None:
        if wire_id in self.wire_path:
            raise ValueError(f"wire {wire_id} already exists")
        gates = [s[0] for s in steps]
        if len(set(gates)) != len(gates):
            raise ValueError("a path must never cross the same gate twice")
        # Slots come from one pre-insertion snapshot; gates are distinct, so
        # the inserts are independent of each other.
        for edge_id, tri_id, corner_k, slot in steps:
            self.gate_order[edge_id].insert(slot, wire_id)
            self.load[edge_id] += weight
            if tri_id >= 0:
                self.corner_cnt[tri_id][corner_k] += 1
        self.wire_path[wire_id] = list(steps)
        self.weight[wire_id] = weight

    def remove(self, wire_id: int) -> None:
        weight = self.weight.pop(wire_id)
        for edge_id, tri_id, corner_k, _ in self.wire_path.pop(wire_id):
            self.gate_order[edge_id].remove(wire_id)
            self.load[edge_id] = self.load[edge_id] - weight if self.gate_order[edge_id] else 0.0
            if tri_id >= 0:
                self.corner_cnt[tri_id][corner_k] -= 1

    def snapshot(self):
        return ([list(o) for o in self.gate_order], [list(c) for c in self.corner_cnt],
                {w: list(p) for w, p in self.wire_path.items()}, list(self.load), dict(self.weight))

    def restore(self, snap) -> None:
        order, cnt, paths, load, weight = snap
        self.load, self.weight = list(load), dict(weight)
        self.gate_order = [list(o) for o in order]
        self.corner_cnt = [list(c) for c in cnt]
        self.wire_path = {w: list(p) for w, p in paths.items()}

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
