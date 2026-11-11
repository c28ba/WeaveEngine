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
        self.cap = np.where(planar_map.edge_kind == 1, 0.0, planar_map.edge_width / planar_map.pitch + 1.0)
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
        # A net narrowed to reach fine-pitch pads is planned at its own width but
        # near them (``router.reach``): there its wires weigh less. Net -> (those
        # gates, or None: every gate, the net planned narrow all along; the
        # weight there). ``near``: the gates near the pads, and ``circles``
        # round them, for the gates of via sites made later.
        self.zone: dict[int, tuple[set | None, float]] = {}
        self.near: dict[int, set] = {}
        self.circles: dict[int, list] = {}
        self._mask = None  # (net, size, the zone as a mask), for the search kernel
        self.epoch = 0  # counts the times a wire has come or gone: what was worked out from the wires holds while it stands
        self.stamp = np.zeros(planar_map.num_edges, dtype=np.int64)  # per gate: ``epoch`` when a wire last came or went there

    def resize(self) -> None:
        """Brings every table up to the map's present size (the map has gained
        room for via sites; it never shrinks). New gates and triangles start empty."""
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
            self.stamp = np.concatenate([self.stamp, np.zeros(edges, dtype=np.int64)])
            for net, circles in self.circles.items():
                self.near[net].update(_near(m, circles, first))
        if tris > 0:
            self.corner_cnt += [[0, 0, 0] for _ in range(tris)]
            self.corner = np.concatenate([self.corner, np.zeros((tris, 3), dtype=np.int32)])

    def narrow_near(self, net: int, circles: list, weight: float) -> None:
        """Wires of ``net`` weigh ``weight`` on the gates that come within a circle
        (x, y, radius) of ``circles``, and their own weight elsewhere. Before any is in."""
        self.circles[net] = list(circles)
        self.near[net] = _near(self.map, circles)
        self.zone[net] = (self.near[net], weight)

    def everywhere(self, net: int, narrow: bool) -> None:
        """Plans ``net`` narrow on every gate (or only near its fine-pitch pads
        again). The loads are not worked out again: no wire of the net may be in,
        or the loads must be those of a snapshot taken with it so."""
        if net in self.zone:
            self.zone[net] = (None if narrow else self.near[net], self.zone[net][1])
            self._mask = None

    def at(self, wire_id: int, edge_id: int) -> float:
        """The load a wire puts on a gate (its weight, or less near a fine-pitch pad of its net)."""
        z = self.zone.get(self.net.get(wire_id, -1))
        if z is not None and (z[0] is None or edge_id in z[0]):
            return z[1]
        return self.weight[wire_id]

    def weight_on(self, net: int, weight: float, edge_id: int) -> float:
        """The load a wire of ``net`` and ``weight`` would put on a gate."""
        z = self.zone.get(net)
        if z is not None and (z[0] is None or edge_id in z[0]):
            return z[1]
        return weight

    def narrow_at(self, net: int, edge_id: int) -> bool:
        """Whether a wire of ``net`` is planned narrower than its weight on this gate."""
        z = self.zone.get(net)
        return z is not None and (z[0] is None or edge_id in z[0])

    def search_weight(self, net: int, weight: float):
        """(weight, weight near the net's fine-pitch pads, mask of those gates, whether there are any), for the kernel."""
        z = self.zone.get(net)
        if z is None:
            return weight, weight, _NO_MASK, False
        if z[0] is None:
            return z[1], z[1], _NO_MASK, False
        n = self.map.num_edges
        if self._mask is None or self._mask[:2] != (net, n):
            mask = np.zeros(n, dtype=np.uint8)
            mask[list(z[0])] = 1
            self._mask = (net, n, mask)
        return weight, z[1], self._mask[2], True

    def usage(self, edge_id: int) -> int:
        return len(self.gate_order[edge_id])

    def tally(self, edge_id: int) -> float:
        """The load on a gate, from the wires on it: every run of neighbours of
        one net counts once, as its widest wire. A run at an end of the gate
        that is a via of its own net counts nothing: there it is the via's
        copper, inside the room the via keeps clear (12.2)."""
        runs, last = [], -1
        for w in self.gate_order[edge_id]:
            net = self.net.get(w, -1)
            if net < 0 or net != last:
                runs.append(0.0)
            runs[-1] = max(runs[-1], self.at(w, edge_id))
            last = net
        vias = self.map.__dict__.get("_site_of_vertex")
        if vias and runs:
            row, (u, v) = self.gate_order[edge_id], self.map.edge_v_list[edge_id]
            if v in vias and vias[v].net >= 0 and vias[v].net == self.net.get(row[-1], -1):
                runs.pop()
            if runs and u in vias and vias[u].net >= 0 and vias[u].net == self.net.get(row[0], -1):
                runs[0] = 0.0
        return sum(runs)

    def beside(self, net: int) -> dict[int, int]:
        """Gate -> the places on it that are beside copper of ``net`` (bit p
        set): a wire put in at place p has a wire of the net as its neighbour,
        or lies against a via of the net."""
        places: dict[int, int] = {}
        for w in self.by_net.get(net, ()):
            for e, _, _, _ in self.wire_path[w]:
                p = self.gate_order[e].index(w)
                places[e] = places.get(e, 0) | (3 << p)
        for site in self.map.sites.values():
            if site.net == net:
                for e in site.spokes:
                    for end, v in enumerate(self.map.edge_v_list[e]):
                        if v in site.verts:
                            places[e] = places.get(e, 0) | (1 << (len(self.gate_order[e]) if end else 0))
        return places

    def _one(self, edge_id: int) -> bool:
        """Whether the wires on a pad edge are one trace leaving its own pad (one wire, or a run of one net),
        which a pad edge always lets through, however wide."""
        row = self.gate_order[edge_id]
        return self.map.edge_kind_list[edge_id] == 2 and (len(row) <= 1 or len({self.net.get(w, -1 - w) for w in row}) == 1)

    def overflow(self, edge_id: int) -> int:
        """Whole base-width wires by which the gate is over capacity."""
        if self._one(edge_id):
            return 0
        return max(0, math.ceil(self.load[edge_id] - self.cap[edge_id] - 1e-9))

    def overflowed_gates(self) -> list[int]:
        return [e for e in np.nonzero(self.load > self.cap + 1e-9)[0].tolist() if not self._one(e)]

    def fits(self, edge_id: int, weight: float, net: int = -1) -> bool:
        """Whether one more wire of this weight (and net) fits on the edge."""
        if not self.gate_order[edge_id] and self.map.edge_kind_list[edge_id] == 2:
            return True
        return self.load[edge_id] + self.weight_on(net, weight, edge_id) <= self.cap[edge_id] + 1e-9

    def full(self, edge_id: int, extra: float = 0.0) -> bool:
        """No further base-width wire fits (after adding ``extra`` load)."""
        return self.cap[edge_id] - self.load[edge_id] - extra < 1.0 - 1e-9

    def insert(self, wire_id: int, steps: list[Step], weight: float = 1.0, net: int | None = None) -> None:
        """``net`` None: the net this wire had when it was last in (or none)."""
        if wire_id in self.wire_path:
            raise ValueError(f"wire {wire_id} already exists")
        self.epoch += 1
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
        nets, mine = self.net, self.net.get(wire_id, -1)
        vias, edge_v = self.map.__dict__.get("_site_of_vertex") or (), self.map.edge_v_list
        for edge_id, tri_id, corner_k, slot in steps:
            row = self.gate_order[edge_id]
            row.insert(slot, wire_id)
            self.stamp[edge_id] = self.epoch
            # Its two neighbours: beside one of its own net it joins that run; between two of
            # one net it splits theirs in two (``remove`` has the converse); at a via, the via decides.
            a = nets.get(row[slot - 1], -1) if slot else -1
            b = nets.get(row[slot + 1], -1) if slot + 1 < len(row) else -1
            if ((mine >= 0 and (a == mine or b == mine)) or (a >= 0 and a == b)
                    or (vias and (edge_v[edge_id][0] in vias or edge_v[edge_id][1] in vias))):
                self.load[edge_id] = self.tally(edge_id)
            else:
                self.load[edge_id] += self.at(wire_id, edge_id)
            self.count[edge_id] += 1
            if tri_id >= 0:
                self.corner_cnt[tri_id][corner_k] += 1
                self.corner[tri_id, corner_k] += 1
        self.wire_path[wire_id] = list(steps)

    def remove(self, wire_id: int) -> None:
        self.epoch += 1
        self.by_net.get(self.net.get(wire_id, -1), set()).discard(wire_id)
        nets, mine = self.net, self.net.get(wire_id, -1)
        vias, edge_v = self.map.__dict__.get("_site_of_vertex") or (), self.map.edge_v_list
        for edge_id, tri_id, corner_k, _ in self.wire_path.pop(wire_id):
            row = self.gate_order[edge_id]
            i = row.index(wire_id)
            del row[i]
            self.stamp[edge_id] = self.epoch
            # Its two neighbours now meet. Only if one of them is of its net, or
            # they are of one net themselves, or the gate is at a via, is there a run to count again.
            a = nets.get(row[i - 1], -1) if i else -1
            b = nets.get(row[i], -1) if i < len(row) else -1
            if not row:
                self.load[edge_id] = 0.0
            elif ((mine >= 0 and (a == mine or b == mine)) or (a >= 0 and a == b)
                  or (vias and (edge_v[edge_id][0] in vias or edge_v[edge_id][1] in vias))):
                self.load[edge_id] = self.tally(edge_id)
            else:
                self.load[edge_id] -= self.at(wire_id, edge_id)
            self.count[edge_id] -= 1
            if tri_id >= 0:
                self.corner_cnt[tri_id][corner_k] -= 1
                self.corner[tri_id, corner_k] -= 1
        del self.weight[wire_id]

    def __getstate__(self):
        return {**self.__dict__, "_mask": None}

    def snapshot(self):
        # The nets go too: a snapshot is also restored in another process (a raced
        # variant's parent, a routing gone on from), whose state has never seen these wires.
        return ([list(o) for o in self.gate_order], [list(c) for c in self.corner_cnt],
                {w: list(p) for w, p in self.wire_path.items()}, self.load.copy(), dict(self.weight),
                {w: self.net[w] for w in self.wire_path if w in self.net})

    def restore(self, snap) -> None:
        order, cnt, paths, load, weight, nets = snap
        self.net.update(nets)
        self.epoch += 1
        self.stamp = np.full(len(order), self.epoch, dtype=np.int64)
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


_NO_MASK = np.zeros(1, dtype=np.uint8)


def _near(pmap: PlanarMap, circles: list, first: int = 0) -> set[int]:
    """The edges from ``first`` on that come within a circle (x, y, radius) of ``circles``."""
    if not circles or pmap.num_edges <= first:
        return set()
    ev = np.array(pmap.edge_v_list[first:], dtype=np.int64)
    xy = np.array(pmap.vxy, dtype=np.float64)
    a, b = xy[ev[:, 0]], xy[ev[:, 1]]
    d = b - a
    n2 = np.maximum((d * d).sum(axis=1), 1e-18)
    hit = np.zeros(len(ev), dtype=bool)
    for x, y, r in circles:
        t = np.clip(((x - a[:, 0]) * d[:, 0] + (y - a[:, 1]) * d[:, 1]) / n2, 0.0, 1.0)
        hit |= np.hypot(a[:, 0] + t * d[:, 0] - x, a[:, 1] + t * d[:, 1] - y) <= r
    return set((np.nonzero(hit)[0] + first).tolist())
