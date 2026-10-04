"""Shared routing context: layers, connections, demand map, airwire index, barrier (Phase 0)."""
import math
import time
from dataclasses import dataclass, field

import numpy as np
from scipy.sparse.csgraph import minimum_spanning_tree

from weaveengine import parallel
from weaveengine.board import Board
from weaveengine.topo.barrier import Barrier
from weaveengine.topo.costs import CostParams
from weaveengine.topo.planar_map import GATE, PlanarMap
from weaveengine.topo.runs import GatePath, airwire_path, path_from_steps
from weaveengine.topo.search import Route, route
from weaveengine.topo.state import TopoState


@dataclass
class Connection:
    """One 2-pin connection; its wire id is ``wire_id``."""
    wire_id: int
    net_id: int
    src: int
    dst: int
    air_len: float = 0.0
    weight: float = 1.0                              # gate load: 1 for a base-width trace
    layers: tuple[int, ...] = ()                     # layers on which both pads can be reached
    airwire: dict[int, GatePath] = field(default_factory=dict)       # per layer
    demand_gates: dict[int, frozenset] = field(default_factory=dict)  # per layer
    layer: int | None = None                         # where it is routed
    fails: int = 0
    dead: bool = False  # no path exists even with crossings allowed


class Layer:
    """One copper layer: its planar map and topological state (section 12, option 2)."""

    def __init__(self, index: int, name: str, pmap: PlanarMap):
        self.index = index
        self.name = name
        self.pmap = pmap
        self.state = TopoState(pmap)
        self.demand: list[float] = [0.0] * pmap.num_edges
        self.air_index: dict[int, set[int]] = {}
        self.barrier = Barrier()
        self.paths: dict[int, GatePath] = {}


def spanning_pairs(pads: list[int], centre: dict[int, tuple[float, float]]) -> list[tuple[int, int, float]]:
    """Minimum spanning tree over pad positions (section 11)."""
    pts = np.array([centre[p] for p in pads])
    dist = np.hypot(pts[:, None, 0] - pts[None, :, 0], pts[:, None, 1] - pts[None, :, 1])
    tree = minimum_spanning_tree(dist).tocoo()
    return [(pads[i], pads[j], w) for i, j, w in sorted(zip(tree.row.tolist(), tree.col.tolist(), tree.data.tolist()))]


def decompose(board: Board, layers: list[Layer]) -> tuple[list[tuple[int, int, int]], list[int]]:
    """Multi-pin nets -> 2-pin connections (net, pad, pad). Also returns pads
    that cannot be reached on any layer."""
    reachable = set().union(*[set(layer.pmap.pad_edges) for layer in layers])
    centre = {p.pad_id: p.centre for p in board.pads}
    pairs: list[tuple[int, int, int]] = []
    buried: list[int] = []
    for net_id, pads in sorted(board.nets().items()):
        buried += [p for p in pads if p not in reachable]
        pads = [p for p in pads if p in reachable]
        if len(pads) >= 2:
            pairs += [(net_id, a, b) for a, b, _ in spanning_pairs(pads, centre)]
    return pairs, buried


@dataclass
class Options:
    """Feature switches for the section 17.3 experiments."""
    global_selection: bool = True   # Phase 1 (E2)
    regret_order: bool = True       # Phase 2 ordering; False = shortest first (E1)
    lookahead: bool = True          # airwire-crossing and closure costs (E3)
    demand: bool = True             # demand map (E4)
    ripup: bool = True              # Phase 3 (E5)
    refine: bool = True             # Phase 4: reroute each wire once, keep it if shorter
    vias: bool = True               # insert via sites for connections that cannot be completed
    smooth: bool = True             # round sharp corners where the design rules leave room
    teardrops: bool = True          # teardrops where traces meet pads and vias
    teardrop_max_length: float = 1.0   # mm beyond the pad
    teardrop_max_width: float = 2.0    # mm across
    teardrop_breathing: float = 1.5    # clearances a teardrop keeps from foreign copper, else it is left out
    portfolio: int = 0              # routing variants raced per pass; 0 = one per worker (at most 8), 1 = off


class Context:
    def __init__(self, board: Board, layers: list[Layer], pairs: list[tuple[int, int, int]],
                 params: CostParams | None = None, options: Options | None = None, seed: int = 0,
                 workers: int | None = None, progress=None, events=None):
        self.workers = workers      # None = all cores; 1 = this process only
        self.progress = progress    # callable(phase: str, done: float, total: float) or None
        self.events = events        # callable(dict) or None: live events for a front end (see weaveengine.session)
        self.variant = 0            # which raced variant this context is (0 = the plain one)
        self.stop = None            # event set when this variant's result is no longer wanted
        self.pass_index = 1
        self._last_snapshot = 0.0
        self.board = board
        self.layers = layers
        self.params = params or CostParams.for_map(layers[0].pmap)
        self.options = options or Options()
        self.seed = seed
        rules = board.rules
        centre = {p.pad_id: p.centre for p in board.pads}
        self.conns: dict[int, Connection] = {}
        for i, (net_id, a, b) in enumerate(pairs, 1):
            self.conns[i] = Connection(
                i, net_id, a, b, air_len=math.dist(centre[a], centre[b]),
                weight=1.0 + 2.0 * rules.extra(net_id) / rules.pitch,
                layers=tuple(l.index for l in layers if a in l.pmap.pad_edges and b in l.pmap.pad_edges))
        self.unrouted: set[int] = set(self.conns)
        self.rounds = 0
        self._setup()

    def _setup(self) -> None:
        """Phase 0: airwire paths and the demand map (9.2)."""
        for c in self.conns.values():
            for li in c.layers:
                layer = self.layers[li]
                pmap = layer.pmap
                c.airwire[li] = airwire_path(pmap, pmap.pad_centre[c.src], pmap.pad_centre[c.dst])
                for g in c.airwire[li].gates:
                    layer.air_index.setdefault(g, set()).add(c.wire_id)
                if self.options.demand:
                    # Plain shortest path, congestion off. A connection that may
                    # use several layers spreads its demand over them.
                    r = route(pmap, layer.state, c.src, c.dst, self.params, congestion=False, weight=c.weight)
                    c.demand_gates[li] = frozenset(r.gates) if r is not None else frozenset()
                    for g in c.demand_gates[li]:
                        layer.demand[g] += c.weight / len(c.layers)

    # -- bookkeeping shared by Phase 2 and 3 -------------------------------
    def _demand(self, conn: Connection, sign: float) -> None:
        for li, gates in conn.demand_gates.items():
            demand = self.layers[li].demand
            for g in gates:
                demand[g] += sign * conn.weight / len(conn.layers)

    def commit(self, conn: Connection, r: Route) -> None:
        layer = self.layers[r.layer]
        layer.state.insert(conn.wire_id, r.steps, conn.weight)
        conn.layer = r.layer
        self.unrouted.discard(conn.wire_id)
        path = layer.paths[conn.wire_id] = path_from_steps(layer.pmap, r.steps)
        self._demand(conn, -1.0)
        for li, air in conn.airwire.items():
            for g in air.gates:
                self.layers[li].air_index[g].discard(conn.wire_id)
        self._weld(layer, conn, path)

    def rip(self, wire_id: int) -> None:
        """Remove a wire. The barrier is not updated (append-only); rebuild it per round."""
        conn = self.conns[wire_id]
        layer = self.layers[conn.layer]
        layer.state.remove(wire_id)
        del layer.paths[wire_id]
        conn.layer = None
        self.unrouted.add(wire_id)
        self._demand(conn, +1.0)
        for li, air in conn.airwire.items():
            for g in air.gates:
                self.layers[li].air_index[g].add(wire_id)

    def _weld(self, layer: Layer, conn: Connection, path: GatePath) -> None:
        pmap, state = layer.pmap, layer.state
        layer.barrier.weld(pmap.pad_obs[conn.src], pmap.pad_obs[conn.dst], path)
        for g in path.gates:
            if pmap.edge_kind_list[g] == GATE and state.full(g):
                u, v = pmap.edge_v_list[g]
                layer.barrier.weld(pmap.v_obs_list[u], pmap.v_obs_list[v], g)

    def rebuild_barrier(self) -> None:
        for layer in self.layers:
            layer.barrier = Barrier()
            for wire_id, path in layer.paths.items():
                self._weld(layer, self.conns[wire_id], path)

    def snapshot(self):
        return [layer.state.snapshot() for layer in self.layers]

    def restore(self, snap) -> None:
        """Restore every layer and recompute everything derived from the states."""
        for layer, s in zip(self.layers, snap):
            layer.state.restore(s)
            layer.paths = {w: path_from_steps(layer.pmap, steps) for w, steps in layer.state.wire_path.items()}
            layer.demand = [0.0] * layer.pmap.num_edges
            layer.air_index = {g: set() for g in layer.air_index}
        for c in self.conns.values():
            c.layer = next((l.index for l in self.layers if c.wire_id in l.state.wire_path), None)
        self.unrouted = {w for w, c in self.conns.items() if c.layer is None}
        for w in self.unrouted:
            c = self.conns[w]
            self._demand(c, +1.0)
            for li, air in c.airwire.items():
                for g in air.gates:
                    self.layers[li].air_index[g].add(w)
        self.rebuild_barrier()

    def report(self, phase: str, done: float = 0.0, total: float = 1.0) -> None:
        if self.stop is not None and self.stop.is_set():
            raise parallel.Stopped()
        if self.progress is not None:
            self.progress(phase, done, total)
        if self.events is not None:
            self.events({"type": "progress", "pass": self.pass_index, "variant": self.variant,
                         "phase": phase, "done": done, "total": total})
            self.emit_snapshot()

    def emit_snapshot(self, force: bool = False, lines: dict | None = None) -> None:
        """Sends the front end a picture of the routing as it stands: every
        wire as a rough polyline through the middles of the gates it crosses
        (or, once geometry exists, the real ``lines``), with a few numbers.
        At most a couple a second unless forced."""
        if self.events is None:
            return
        now = time.monotonic()
        if not force and now - self._last_snapshot < 0.4:
            return
        self._last_snapshot = now
        wires = []
        for layer in self.layers:
            mids, centre = layer.pmap.edge_mid_list, layer.pmap.pad_centre
            for w, path in layer.paths.items():
                conn = self.conns[w]
                if lines is not None and w in lines:
                    pts = lines[w]
                else:
                    pts = [centre[conn.src]] + [mids[g] for g in path.gates] + [centre[conn.dst]]
                wires.append((layer.index, conn.net_id, [(round(x, 3), round(y, 3)) for x, y in pts]))
        centre = {p.pad_id: p.centre for p in self.board.pads}
        self.events({
            "type": "snapshot", "pass": self.pass_index, "variant": self.variant, "final": lines is not None,
            "wires": wires,
            "open": [(centre[self.conns[w].src], centre[self.conns[w].dst]) for w in self.unrouted],
            "vias": [p.centre for p in self.board.pads if p.is_via],
            "routed": len(self.conns) - len(self.unrouted), "total": len(self.conns),
            "overflow": sum(len(l.state.overflowed_gates()) for l in self.layers),
            "rounds": self.rounds, "length": self.estimated_length(),
            "per_layer": {l.name: len(l.paths) for l in self.layers},
        })

    # -- metrics ------------------------------------------------------------
    def violations(self) -> int:
        return len(self.unrouted) + sum(l.state.overflow(e) for l in self.layers for e in l.state.overflowed_gates())

    def estimated_length(self) -> float:
        total = 0.0
        for layer in self.layers:
            mids = layer.pmap.edge_mid_list
            for path in layer.paths.values():
                g = path.gates
                total += sum(math.hypot(mids[a][0] - mids[b][0], mids[a][1] - mids[b][1]) for a, b in zip(g, g[1:]))
        return total
