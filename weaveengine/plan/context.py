"""Shared routing context: layers, connections, demand map, airwire index, barrier (Phase 0)."""
import copy
import math
import time
from dataclasses import dataclass, field

import numpy as np
import shapely
from scipy.sparse.csgraph import minimum_spanning_tree

from weaveengine import parallel
from weaveengine.board import Board
from weaveengine.plan.path import locate
from weaveengine.topo import sites
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
    # A connection routed through vias (12.3) is made of pieces, one per layer
    # it runs on: each an ordinary connection of its own, from pad or via to
    # via or pad, with this one as its ``parent``. They come and go together.
    pieces: tuple[int, ...] = ()                     # wire ids of the pieces, in order
    sites: tuple[int, ...] = ()                      # pad ids of the vias between them
    parent: int | None = None

    @property
    def routed(self) -> bool:
        return self.layer is not None or bool(self.pieces)


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


DRAWN = 0.004  # mm: how far a drawn trace may be from the real one
ROUGH = 0.5    # of a pitch: the same for the rough picture while routing


def thin(lines: list, tolerance: float) -> list[list[tuple[float, float]]]:
    """The polylines with the points left out that matter less than ``tolerance`` (for drawing only)."""
    if not lines:
        return []
    sizes = [len(line) for line in lines]
    flat = np.array([p for line in lines for p in line], dtype=np.float64)
    geoms = shapely.simplify(shapely.linestrings(flat, indices=np.repeat(np.arange(len(lines)), sizes)), tolerance, preserve_topology=False)
    return [[(round(x, 3), round(y, 3)) for x, y in g.coords] for g in geoms]


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
    vias: bool = True               # routes may change layer through vias (12.3)
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
        self.next_wire = len(self.conns) + 1             # ids of the pieces of connections through vias; never reused
        self.next_pad = max((p.pad_id for p in board.pads), default=-1) + 1   # pad ids of via sites
        self._legal = None
        self._setup()

    # -- vias (12.3) -----------------------------------------------------------
    @property
    def vias(self) -> bool:
        return self.options.vias and len(self.layers) > 1

    @property
    def legal(self):
        """Where a via may be (``plan.path.Legal``); worked out once."""
        if self._legal is None:
            from weaveengine.plan.path import Legal
            self._legal = Legal(self)
        return self._legal

    def pad_centre(self, pad: int) -> tuple[float, float]:
        for layer in self.layers:
            if pad in layer.pmap.pad_centre:
                return layer.pmap.pad_centre[pad]
        raise KeyError(pad)

    def board_with_vias(self) -> Board:
        """The board with the vias as pads (for realisation, DRC and output)."""
        board = copy.copy(self.board)
        have = {p.pad_id for p in self.board.pads}
        board.pads = self.board.pads + [p for p in sites.via_pads(self.layers[0].pmap, self.board.rules) if p.pad_id not in have]
        return board

    def _maps_changed(self) -> None:
        """After via sites came or went: sizes of the per-gate tables, and the
        stored gate paths of the wires the change moved."""
        for layer in self.layers:
            layer.demand += [0.0] * (layer.pmap.num_edges - len(layer.demand))
            for w in layer.pmap.__dict__.pop("_moved_wires", ()):
                if w in layer.state.wire_path:
                    layer.paths[w] = path_from_steps(layer.pmap, layer.state.wire_path[w])

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
                    r = route(pmap, layer.state, c.src, c.dst, self.params, congestion=False, weight=c.weight, net=c.net_id)
                    c.demand_gates[li] = frozenset(r.gates) if r is not None else frozenset()
                    for g in c.demand_gates[li]:
                        layer.demand[g] += c.weight / len(c.layers)

    # -- bookkeeping shared by Phase 2 and 3 -------------------------------
    def _waiting(self, conn: Connection, waiting: bool) -> None:
        """Enters the connection among those still to be routed, or takes it
        out: the unrouted set, the demand map (9.2) and the airwire index (9.4)."""
        (self.unrouted.add if waiting else self.unrouted.discard)(conn.wire_id)
        sign = 1.0 if waiting else -1.0
        for li, gates in conn.demand_gates.items():
            demand = self.layers[li].demand
            for g in gates:
                demand[g] += sign * conn.weight / len(conn.layers)
        for li, air in conn.airwire.items():
            index = self.layers[li].air_index
            for g in air.gates:
                (index[g].add if waiting else index[g].discard)(conn.wire_id)

    def _lay(self, conn: Connection, li: int, steps) -> None:
        """Puts one wire into a layer's state."""
        layer = self.layers[li]
        layer.state.insert(conn.wire_id, steps, conn.weight, conn.net_id)
        conn.layer = li

    def _laid(self, conn: Connection) -> None:
        """Records a wire that is in its layer's state for good."""
        layer = self.layers[conn.layer]
        path = layer.paths[conn.wire_id] = path_from_steps(layer.pmap, layer.state.wire_path[conn.wire_id])
        self._weld(layer, conn, path)

    def commit(self, conn: Connection, how) -> bool:
        """Routes the connection as ``how`` says: a ``Route`` on one layer, or a
        ``Path``, which may go through vias. The vias' sites are made in the
        maps of every layer and the pieces put in exactly as planned.

        Returns False, with nothing changed, if a via turns out not to fit
        where it was planned: a trace may over-fill a gate for the rip-up to
        sort out, but a via is copper that has to fit among the wires beside it.
        """
        routes, vias = (how.routes, how.vias) if hasattr(how, "routes") else ([how], [])
        if not vias:
            self._lay(conn, routes[0].layer, routes[0].steps)
            self._laid(conn)
            self._waiting(conn, False)
            return True
        # Each piece was planned as if the others were not there. Two on the
        # same layer that use a gate in common might cross each other, and
        # their places on that gate are not known: such a plan is not used.
        used = [set() for _ in self.layers]
        for r in routes:
            if used[r.layer].intersection(r.gates):
                return False
            used[r.layer].update(r.gates)
        keep = sites.keep_off(self.board.rules)
        pads = [conn.src, *range(self.next_pad, self.next_pad + len(vias)), conn.dst]
        pieces = [Connection(self.next_wire + n, conn.net_id, pads[n], pads[n + 1], weight=conn.weight, parent=conn.wire_id)
                  for n in range(len(routes))]
        def mark():
            return [(len(sites.log(l.pmap)), len(sites.journal(l.state))) for l in self.layers]

        def back_to(marks) -> None:
            for l, m in zip(self.layers, marks):
                sites.undo(l.pmap, l.state, *m)

        before: list = []                                  # per via, the mark before its site was made
        made: list[list] = []                              # per via, its site on each layer
        split = [set() for _ in self.layers]               # triangles that have had a site put in them, per layer
        laid: list[Connection] = []
        settled = None
        try:
            for n, r in enumerate(routes):
                if n < len(vias):  # the via this piece ends on: its site, on every layer
                    point = vias[n]
                    where = [int(locate(l.pmap, np.array([point]))[0]) for l in self.layers]
                    if min(where) < 0 or not self.legal.exactly(*point):
                        raise ValueError("no via there")
                    before.append(mark())
                    here = [sites.create(l.pmap, l.state, t, point, pad=pads[n + 1], settle=False) for l, t in zip(self.layers, where)]
                    for l, site, t in zip(self.layers, here, where):
                        sites.set_net(l.pmap, l.state, site, conn.net_id, keep)
                        split[l.index].add(t)
                    made.append(here)
                layer = self.layers[r.layer]
                steps = list(r.steps)
                if any(step[1] in split[r.layer] for step in steps):
                    raise ValueError("the plan passes a triangle one of its own vias went into")
                if n > 0:
                    steps[:1] = sites.leave(layer.pmap, layer.state, made[n - 1][r.layer], steps[0])
                if n < len(vias):
                    steps += sites.arrive(layer.pmap, layer.state, made[n][r.layer], steps[-1])
                self._lay(pieces[n], r.layer, steps)
                laid.append(pieces[n])
            settled = mark()
            for here in made:
                for l, site in zip(self.layers, here):
                    sites.refresh(l.pmap, l.state, site.pad)
                    sites.legalise(l.pmap, l.state, site)
            if any(l.state.overflow(e) for here in made for l, site in zip(self.layers, here) for e in site.spokes):
                raise ValueError("a gate beside the via would be over-full")
        except ValueError:
            # Take everything out again, in the reverse of the order it went in:
            # the settling of the sites' edges, then each piece and the site made for it.
            if settled is not None:
                back_to(settled)
            for n in reversed(range(max(len(laid), len(before)))):
                if n < len(laid):
                    self.layers[laid[n].layer].state.remove(laid[n].wire_id)
                if n < len(before):
                    back_to(before[n])
            self._maps_changed()
            return False
        for l in self.layers:
            sites.journal(l.state).clear()
        self._maps_changed()
        self.next_wire += len(pieces)
        self.next_pad += len(vias)
        for piece in pieces:
            piece.air_len = math.dist(self.pad_centre(piece.src), self.pad_centre(piece.dst))
            self.conns[piece.wire_id] = piece
            self._laid(piece)
        conn.pieces, conn.sites = tuple(p.wire_id for p in pieces), tuple(pads[1:-1])
        self._waiting(conn, False)
        return True

    def rip(self, wire_id: int) -> None:
        """Removes the connection this wire belongs to: all its pieces, and its
        vias from the maps. The barrier is not updated (append-only); rebuild it per round."""
        conn = self.conns[wire_id]
        if conn.parent is not None:
            conn = self.conns[conn.parent]
        for w in conn.pieces or (conn.wire_id,):
            layer = self.layers[self.conns[w].layer]
            layer.state.remove(w)
            del layer.paths[w]
            if w != conn.wire_id:
                del self.conns[w]
        for pad in conn.sites:
            for layer in self.layers:
                site = layer.pmap.sites.get(pad)
                if site is not None:
                    sites.set_net(layer.pmap, layer.state, site, -1)
                    sites.delete(layer.pmap, layer.state, site)  # if it cannot be, it stays, asleep
        if conn.sites:
            self._maps_changed()
        conn.layer, conn.pieces, conn.sites = None, (), ()
        self._waiting(conn, True)

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
        """The routing as it stands: the states, how far each map's log of via
        sites had got, which sites are vias of which net, and the connections
        that are routed in pieces."""
        return {"states": [layer.state.snapshot() for layer in self.layers],
                "caps": [(layer.state.cap.copy(), layer.state.hist.copy()) for layer in self.layers],
                "logs": [len(sites.log(layer.pmap)) for layer in self.layers],
                "vias": {s.pad: (s.net, s.keep) for s in self.layers[0].pmap.sites.values() if s.active},
                "pieces": {w: (c.pieces, c.sites, [(self.conns[p].src, self.conns[p].dst) for p in c.pieces])
                           for w, c in self.conns.items() if c.pieces},
                "next": (self.next_wire, self.next_pad)}

    def restore(self, snap, logs: list | None = None) -> None:
        """Restore every layer and recompute everything derived from the states.
        The maps are taken back to where they were (sites made since are
        removed, sites deleted since are put back). ``logs``, per layer, are
        log entries to do again first: those of another copy of the maps, on
        which the snapshot was taken."""
        for i, (layer, s, length) in enumerate(zip(self.layers, snap["states"], snap["logs"])):
            if logs is not None:
                sites.replay(layer.pmap, logs[i][len(sites.log(layer.pmap)):])
            sites.rewind(layer.pmap, length)
            layer.pmap.__dict__.pop("_moved_wires", None)
            sites.journal(layer.state).clear()
            layer.state.restore(s)
            layer.state.cap, layer.state.hist = (x.copy() for x in snap["caps"][i])
            layer.state.resize()   # the map's tables may have grown since the snapshot (they never shrink)
            sites.clear_free(layer.pmap, layer.state)
            for site in layer.pmap.sites.values():
                site.net, site.keep = snap["vias"].get(site.pad, (-1, 0.0))
                layer.pmap.pad_net[site.pad] = site.net
            layer.paths = {w: path_from_steps(layer.pmap, steps) for w, steps in layer.state.wire_path.items()}
            layer.demand = [0.0] * layer.pmap.num_edges
            layer.air_index = {g: set() for g in layer.air_index}
        self.conns = {w: c for w, c in self.conns.items() if c.parent is None}
        self.next_wire, self.next_pad = snap["next"]
        for c in list(self.conns.values()):
            c.pieces, c.sites, ends = snap["pieces"].get(c.wire_id, ((), (), []))
            for w, (src, dst) in zip(c.pieces, ends):
                self.conns[w] = Connection(w, c.net_id, src, dst, weight=c.weight, parent=c.wire_id,
                                           air_len=math.dist(self.pad_centre(src), self.pad_centre(dst)))
        for c in self.conns.values():
            c.layer = next((l.index for l in self.layers if c.wire_id in l.state.wire_path), None)
        self.unrouted = set()
        for c in self.conns.values():
            if c.parent is None and not c.routed:
                self._waiting(c, True)
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
        wires, shapes = [], []
        for layer in self.layers:
            mids, centre = layer.pmap.edge_mid_list, layer.pmap.pad_centre
            for w, path in layer.paths.items():
                conn = self.conns[w]
                if lines is not None and w in lines:
                    pts = lines[w]
                else:
                    pts = [centre[conn.src]] + [mids[g] for g in path.gates] + [centre[conn.dst]]
                if len(pts) > 1:
                    wires.append((layer.index, conn.net_id))
                    shapes.append(pts)
        # A picture needs far fewer points than a route has gates (or an arc has
        # steps): drop those that move the line by less than can be seen.
        wires = [(layer, net, pts) for (layer, net), pts in zip(wires, thin(shapes, DRAWN if lines is not None else ROUGH * self.board.rules.pitch))]
        centre = self.pad_centre
        total = sum(1 for c in self.conns.values() if c.parent is None)
        self.events({
            "type": "snapshot", "pass": self.pass_index, "variant": self.variant, "final": lines is not None,
            "wires": wires,
            "open": [(centre(self.conns[w].src), centre(self.conns[w].dst)) for w in self.unrouted],
            "vias": [p.centre for p in self.board.pads if p.is_via]
                    + [s.centre for s in self.layers[0].pmap.sites.values() if s.active],
            "routed": total - len(self.unrouted), "total": total,
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
