"""Choosing the pad edge a trace leaves through (post-processing, M8).

The topological search starts a wire on whichever edge of the pad's keep-off
ring gave the cheapest path, which is rarely the edge the taut trace would pick.
The realised trace then turns sharply at the ring. When relaxation reports an
end pressed against a corner of its pad edge, the wire is moved to the next
edge round the pad: the same route, entered one triangle later.
"""
from weaveengine.board import Board
from weaveengine.realize.relax import relax
from weaveengine.topo.planar_map import TERMINAL
from weaveengine.topo.state import Step, TopoState


def reversed_steps(steps: list[Step]) -> list[Step]:
    """The same wire described from its other end."""
    n = len(steps) - 1
    out = [(steps[n][0], -1, -1, steps[n][3])]
    for j in range(1, n + 1):
        out.append((steps[n - j][0], steps[n - j + 1][1], steps[n - j + 1][2], steps[n - j][3]))
    return out


def hop(state: TopoState, wire: int, at_start: bool, vertex: int) -> bool:
    """Move one end of ``wire`` to the pad edge on the other side of ``vertex``,
    a corner of its own pad's keep-off ring.

    The wire leaves its pad edge and may wrap ``vertex`` through some gates
    before turning away in a triangle T. The replacement starts on the pad edge
    beyond the corner and reaches T round the other side. Nothing else may lie
    between the wire and the corner, so no other wire's topology changes.
    Returns False (and changes nothing) if that does not hold or a gate would
    be over-filled.
    """
    pmap = state.map
    order, edge_v, kind = state.gate_order, pmap.edge_v_list, pmap.edge_kind_list
    # Present positions on every gate (stored slots date from insertion time).
    steps = [(e, t, k, order[e].index(wire)) for e, t, k, _ in state.wire_path[wire]]
    if not at_start:
        steps = reversed_steps(steps)
    pad = pmap.edge_owner_list[steps[0][0]]
    weight = state.weight[wire]

    # Leading gates round the corner: i of them, then the wire turns away in T.
    i = 0
    while i + 1 < len(steps) and pmap.tri_v_list[steps[i + 1][1]][steps[i + 1][2]] == vertex:
        i += 1
    if i + 1 >= len(steps):
        return False
    for edge, _, _, pos in steps[:i + 1]:
        if pos != (0 if edge_v[edge][0] == vertex else len(order[edge]) - 1):
            return False  # another wire lies between this one and the corner
    exit_gate, tri, _, exit_pos = steps[i + 1]
    entry = steps[i][0]

    # Walk the fan of triangles round the corner, away from the present route,
    # until the pad's other edge at this corner.
    fan: list[tuple[int, int]] = []  # (gate, triangle beyond it)
    here, came = tri, entry
    while True:
        edges = pmap.tri_e_list[here]
        if here != tri and state.corner_cnt[here][pmap.tri_v_list[here].index(vertex)] != 0:
            return False
        nxt = [x for x in edges if x != came and vertex in edge_v[x] and (here != tri or x != exit_gate)]
        if len(nxt) != 1:
            return False
        x = nxt[0]
        if kind[x] == TERMINAL:
            if pmap.edge_owner_list[x] != pad or not state.fits(x, weight):
                return False
            new_edge = x
            break
        ta, tb = pmap.edge_t_list[x]
        beyond = tb if ta == here else ta
        if kind[x] != 0 or beyond < 0 or len(fan) > 32 or any(x == st[0] for st in steps):
            return False
        if len(order[x]) >= 15 or not state.fits(x, weight):
            return False  # no room on this gate
        fan.append((x, beyond))
        here, came = beyond, x

    def at_corner(edge: int) -> int:
        return 0 if edge_v[edge][0] == vertex else len(order[edge])

    state.remove(wire)
    new = [(new_edge, -1, -1, at_corner(new_edge))]
    for gate, beyond in reversed(fan):
        new.append((gate, beyond, pmap.tri_v_list[beyond].index(vertex), at_corner(gate)))
    last = fan[0][0] if fan else new_edge
    new.append((exit_gate, tri, pmap.tri_v_list[tri].index(pmap.shared_vertex(last, exit_gate)), exit_pos))
    new += steps[i + 2:]
    state.insert(wire, new if at_start else reversed_steps(new), weight)
    return True


def straighten(state: TopoState, board: Board, rounds: int = 8) -> set[int]:
    """Hop wire ends round their pads until no end is pressed against a pad-edge
    corner (or nothing more can move). Returns the wires that were changed."""
    changed: set[int] = set()
    seen: dict[tuple[int, bool], set[int]] = {}  # pad edges each end has already tried
    frozen: set[tuple[int, bool]] = set()

    def edge_of(wire: int, at_start: bool) -> int:
        path = state.wire_path[wire]
        return path[0][0] if at_start else path[-1][0]

    for _ in range(rounds):
        report: dict = {"detect_only": True}
        relax(state, board, max_sweeps=40, arcs=False, slide=1, report=report)  # a rough pass is enough to see which ends are pressed
        moved = False
        for wire, at_start, vertex in report.get("clamped", []):
            key = (wire, at_start)
            if key in frozen or wire not in state.wire_path:
                continue
            seen.setdefault(key, set()).add(edge_of(wire, at_start))
            if not hop(state, wire, at_start, vertex):
                continue
            if edge_of(wire, at_start) in seen[key]:
                # Back where it has been: the end wants to sit on the corner itself. Leave it.
                frozen.add(key)
                continue
            changed.add(wire)
            moved = True
        if not moved:
            break
    return changed
