import random

import pytest

from weaveengine.topo.search import route
from weaveengine.topo.state import TopoState


def test_insert_remove_counts(grid):
    _, pmap = grid
    state = TopoState(pmap)
    r = route(pmap, state, 0, 24)
    state.insert(1, r.steps)
    assert state.check_invariants()
    assert all(state.usage(g) == 1 for g in r.gates)
    assert sum(sum(c) for c in state.corner_cnt) == len(r.steps) - 1
    with pytest.raises(ValueError):
        state.insert(1, r.steps)
    state.remove(1)
    assert all(not o for o in state.gate_order)
    assert all(c == [0, 0, 0] for c in state.corner_cnt)


def test_path_may_not_cross_a_gate_twice(grid):
    _, pmap = grid
    state = TopoState(pmap)
    r = route(pmap, state, 0, 24)
    with pytest.raises(ValueError):
        state.insert(1, r.steps + [r.steps[1]])


def test_invariants_detect_a_crossing(grid):
    _, pmap = grid
    state = TopoState(pmap)
    for wire, (a, b) in enumerate([(0, 24), (1, 23)], 1):
        state.insert(wire, route(pmap, state, a, b).steps)
    shared = next(g for g, o in enumerate(state.gate_order) if len(o) == 2)
    state.gate_order[shared].reverse()  # swap two wires on one gate only
    assert not state.check_invariants()


def test_snapshot_restore(grid):
    _, pmap = grid
    state = TopoState(pmap)
    state.insert(1, route(pmap, state, 0, 24).steps)
    snap = state.snapshot()
    state.insert(2, route(pmap, state, 4, 20).steps)
    state.restore(snap)
    assert set(state.wire_path) == {1} and state.check_invariants()


def test_random_insert_remove_keeps_invariants(grid):
    """M2 acceptance (17.1): random inserts and removes via the search, invariants after every one."""
    _, pmap = grid
    state = TopoState(pmap)
    rng = random.Random(7)
    pads = sorted(pmap.pad_edges)
    live, wire, ops, peak = [], 0, 0, 0
    while ops < 3000:
        if live and (rng.random() < 0.45 or len(live) > 30):
            state.remove(live.pop(rng.randrange(len(live))))
        else:
            a, b = rng.sample(pads, 2)
            r = route(pmap, state, a, b)
            if r is None:
                continue
            wire += 1
            state.insert(wire, r.steps)
            live.append(wire)
        ops += 1
        assert state.invariant_errors() == [], f"after {ops} operations"
        peak = max(peak, max(len(o) for o in state.gate_order))
    assert peak > 2  # the test did exercise shared gates


def test_a_wire_between_two_of_one_net_splits_their_run():
    """Neighbours of one net on a gate are one trace and count once; a foreign
    wire put in between makes them two, and the load has to say so."""
    from weaveengine.board import Board, Pad
    from weaveengine.topo import planar_map
    from weaveengine.topo.state import TopoState
    board = Board.rectangle(10, 10)
    board.pads += [Pad.circle(0, 2, 5, 0.3), Pad.circle(1, 8, 5, 0.3)]
    pmap = planar_map.build(board)
    state = TopoState(pmap)
    e = next(i for i, k in enumerate(pmap.edge_kind_list) if k == 0)
    t, side = pmap.edge_t_list[e][0], 0
    k = next(c for c in range(3) if pmap.tri_e_list[t][c] != e)  # any corner: only the gate's row and load matter here
    state.insert(1, [(e, -1, -1, 0)], 1.0, net=7)
    state.insert(2, [(e, -1, -1, 1)], 1.0, net=7)
    assert state.load[e] == state.tally(e) == 1.0      # one trace
    state.insert(3, [(e, -1, -1, 1)], 1.0, net=8)    # between them
    assert state.load[e] == state.tally(e) == 3.0      # now three
    state.remove(3)
    assert state.load[e] == state.tally(e) == 1.0
