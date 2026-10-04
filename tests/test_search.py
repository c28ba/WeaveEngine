import random

from tests.conftest import grid_board
from weaveengine.topo import planar_map
from weaveengine.topo.costs import CostParams
from weaveengine.topo.planar_map import TERMINAL
from weaveengine.topo.search import route
from weaveengine.topo.state import TopoState


def reachable(pmap, state, src, dst) -> bool:
    """Brute-force oracle: plain graph search over (half-edge, slot) states
    with the section 8.2 rules written out from the raw tables."""
    seen, stack = set(), []
    for e in pmap.pad_edges[src]:
        t = pmap.edge_t_list[e][0]
        stack += [(e, t, p) for p in range(state.usage(e) + 1)]
    while stack:
        node = stack.pop()
        if node in seen:
            continue
        seen.add(node)
        a, t, p = node
        edges, verts = pmap.tri_e_list[t], pmap.tri_v_list[t]
        for b in edges:
            if b == a or pmap.edge_kind_list[b] == 1:
                continue
            c = pmap.shared_vertex(a, b)
            r = p if c == pmap.edge_v_list[a][0] else state.usage(a) - p
            if r > state.corner_cnt[t][verts.index(c)]:
                continue
            if pmap.edge_kind_list[b] == TERMINAL:
                if pmap.edge_owner_list[b] == dst:
                    return True
                continue
            pb = r if c == pmap.edge_v_list[b][0] else state.usage(b) - r
            ta, tb = pmap.edge_t_list[b]
            stack.append((b, tb if ta == t else ta, pb))
    return False


def test_route_is_planar_and_ends_on_the_right_pads(grid):
    _, pmap = grid
    state = TopoState(pmap)
    r = route(pmap, state, 0, 24)
    assert pmap.edge_owner_list[r.steps[0][0]] == 0
    assert pmap.edge_owner_list[r.steps[-1][0]] == 24
    assert all(pmap.edge_kind_list[s[0]] == 0 for s in r.steps[1:-1])
    assert len(set(r.gates)) == len(r.gates)
    # consecutive gates share the triangle named in the step
    for (a, _, _, _), (b, t, k, _) in zip(r.steps, r.steps[1:]):
        assert a in pmap.tri_e_list[t] and b in pmap.tri_e_list[t]
        assert pmap.tri_v_list[t][k] == pmap.shared_vertex(a, b)


def test_tiny_board_oracle():
    """17.1: the search against brute force on small boards.

    The oracle allows a path to cross a gate twice; the router must not (7.3).
    So: no path found by the oracle means none from the search, every path
    from the search is one the oracle agrees exists, and the cases where only
    the oracle finds one (paths that need a repeat crossing) stay rare.
    """
    rng = random.Random(3)
    blocked = found = only_oracle = 0
    for seed in range(6):
        board = grid_board(n=3, seed=seed)
        pmap = planar_map.build(board)
        state = TopoState(pmap)
        pads = sorted(pmap.pad_edges)
        for wire in range(1, 30):
            a, b = rng.sample(pads, 2)
            r = route(pmap, state, a, b)
            exists = reachable(pmap, state, a, b)
            if r is None:
                blocked += not exists
                only_oracle += exists
            else:
                assert exists
                found += 1
                state.insert(wire, r.steps)
        assert state.check_invariants()
    assert blocked > 0 and found > 50
    assert only_oracle <= 0.05 * (found + blocked + only_oracle)


def test_relaxed_search_names_the_blockers():
    """8.5: ripping up the blocking set of the relaxed path makes the connection routable."""
    rng = random.Random(11)
    checked = 0
    for seed in range(8):
        board = grid_board(n=3, seed=seed)
        pmap = planar_map.build(board)
        state = TopoState(pmap)
        params = CostParams.for_map(pmap)
        pads = sorted(pmap.pad_edges)
        for wire in range(1, 40):
            a, b = rng.sample(pads, 2)
            r = route(pmap, state, a, b, params)
            if r is not None:
                state.insert(wire, r.steps)
                continue
            relaxed = route(pmap, state, a, b, params, mode="relaxed")
            assert relaxed is not None
            if not relaxed.blocking:
                continue  # blocked only by the no-repeat-crossing rule, not by wires
            snap = state.snapshot()
            for w in relaxed.blocking:
                state.remove(w)
            assert route(pmap, state, a, b, params) is not None
            state.restore(snap)
            checked += 1
    assert checked > 0


def test_corridor_and_hard_capacity(grid):
    _, pmap = grid
    state = TopoState(pmap)
    r = route(pmap, state, 0, 24)
    replay = route(pmap, state, 0, 24, mode="corridor", corridor=set(r.gates))
    assert replay.gates == r.gates
    assert route(pmap, state, 0, 24, mode="corridor", corridor=set(r.gates[:-1])) is None
    for g in r.gates[1:-1]:
        state.cap[g] = 0
    blocked = route(pmap, state, 0, 24, hard_cap=True)
    assert blocked is None or not set(blocked.gates[1:-1]) & set(r.gates[1:-1])
