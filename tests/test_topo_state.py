import numpy as np
from toporouter.geom.planar_map import PlanarMap
from toporouter.topo.state import TopoState
from toporouter.topo.search import slot_aware_search

def create_two_triangle_map() -> PlanarMap:
    vx = np.array([0.0, 1.0, 0.5, 2.0], dtype=np.float64)
    vy = np.array([0.0, 0.0, 1.0, 0.5], dtype=np.float64)
    v_obs = np.array([-1, -1, -1, -1], dtype=np.int32)

    tri_v = np.array([[0, 1, 2], [1, 3, 2]], dtype=np.int32)
    tri_n = np.array([[1, -1, -1], [0, -1, -1]], dtype=np.int32)
    # Opposite edges for T0 and T1
    tri_e = np.array([[1, 2, 0], [4, 1, 3]], dtype=np.int32)

    edge_v = np.array([[0, 1], [1, 2], [0, 2], [1, 3], [2, 3]], dtype=np.int32)
    edge_t = np.array([[0, -1], [0, 1], [0, -1], [1, -1], [1, -1]], dtype=np.int32)
    edge_len = np.array([1.0, 1.0, 1.0, 1.0, 1.0], dtype=np.float64)
    edge_mid = np.array([[0.5, 0.0], [0.75, 0.5], [0.25, 0.5], [1.5, 0.25], [1.25, 0.75]], dtype=np.float64)
    edge_cap = np.array([5, 5, 0, 5, 0], dtype=np.int32)
    edge_kind = np.array([2, 0, 1, 2, 1], dtype=np.int32)
    edge_owner = np.array([10, -1, -1, 20, -1], dtype=np.int32)

    return PlanarMap(
        num_vertices=4,
        num_triangles=2,
        num_edges=5,
        tri_v=tri_v,
        tri_n=tri_n,
        tri_e=tri_e,
        vx=vx,
        vy=vy,
        v_obs=v_obs,
        edge_v=edge_v,
        edge_t=edge_t,
        edge_len=edge_len,
        edge_mid=edge_mid,
        edge_cap=edge_cap,
        edge_kind=edge_kind,
        edge_owner=edge_owner,
    )

def test_topo_state_invariants():
    pmap = create_two_triangle_map()
    state = TopoState(pmap)

    # In T0 [0, 1, 2], vertex 1 is local corner 1
    # In T1 [1, 3, 2], vertex 1 is local corner 0
    path_w1 = [(0, 0, -1, 0), (1, 0, 1, 0), (3, 1, 0, 0)]
    state.insert(wire_id=1, path_steps=path_w1)

    assert state.check_invariants() is True
    assert state.corner_cnt[0][1] == 1
    assert state.corner_cnt[1][0] == 1

    path_w2 = [(0, 0, -1, 1), (1, 0, 1, 0), (3, 1, 0, 0)]
    state.insert(wire_id=2, path_steps=path_w2)

    assert state.check_invariants() is True
    assert state.corner_cnt[0][1] == 2

    state.remove(wire_id=1)
    assert state.check_invariants() is True
    assert state.corner_cnt[0][1] == 1

def test_slot_aware_search_spike():
    pmap = create_two_triangle_map()
    state = TopoState(pmap)

    path = slot_aware_search(pmap, state, start_pad_id=10, target_pad_id=20, target_pos=(2.0, 0.5))

    assert path is not None
    assert len(path) >= 2

    state.insert(wire_id=101, path_steps=path)
    assert state.check_invariants() is True