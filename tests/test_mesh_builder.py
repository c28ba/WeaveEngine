from toporouter.geom.mesh_builder import build_planar_map_from_points
from toporouter.topo.state import TopoState
from toporouter.topo.router import TopoRouter, Net

def test_mesh_builder_delaunay():
    points = [(0.0, 0.0), (2.0, 0.0), (1.0, 2.0), (3.0, 2.0), (1.0, 0.5)]
    pad_owners = {0: 10, 3: 20}

    pmap = build_planar_map_from_points(
        points=points,
        pad_owners=pad_owners,
        trace_width=0.2,
        clearance=0.2
    )

    assert pmap.num_vertices == 5
    assert pmap.num_triangles >= 3
    assert len(pmap.edge_cap_list) == pmap.num_edges

    state = TopoState(pmap)
    router = TopoRouter(pmap, state)

    nets = [
        Net(net_id=1, start_pad_id=10, target_pad_id=20, target_pos=(3.0, 2.0)),
    ]

    success = router.route_all(nets)
    assert success is True
    assert state.check_invariants() is True

def test_complex_geometry_routing():
    points = [
        (0.5, 0.5), (4.5, 0.5), (0.5, 3.5), (4.5, 3.5),
        (2.5, 0.2), (2.5, 3.8), (1.5, 1.5), (3.5, 1.5),
        (1.5, 2.5), (3.5, 2.5), (2.5, 2.0), (2.5, 1.0)
    ]
    pad_owners = {0: 101, 1: 102, 2: 201, 3: 202, 4: 301, 5: 302}

    pmap = build_planar_map_from_points(points, pad_owners, trace_width=0.15, clearance=0.15)
    state = TopoState(pmap)
    router = TopoRouter(pmap, state)

    nets = [
        Net(net_id=1, start_pad_id=101, target_pad_id=102, target_pos=(4.5, 0.5)),
        Net(net_id=2, start_pad_id=201, target_pad_id=202, target_pos=(4.5, 3.5)),
        Net(net_id=3, start_pad_id=301, target_pad_id=302, target_pos=(2.5, 3.8)),
    ]

    assert router.route_all(nets) is True
    assert state.check_invariants() is True