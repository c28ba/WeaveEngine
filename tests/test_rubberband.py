from toporouter.topo.state import TopoState
from toporouter.topo.router import TopoRouter, Net
from toporouter.geom.rubberband import realize_rubberband_path
from tests.test_router import create_two_triangle_map

def test_rubberband_path_generation():
    pmap = create_two_triangle_map()
    state = TopoState(pmap)
    router = TopoRouter(pmap, state)

    nets = [
        Net(net_id=1, start_pad_id=10, target_pad_id=20, target_pos=(2.0, 0.5)),
    ]
    assert router.route_all(nets) is True

    start_pos = (0.5, 0.0)
    target_pos = (2.0, 0.5)

    polyline = realize_rubberband_path(pmap, state, wire_id=1, start_pos=start_pos, target_pos=target_pos)

    assert len(polyline) >= 3
    assert polyline[0] == start_pos
    assert polyline[-1] == target_pos