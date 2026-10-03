from toporouter.topo.state import TopoState
from toporouter.topo.router import TopoRouter, Net
from toporouter.viz.svg import export_svg
from tests.test_router import create_two_triangle_map

def test_svg_export(tmp_path):
    pmap = create_two_triangle_map()
    state = TopoState(pmap)
    router = TopoRouter(pmap, state)

    nets = [
        Net(net_id=1, start_pad_id=10, target_pad_id=20, target_pos=(2.0, 0.5)),
        Net(net_id=2, start_pad_id=10, target_pad_id=20, target_pos=(2.0, 0.5)),
    ]
    assert router.route_all(nets) is True

    output_file = tmp_path / "test_output.svg"
    export_svg(pmap, state, str(output_file))

    assert output_file.exists()
    svg_content = output_file.read_text()
    assert "<svg" in svg_content
    assert "</svg>" in svg_content
    assert "polyline" in svg_content