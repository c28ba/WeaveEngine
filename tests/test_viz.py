from tests.conftest import demo_board
from weaveengine.router import route_board
from weaveengine.viz.svg import export_result, export_svg


def test_svg_export(tmp_path):
    board = demo_board()
    result = route_board(board)
    out = tmp_path / "board.svg"
    export_svg(board, str(out), result.polylines, result.wire_net, pmap=result.pmap)
    text = out.read_text()
    assert text.startswith("<svg") and text.rstrip().endswith("</svg>")
    assert text.count("<title>wire") == 3
    assert text.count("<title>pad") == 6
    export_result(result, str(out))
    text = out.read_text()
    assert text.count("<title>wire") == 3 and text.count("<polygon") >= 1 + 1 + 6 + 6  # outline, keepout, pads, teardrops
