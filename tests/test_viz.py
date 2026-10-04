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


def test_progress_bar_output():
    import io
    from weaveengine.progress import ProgressBar, clock
    assert clock(75) == "1:15" and clock(3725) == "1:02:05"
    out = io.StringIO()
    bar = ProgressBar(out)
    for phase, done, total in (("pass 1: candidates", 0, 1), ("pass 1: commit", 5, 10), ("pass 1: rip-up", 1, 4),
                               ("pass 1: geometry", 0, 1), ("pass 2: candidates", 0, 1)):
        bar(phase, done, total)
    text = out.getvalue()
    assert "pass 1 [" in text and "commit" in text and "rough estimate for this pass" in text
    assert "pass 2 [" in text and " 16%" in text  # half-way through commit: 10% + half of 12%
    lines = [l for l in text.splitlines() if l.startswith("pass 1 [")]
    shares = [int(l.split("]")[1].split("%")[0]) for l in lines]
    assert shares == sorted(shares)  # the bar never moves backwards within a pass
