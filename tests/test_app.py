"""The desktop application, driven without a person (Qt's offscreen platform)."""
import os
import subprocess
import sys

import pytest

pytest.importorskip("PySide6")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def run_app(*args, timeout=240):
    env = dict(os.environ, QT_QPA_PLATFORM="offscreen")
    return subprocess.run([sys.executable, os.path.join(ROOT, "main.py"), *args], cwd=ROOT, env=env,
                          capture_output=True, text=True, timeout=timeout)


def test_app_opens_routes_shows_and_exports(tmp_path):
    from tests.conftest import demo_board
    from weaveengine.io.check import read_session
    from weaveengine.io.dsn import write_dsn
    dsn, ses, png = (str(tmp_path / n) for n in ("board.dsn", "board.ses", "shot.png"))
    write_dsn(demo_board(), dsn)
    done = run_app("--self-test", dsn, "--ses", ses, "--screenshot", png)
    out = done.stdout
    assert done.returncode == 0, out + done.stderr
    assert "kernels:" in out                                  # the compiled-kernel status is shown before routing
    assert "routed 3/3" in out and "live picture shown: True" in out
    assert "-> OK" in out                                     # the exported session measures clean
    # view controls: zoom stays on the point under the cursor, in small steps, within limits; pan is exact
    assert "point under cursor drifted 0.00 px" in out and "pan error 0.00 px" in out
    assert "zoom limits 0.25x .. 400x of fit" in out
    # selection: a pad and a trace report what they are
    assert "'kind': 'pad'" in out and "'kind': 'trace'" in out and "'length':" in out and "'net total':" in out
    assert len(read_session(ses)[0]) >= 3
    assert os.path.getsize(png) > 10_000                      # a real window was drawn
    for tab in range(1, 5):                                   # the four tabs of the settings editor
        assert os.path.getsize(os.path.splitext(png)[0] + f"_settings{tab}.png") > 5_000


def test_app_warns_when_kernels_are_unavailable(tmp_path):
    from tests.conftest import demo_board
    from weaveengine.io.dsn import write_dsn
    dsn = str(tmp_path / "board.dsn")
    write_dsn(demo_board(), dsn)
    env = dict(os.environ, QT_QPA_PLATFORM="offscreen", WEAVEENGINE_NO_NUMBA="1")
    done = subprocess.run([sys.executable, os.path.join(ROOT, "main.py"), "--self-test", dsn], cwd=ROOT, env=env,
                          capture_output=True, text=True, timeout=240)
    assert done.returncode == 0, done.stdout + done.stderr   # no crash: it routes on the fallback
    assert "WARNING: running without the compiled kernels" in done.stdout
    assert "routed 3/3" in done.stdout


def test_app_reports_a_bad_file_without_crashing(tmp_path):
    bad = tmp_path / "bad.dsn"
    bad.write_text("this is not a board")
    done = run_app("--self-test", str(bad))
    assert done.returncode == 2 and "could not open" in done.stdout


def test_hidden_things_cannot_be_selected():
    """Design section 23: only what is switched on can be picked."""
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    from PySide6.QtWidgets import QApplication
    from weaveengine.app import BoardView
    from weaveengine.board import Board, Component, Pad, Rules
    app = QApplication.instance() or QApplication([])
    board = Board.rectangle(40.0, 30.0, Rules(), layers=["F.Cu", "B.Cu"])
    board.pads += [Pad.circle(0, 5, 5, 1.0, 1), Pad.rect(1, 30, 5, 2, 2, 2, layers=frozenset({0})),
                   Pad.rect(2, 30, 25, 2, 2, 2, layers=frozenset({1}))]
    board.components.append(Component("U1", outlines=[[(10, 10), (20, 10), (20, 20), (10, 20), (10, 10)]]))
    view = BoardView()
    view.resize(800, 600)
    view.set_board(board)
    wires = [(0, 1, [(5.0, 15.0), (35.0, 15.0)]), (1, 2, [(5.0, 22.0), (35.0, 22.0)])]
    view.show_routing(wires)
    seen = []
    view.selected.connect(seen.append)
    kind = lambda x, y: (view.pick(x, y) or {}).get("kind")

    assert kind(15, 12) == "part" and kind(25, 15) == "trace" and kind(30, 5) == "pad" and kind(30, 25) == "pad"
    # outlines off: the part is gone, and so is a selection of it; what lies under it is still found
    view.select_at(view.mapToScene(view.mapFromScene(15.0, -12.0)))
    assert seen[-1]["kind"] == "part"
    view.set_outlines_visible(False)
    assert seen[-1] is None and not view.selection_items
    assert kind(15, 12) is None and kind(15, 15) == "trace"
    view.set_outlines_visible(True)
    assert kind(15, 12) == "part"
    # front layer off: its trace and its surface pad are neither drawn nor picked; the rest stays
    view.select_at(view.mapToScene(view.mapFromScene(25.0, -15.0)))
    assert seen[-1]["kind"] == "trace"
    view.set_layer_visible(0, False)
    assert seen[-1] is None
    assert kind(25, 15) is None and kind(15, 15) == "part" and kind(30, 5) is None and kind(25, 22) == "trace"
    assert kind(5, 5) == "pad" and kind(30, 25) == "pad"      # through-hole, and a pad of the layer still shown
    shown = {pad.pad_id: item.isVisible() for item, pad in view.pad_items}
    assert shown == {0: True, 1: False, 2: True}
    view.set_layer_visible(1, False)                         # nothing shown: no pad at all
    assert kind(5, 5) is None and kind(30, 25) is None
    view.set_layer_visible(0, True)
    assert kind(30, 5) == "pad" and kind(25, 15) == "trace"
    app.processEvents()
