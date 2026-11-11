"""The desktop application, driven without a person (Qt's offscreen platform)."""
import math
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
    bad.write_text("this is not a board", encoding="utf-8")
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


def test_no_via_areas_are_drawn_dotted():
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    from PySide6.QtCore import Qt
    from PySide6.QtWidgets import QApplication, QGraphicsPolygonItem
    from shapely.geometry import box
    from weaveengine.app import BoardView
    from weaveengine.board import Board, Rules
    app = QApplication.instance() or QApplication([])
    board = Board.rectangle(40.0, 30.0, Rules())
    board.no_via = [box(5, 5, 10, 10)]
    view = BoardView()
    view.set_board(board)
    hatched = [i for i in view.scene().items() if isinstance(i, QGraphicsPolygonItem) and i.brush().style() == Qt.Dense6Pattern]
    assert len(hatched) == 1


def test_skip_nets_setting_is_a_text_field_and_skips_by_glob(tmp_path):
    from tests.conftest import demo_board
    from weaveengine.io.dsn import read_dsn, write_dsn
    from weaveengine.settings import Settings, skip_nets
    dsn = str(tmp_path / "board.dsn")
    write_dsn(demo_board(), dsn)
    design = read_dsn(dsn)
    name = sorted(design.net_names.values())[0]
    before = len(design.board.nets())
    s = Settings(skip_nets=f" {name[:-1]}* , nothing ")
    assert "left unrouted by choice" in skip_nets(design, s.skip_nets.split(","))
    assert len(design.board.nets()) < before
    assert skip_nets(design, ["", " "]) == "" and not design.board.skip_nets
    path = str(tmp_path / "s.json")
    s.save(path)
    assert Settings.load(path).skip_nets == s.skip_nets


def test_cli_skip_net_leaves_the_net_out(tmp_path):
    from tests.conftest import demo_board
    from weaveengine.io.dsn import read_dsn, write_dsn
    dsn = str(tmp_path / "board.dsn")
    write_dsn(demo_board(), dsn)
    name = sorted(read_dsn(dsn).net_names.values())[0]
    done = subprocess.run([sys.executable, "-m", "weaveengine.cli", dsn, "--quiet", "--skip-net", name[:-1] + "*"],
                          cwd=ROOT, capture_output=True, text=True, timeout=240)
    assert "left unrouted by choice" in done.stdout and name in done.stdout, done.stdout + done.stderr
    assert f"unrouted: net {name}" not in done.stdout


def test_the_estimated_fill_is_drawn_per_layer():
    """Roadmap 1.1: a pour's pieces under the routing, the cut-off ones marked, hidden with their layer."""
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    from PySide6.QtWidgets import QApplication
    from shapely.geometry import box
    from weaveengine import pours
    from weaveengine.app import BoardView
    from weaveengine.board import Board, Pad, Rules
    app = QApplication.instance() or QApplication([])
    board = Board.rectangle(40.0, 30.0, Rules(), layers=["F.Cu", "B.Cu"])
    board.pads += [Pad.circle(0, 5, 5, 0.5, 0, "U1-1"), Pad.circle(1, 20, 15, 0.5, 0, "U1-2")]
    board.pours.append((0, 1, box(1, 1, 39, 29)))
    ring = [(20 + 3 * math.cos(a * math.pi / 16), 15 + 3 * math.sin(a * math.pi / 16)) for a in range(33)]
    view = BoardView()
    view.set_board(board)
    view.show_pours(pours.estimate(board, {0: ring}, {0: 1}, {0: 1}))
    assert len(view.pour_items) == 3 and any("cut off: U1-2" in i.toolTip() for i in view.pour_items)
    view.set_layer_visible(1, False)
    assert not view.pour_items


def test_pads_a_pour_does_not_reach_are_outlined_in_the_cut_off_colour():
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    from PySide6.QtWidgets import QApplication
    from shapely.geometry import box
    from weaveengine import pours
    from weaveengine.app import CUT_OFF_COLOR, BoardView
    from weaveengine.board import Board, Pad, Rules
    app = QApplication.instance() or QApplication([])
    board = Board.rectangle(40.0, 30.0, Rules(), layers=["F.Cu", "B.Cu"])
    board.pads += [Pad.circle(0, 5, 5, 0.5, 0, "U1-1"), Pad.circle(1, 20, 15, 0.5, 0, "U1-2")]
    board.pours.append((0, 1, box(1, 1, 39, 29)))
    ring = [(20 + 3 * math.cos(a * math.pi / 16), 15 + 3 * math.sin(a * math.pi / 16)) for a in range(33)]
    view = BoardView()
    view.set_board(board)
    view.show_pours(pours.estimate(board, {0: ring}, {0: 1}, {0: 1}))
    marks = [i for i in view.pour_items if i.pen().isCosmetic() and i.pen().color().name() == CUT_OFF_COLOR and i.pen().width() == 5]
    assert len(marks) == 1 and "U1-2" in marks[0].toolTip()   # (the cut-off pad only, not U1-1)
    view.set_layer_visible(1, False)
    assert not view.pour_items
    view.set_layer_visible(1, True)
    view.show_pour_fill = False
    view.show_pours(view.pour_shown)
    assert not view.pour_items


def test_the_warnings_panel_lists_what_is_wrong_with_the_board(tmp_path):
    from PySide6.QtWidgets import QApplication
    from tests.test_io import KICAD_STYLE, MARGIN_PCB
    from weaveengine.app import MainWindow
    app = QApplication.instance() or QApplication([])
    (tmp_path / "m.dsn").write_text(KICAD_STYLE.replace("(pcb demo.dsn", "(pcb m.dsn"), encoding="utf-8")
    (tmp_path / "m.kicad_pcb").write_text(MARGIN_PCB, encoding="utf-8")
    window = MainWindow(str(tmp_path / "settings.json"))
    window.interactive = False
    assert window.open(str(tmp_path / "m.dsn"))
    assert window.warnings_label.isHidden() is False
    text = window.warnings_label.text()
    assert "Warnings (" in text and "2 drawings on the Margin layer" in text
    window.show_warnings([])
    assert not window.warnings_label.isHidden() and window.warnings_label.text() == "No warnings"


def test_islands_are_hidden_unless_asked_for():
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    from PySide6.QtWidgets import QApplication
    from weaveengine.app import BoardView
    from weaveengine.settings import Settings
    from weaveengine.board import Board, Rules
    from weaveengine import pours
    from shapely.geometry import box
    app = QApplication.instance() or QApplication([])
    assert Settings().show_pour_islands is False and Settings().show_pours is True
    board = Board.rectangle(40.0, 30.0, Rules(), layers=["F.Cu", "B.Cu"])
    view = BoardView()
    view.set_board(board)
    pour = pours.Pour(0, 1, box(1, 1, 5, 5), [pours.Piece(box(1, 1, 5, 5), "island")])
    view.show_pours([pour])
    assert not view.pour_items
    view.show_pour_islands = True
    view.show_pours([pour])
    assert len(view.pour_items) == 1


def test_settings_has_a_pours_tab_and_search_filters_rows():
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    from PySide6.QtWidgets import QApplication
    from weaveengine.app import SettingsDialog
    from weaveengine.settings import Settings
    app = QApplication.instance() or QApplication([])
    dialog = SettingsDialog(Settings(), "settings.json")
    titles = [dialog.tabs.tabText(i) for i in range(dialog.tabs.count())]
    assert "Pours" in titles
    form, first, end = dialog.rows["pour_skeletons"]
    other, ofirst, _ = dialog.rows["clearance"]
    dialog.search.setText("KEEP POURS")
    assert form.isRowVisible(first) and not other.isRowVisible(ofirst)
    assert dialog.tabs.tabText(dialog.tabs.currentIndex()) == "Pours (1)"
    dialog.search.setText("")
    assert other.isRowVisible(ofirst) and "Pours" in [dialog.tabs.tabText(i) for i in range(dialog.tabs.count())]


def test_the_side_panel_can_shrink_to_a_thin_strip(tmp_path):
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    from PySide6.QtCore import Qt
    from PySide6.QtWidgets import QApplication
    from weaveengine.app import MainWindow
    app = QApplication.instance() or QApplication([])
    window = MainWindow(str(tmp_path / "settings.json"))
    dock = window.dock
    assert dock.minimumSizeHint().width() < 100 and dock.minimumWidth() < 100
    window.resize(1000, 700)
    window.show()
    window.resizeDocks([dock], [70], Qt.Horizontal)
    window.resize(900, 650)
    app.processEvents()
    assert window.width() == 900 and dock.width() < 100
    window.resizeDocks([dock], [400], Qt.Horizontal)
    app.processEvents()
    assert dock.width() > 300


def test_tapered_traces_are_drawn_at_the_width_of_each_piece():
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    from PySide6.QtWidgets import QApplication
    from weaveengine.app import BoardView
    from weaveengine.board import Board, Rules
    app = QApplication.instance() or QApplication([])
    board = Board.rectangle(40.0, 30.0, Rules(0.25, 0.2))
    view = BoardView()
    view.resize(800, 600)
    view.set_board(board)
    view.widths = {1: 0.2}   # a net narrowed (``router.reach``)
    pieces = [([(5.0, 15.0), (7.0, 15.0)], 0.2), ([(7.0, 15.0), (35.0, 15.0)], 0.25)]
    view.show_routing([(0, 1, [(5.0, 15.0), (7.0, 15.0), (35.0, 15.0)], pieces), (0, 1, [(5.0, 20.0), (35.0, 20.0)])])
    assert [round(w, 3) for _, w, _ in view.trace_items] == [0.2, 0.25, 0.2]   # the plain wire: the net's narrowed width
    seen = []
    view.selected.connect(seen.append)
    picked = view.pick(20.0, 15.0)   # a tapered trace (its wire has four items) can be picked
    assert picked["kind"] == "trace" and picked["net"] == "1" and picked["outline"] == [[(5.0, 15.0), (7.0, 15.0), (35.0, 15.0)]]
    view.show_routing(*view.last[:5], rough=True)
    assert all(w == 0 for _, w, _ in view.trace_items)
    view.show_routing(*view.last[:5])   # the final picture again: real widths, and pens to match at a zoom that shows them
    view.scale(40, 40)
    view._level_of_detail()
    assert all(item.pen().widthF() > 0 for item, _, _ in view.trace_items)
    app.processEvents()


def test_settings_search_keywords_words_and_typos():
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    from PySide6.QtWidgets import QApplication
    from weaveengine.app import SettingsDialog
    from weaveengine.settings import Settings
    app = QApplication.instance() or QApplication([])
    dialog = SettingsDialog(Settings(), "settings.json")

    def shown(text):
        dialog.search.setText(text)
        return {name for name, (form, first, _) in dialog.rows.items() if form.isRowVisible(first)}

    assert "taper_to_pins" in shown("neckdown")
    assert "teardrop_style" in shown("tear")
    assert "teardrop_style" in shown("teardorp")
    assert "teardrop_style" in shown("TEARDROP")
    both = shown("via size")
    assert {"via_diameter", "via_drill"} <= both and "clearance" not in both and both < shown("via")
    assert shown("size via") == both
    assert shown("") == set(dialog.rows) and shown("   ") == set(dialog.rows)
    assert shown("zzzzqq") == set()
