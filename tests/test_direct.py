"""Pads the user wants joined by a trace of their own (the ``direct_pairs`` setting, ``--connect``)."""
from weaveengine.board import Board, Pad, Rules
from weaveengine.plan.context import Layer, decompose
from weaveengine.topo import planar_map


def supply() -> Board:
    """A net of five pads in a row, 5 mm apart, and a capacitor's pad (5) beside the far end (pad 4)."""
    board = Board.rectangle(40, 12, Rules(0.2, 0.2))
    board.pads += [Pad.circle(i, 5 + 5 * i, 4, 0.6, 1, name=f"P{i}") for i in range(5)]
    board.pads.append(Pad.circle(5, 7, 8, 0.6, 1, name="C1-1"))  # nearest P0 and P1: the tree joins it there
    return board


def test_a_pair_to_join_directly_gets_a_connection_of_its_own():
    board = supply()
    layers = [Layer(0, "F.Cu", planar_map.build(board))]
    plain = {tuple(sorted((a, b))) for _, a, b in decompose(board, layers)[0]}
    assert (4, 5) not in plain and len(plain) == 5
    board.direct = [(5, 4)]
    pairs = {tuple(sorted((a, b))) for _, a, b in decompose(board, layers)[0]}
    assert (4, 5) in pairs and len(pairs) == 5          # still a tree: the capacitor to the far pin, the rest joined on


def test_a_pair_on_a_net_with_a_pour_is_routed_though_the_net_is_not():
    board = supply()
    board.plane_nets = {1}
    layers = [Layer(0, "F.Cu", planar_map.build(board))]
    assert decompose(board, layers)[0] == []
    board.direct = [(5, 4)]
    assert decompose(board, layers)[0] == [(1, 5, 4)]


def test_pairs_are_read_by_pad_name_and_what_cannot_be_paired_is_said():
    from types import SimpleNamespace
    from weaveengine.settings import direct_pairs
    board = supply()
    board.pads.append(Pad.circle(6, 30, 8, 0.6, 2, name="R1-1"))
    design = SimpleNamespace(board=board, net_names={1: "+3V3", 2: "SIG"})
    said = direct_pairs(design, ["C1-1:P4", " P4:C1-1", "C1-1:X9", "C1-1:R1-1", "C1-1", "P2:P2", ""])
    assert board.direct == [(5, 4)]                     # once, either way round
    assert said[0].startswith("1 pairs of pads joined directly") and "C1-1-P4" in said[0]
    assert said[1].startswith("WARNING") and "no pad X9" in said[1] and "on two nets: +3V3, SIG" in said[1] \
        and "C1-1 (not PAD:PAD)" in said[1] and "P2:P2 (one pad)" in said[1]


def test_the_command_line_routes_the_pair(tmp_path, monkeypatch):
    from weaveengine import cli
    from weaveengine.io.dsn import write_dsn
    got, real = [], cli.route_board
    monkeypatch.setattr(cli, "route_board", lambda *a, **k: got.append(real(*a, **k)) or got[-1])
    dsn, ses = str(tmp_path / "b.dsn"), str(tmp_path / "b.ses")
    write_dsn(supply(), dsn)  # (written so, the pads are named P<n>-1)
    cli.main([dsn, "-o", ses, "--quiet", "--workers", "1", "--connect", "P5-1:P4-1"])
    result = got[0]
    names = {p.pad_id: p.name for p in result.board.pads}
    joined = {frozenset((names[c.src], names[c.dst])) for w, c in result.connections.items() if w in result.polylines}
    assert frozenset(("P5-1", "P4-1")) in joined and result.stats["routed"] == result.stats["connections"] == 5
    # and it is a trace of its own: it rides no other copper of its net and keeps clear of the net's other pads
    from shapely.geometry import LineString
    w = next(w for w, c in result.connections.items() if w in result.polylines and {c.src, c.dst} == {4, 5})
    line = LineString(result.polylines[w])
    assert all(line.distance(p.shape) >= 0.2 + 0.1 - 1e-6 for p in result.board.pads if p.pad_id in (0, 1, 2, 3))


def test_the_app_pairs_two_pads_picked_by_hand(tmp_path):
    import os
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    from PySide6.QtCore import QPointF
    from PySide6.QtWidgets import QApplication
    from weaveengine.app import MainWindow
    from weaveengine.io.dsn import write_dsn
    app = QApplication.instance() or QApplication([])
    dsn = str(tmp_path / "b.dsn")
    write_dsn(supply(), dsn)
    window = MainWindow(str(tmp_path / "settings.json"))
    window.interactive = False
    assert window.open(dsn)
    window.view.select_at(QPointF(7, -8))               # the capacitor's pad (the scene's y is the board's, turned over)
    assert window.join_button.isVisibleTo(window)
    window.join_button.click()
    window.view.select_at(QPointF(25, -4))              # the far pin
    assert window.settings.direct_pairs == "P5-1:P4-1"   # (pads named as the DSN names them)
    assert "joined directly" in window.log.toPlainText()
    assert "P5-1" in window.selection_label.text()      # each pad says what it is joined to
    window.join_button.click()
    window.view.select_at(QPointF(1, -1))               # nothing there: cancelled, nothing added
    assert window.settings.direct_pairs == "P5-1:P4-1"
    # Kept for this board only: not in the settings file, none on another board, back when this one is opened again.
    import json
    from weaveengine.io.dsn import write_dsn as again
    window.settings.save(str(tmp_path / "settings.json"))
    assert "direct_pairs" not in json.load(open(tmp_path / "settings.json", encoding="utf-8"))
    other = str(tmp_path / "other.dsn")
    again(supply(), other)
    assert window.open(other) and window.settings.direct_pairs == ""
    assert window.open(dsn) and window.settings.direct_pairs == "P5-1:P4-1"
    while window.accel_status is None:  # (the window's check of the kernels, done before it goes: design 24 thread)
        app.processEvents()
