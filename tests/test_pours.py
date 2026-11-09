"""The estimate of a pour's fill (design 11.3): a ring of traces cuts off a pad."""
import math

import pytest

from shapely.geometry import Point, box

from weaveengine import pours
from weaveengine.board import Board, Pad, Rules

GND, SIG = 0, 1
BACK = frozenset({1})


def ring(x: float, y: float, r: float) -> list[tuple[float, float]]:
    return [(x + r * math.cos(a * math.pi / 16), y + r * math.sin(a * math.pi / 16)) for a in range(33)]


def board_with_pour() -> Board:
    board = Board.rectangle(40, 30, Rules(0.2, 0.2), ["F.Cu", "B.Cu"])
    board.net_names = {GND: "GND", SIG: "SIG"}
    board.pads += [Pad.circle(0, 8, 8, 0.6, GND, "U1-1", BACK), Pad.circle(1, 30, 8, 0.6, GND, "U1-2", BACK),
                  Pad.circle(2, 20, 22, 0.6, GND, "C3-2", BACK)]
    board.pours.append((GND, 1, box(1, 1, 39, 29)))
    return board


def test_a_pour_with_nothing_in_the_way_is_whole():
    board = board_with_pour()
    [pour] = pours.estimate(board, {}, {}, {})
    assert [p.kind for p in pour.pieces] == ["main"] and not pour.cut_off and pour.islands == 0
    assert pours.note(pour, board) == "GND on B.Cu in 1 piece"


def test_a_ring_of_traces_cuts_off_the_pad_inside_it():
    board = board_with_pour()
    lines, nets, layers = {0: ring(20, 22, 3)}, {0: SIG}, {0: 1}
    [pour] = pours.estimate(board, lines, nets, layers)
    assert pour.cut_off == ["C3-2"] and pour.islands == 0
    assert sorted(p.kind for p in pour.pieces) == ["cut", "main"]
    assert pours.note(pour, board) == "GND on B.Cu in 2 pieces: 1 pad cut off (C3-2)"
    cut = next(p for p in pour.pieces if p.kind == "cut")
    assert cut.shape.contains(Point(20, 22)) and cut.shape.area < math.pi * 3 ** 2


def test_a_ring_round_nothing_makes_an_island_and_other_layers_do_not_count():
    board = board_with_pour()
    lines, nets, layers = {0: ring(20, 12, 3), 1: ring(20, 22, 3)}, {0: SIG, 1: SIG}, {0: 1, 1: 0}
    [pour] = pours.estimate(board, lines, nets, layers)
    assert pour.cut_off == [] and pour.islands == 1
    assert pours.note(pour, board) == "GND on B.Cu in 2 pieces: 1 island"


def test_a_via_joins_a_cut_off_piece_to_the_pour_on_the_other_layer():
    board = board_with_pour()
    board.pours.append((GND, 0, box(1, 1, 39, 29)))
    board.pads += [Pad.circle(3, 20, 24, 0.4, GND, "", None, True), Pad.circle(4, 5, 25, 0.6, GND, "J1-1")]  # (a via, a pad through the board)
    ringed = {0: ring(20, 22, 3), 1: [(2, 15), (38, 15)]}      # (a trace of the net is no obstacle)
    [back, front] = pours.estimate(board, ringed, {0: SIG, 1: GND}, {0: 1, 1: 1})
    assert back.cut_off == [] and sorted(p.kind for p in back.pieces) == ["main", "via"]
    board.pads.pop(-2)                                         # without the via, nothing joins them
    assert pours.estimate(board, ringed, {0: SIG, 1: GND}, {0: 1, 1: 1})[0].cut_off == ["C3-2"]


def test_a_sliver_narrower_than_the_minimum_width_is_dropped():
    for top, pieces in [(1.2, 0), (1.5, 1)]:  # (the pour starts at 1: a strip 0.2 wide is left, or 0.5)
        board = board_with_pour()
        board.pads = []
        board.add_keepout([(0, top), (40, top), (40, 30), (0, 30)], layers=frozenset({1}))
        [pour] = pours.estimate(board, {}, {}, {})
        assert len(pour.pieces) == pieces


def test_fanout_puts_no_via_where_the_plane_has_no_fill():
    """A pad on the front has GND poured on the back, but a ring of another net's trace on the back cuts the
    pour round the pad off from the pad it joins: the pour's outline is there, its fill is not. No via."""
    from weaveengine.plan.fanout import fanout
    board = Board.rectangle(40, 30, Rules(0.2, 0.2), ["F.Cu", "B.Cu"])
    board.net_names = {GND: "GND", SIG: "SIG"}
    board.pads += [Pad.circle(0, 20, 15, 0.5, GND, "U1-1", frozenset({0})), Pad.circle(1, 5, 5, 0.6, GND, "J1-1", BACK)]
    board.plane_nets.add(GND)
    board.pours.append((GND, 1, box(1, 1, 39, 29)))
    vias, traces, left = fanout(board, {}, {}, {})
    assert len(vias) == 1 and not left        # (nothing in the way: a via into the pour)
    ringed = {0: ring(20, 15, 2.5)}
    vias, traces, left = fanout(board, ringed, {0: SIG}, {0: 1})
    assert vias == [] and traces == [] and [p.name for p in left] == ["U1-1"]


def test_a_zones_minimum_thickness_sets_the_slivers_its_fill_drops(tmp_path):
    """The .kicad_pcb zone's min_thickness is read per zone, and a fill is not narrower than it; the setting
    overrides it, and the skeleton is drawn 1.4 times as wide."""
    from weaveengine.io.kicad import read_kicad
    from weaveengine.plan import skeleton
    pcb = """(kicad_pcb (version 20240108) (generator "pcbnew")
  (layers (0 "F.Cu" signal) (31 "B.Cu" signal))
  (zone (net 1) (net_name "GND") (layer "B.Cu") (min_thickness 0.4) (fill yes)
    (polygon (pts (xy 0 0) (xy 10 0) (xy 10 10) (xy 0 10))))
  (zone (net 1) (net_name "GND") (layer "F.Cu") (fill yes)
    (polygon (pts (xy 0 0) (xy 10 0) (xy 10 10) (xy 0 10))))
)
"""
    (tmp_path / "z.kicad_pcb").write_text(pcb, encoding="utf-8")
    [(name, layer, _, least)] = read_kicad(str(tmp_path / "z.kicad_pcb")).thickness
    assert (name, layer, least) == ("GND", "B.Cu", 0.4)
    # A strip of pour 0.3 mm wide between two traces of another net: whole at 0.25, gone at 0.4.
    board = board_with_pour()
    lines = {0: [(1, 14.8), (39, 14.8)], 1: [(1, 15.7), (39, 15.7)]}   # (0.2 wide, 0.2 clear either side: leaves 0.3 of fill between)
    nets, layers = {0: SIG, 1: SIG}, {0: 1, 1: 1}
    thin = lambda: any(p.shape.contains(Point(30, 15.25)) for p in pours.estimate(board, lines, nets, layers)[0].pieces)
    assert board.pour_width(GND, 1) == 0.25 and thin()
    board.pour_min[(GND, 1)] = 0.4
    assert board.pour_width(GND, 1) == 0.4 and not thin()
    board.pour_override = 0.25
    assert board.pour_width(GND, 1) == 0.25 and thin()
    assert skeleton.width(board, GND) == pytest.approx(0.35)
    board.pour_override = 0.0
    assert skeleton.width(board, GND) == pytest.approx(0.56)
