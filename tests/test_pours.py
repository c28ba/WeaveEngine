"""The estimate of a pour's fill (design 11.3): a ring of traces cuts off a pad."""
import math

from shapely.geometry import Point, box

from weaveengine import pours
from weaveengine.board import Board, Pad, Rules

GND, SIG = 0, 1


def ring(x: float, y: float, r: float) -> list[tuple[float, float]]:
    return [(x + r * math.cos(a * math.pi / 16), y + r * math.sin(a * math.pi / 16)) for a in range(33)]


def board_with_pour() -> Board:
    board = Board.rectangle(40, 30, Rules(0.2, 0.2), ["F.Cu", "B.Cu"])
    board.net_names = {GND: "GND", SIG: "SIG"}
    board.pads += [Pad.circle(0, 8, 8, 0.6, GND, "U1-1"), Pad.circle(1, 30, 8, 0.6, GND, "U1-2"), Pad.circle(2, 20, 22, 0.6, GND, "C3-2")]
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


def test_a_via_joins_a_cut_off_piece_on_and_a_trace_of_the_net_is_no_obstacle():
    board = board_with_pour()
    board.pads.append(Pad.circle(3, 20, 24, 0.4, GND, "", None, True))
    [pour] = pours.estimate(board, {0: ring(20, 22, 3), 1: [(2, 15), (38, 15)]}, {0: SIG, 1: GND}, {0: 1, 1: 1})
    assert pour.cut_off == [] and sorted(p.kind for p in pour.pieces) == ["main", "via"]


def test_a_sliver_narrower_than_the_minimum_width_is_dropped():
    for top, pieces in [(1.2, 0), (1.5, 1)]:  # (the pour starts at 1: a strip 0.2 wide is left, or 0.5)
        board = board_with_pour()
        board.pads = []
        board.add_keepout([(0, top), (40, top), (40, 30), (0, 30)], layers=frozenset({1}))
        [pour] = pours.estimate(board, {}, {}, {})
        assert len(pour.pieces) == pieces
