"""Acute angles where copper of one net meets (``realize.acute``, design 13.3b)."""
import math

from shapely.geometry import LineString, Polygon, box

from weaveengine.board import Board, Pad, Rules
from weaveengine.realize.acute import blunt, corners, count, meetings


def _board() -> Board:
    return Board.rectangle(40.0, 40.0, Rules(trace_width=0.25, clearance=0.2))


def _ray(start, degrees: float, length: float) -> tuple[float, float]:
    a = math.radians(degrees)
    return (start[0] + length * math.cos(a), start[1] + length * math.sin(a))


def _found(board, lines, ends, layer_of=None):
    wire_net = {w: 1 for w in lines}
    wire_layer = layer_of or {w: 0 for w in lines}
    return meetings(board, lines, wire_net, wire_layer, ends)


def test_a_corner_into_the_copper_is_measured_on_its_empty_side():
    two = LineString([(0, 0), (10, 0)]).buffer(0.2).union(LineString([(0, 0), _ray((0, 0), 30, 10)]).buffer(0.2))
    found = corners(two, 90.0)
    assert len(found) == 1 and abs(found[0][1] - 30.0) < 2.0
    tee = LineString([(-5, 0), (5, 0)]).buffer(0.2).union(LineString([(0, 0), (0, 5)]).buffer(0.2))
    assert corners(tee, 90.0) == []  # (a branch square to its trunk)


def test_a_trace_into_its_pad_at_a_slant_leaves_it_square():
    board = _board()
    pad = Pad(1, 1, box(8, 10, 16, 11))  # long and thin: a slanting trace leaves through a long side
    far = Pad.circle(2, 25, 13.5, 0.5, 1)
    board.pads += [pad, far]
    lines = {7: [pad.centre, far.centre]}
    ends = {7: (1, 2)}
    before = _found(board, lines, ends)
    assert count(before)["pad"] >= 1
    new = blunt(board, lines, {7: 1}, {7: 0}, ends)
    assert 7 in new and new[7][0] == pad.centre and new[7][-1] == far.centre  # (still from pad to pad)
    assert count(_found(board, {7: new[7]}, ends))["pad"] == 0


def test_traces_leaving_a_via_at_a_slant_share_its_stub_and_branch_square():
    board = _board()
    via = Pad.circle(1, 20, 20, 0.3, 1, is_via=True)
    a, b = Pad.circle(2, *_ray((20, 20), 0, 8), 0.5, 1), Pad.circle(3, *_ray((20, 20), 35, 8), 0.5, 1)
    board.pads += [via, a, b]
    lines = {1: [via.centre, a.centre], 2: [via.centre, b.centre]}
    ends = {1: (1, 2), 2: (1, 3)}
    assert count(_found(board, lines, ends))["via"] >= 1
    new = blunt(board, lines, {1: 1, 2: 1}, {1: 0, 2: 0}, ends)
    assert new
    lines.update(new)
    assert count(_found(board, lines, ends))["via"] == 0
    for w, (p, q) in ends.items():  # (each still joins its two pads)
        pads = {x.pad_id: x for x in board.pads}
        assert {lines[w][0], lines[w][-1]} == {pads[p].centre, pads[q].centre}


def test_a_branch_off_a_shared_trunk_squares_off_it():
    board = _board()
    start = Pad.circle(1, 5, 20, 0.5, 1)
    a, b = Pad.circle(2, 35, 20, 0.5, 1), Pad.circle(3, *_ray((15, 20), 12, 15), 0.5, 1)
    board.pads += [start, a, b]
    lines = {1: [start.centre, a.centre], 2: [start.centre, (15, 20), b.centre]}
    ends = {1: (1, 2), 2: (1, 3)}
    assert count(_found(board, lines, ends))["tee"] >= 1
    lines.update(blunt(board, lines, {1: 1, 2: 1}, {1: 0, 2: 0}, ends))
    assert count(_found(board, lines, ends))["tee"] == 0


def test_nothing_is_redrawn_through_foreign_copper():
    board = _board()
    via = Pad.circle(1, 20, 20, 0.3, 1, is_via=True)
    a, b = Pad.circle(2, *_ray((20, 20), 0, 8), 0.5, 1), Pad.circle(3, *_ray((20, 20), 35, 8), 0.5, 1)
    wall = Pad(9, 2, Polygon([(20.6, 20.45), (27, 20.45), (27, 21.0), (20.6, 21.0)]))  # another net's, in the wedge
    board.pads += [via, a, b, wall]
    lines = {1: [via.centre, a.centre], 2: [via.centre, b.centre]}
    ends = {1: (1, 2), 2: (1, 3)}
    assert blunt(board, lines, {1: 1, 2: 1}, {1: 0, 2: 0}, ends) == {}


def test_traces_on_different_layers_through_a_via_count_once_as_through():
    board = Board.rectangle(40.0, 40.0, Rules(trace_width=0.25, clearance=0.2), layers=["F.Cu", "B.Cu"])
    via = Pad.circle(1, 20, 20, 0.3, 1, is_via=True)
    a, b = Pad.circle(2, *_ray((20, 20), 0, 8), 0.5, 1), Pad.circle(3, *_ray((20, 20), 40, 8), 0.5, 1)
    board.pads += [via, a, b]
    lines = {1: [via.centre, a.centre], 2: [via.centre, b.centre]}
    found = _found(board, lines, {1: (1, 2), 2: (1, 3)}, {1: 0, 2: 1})
    assert count(found) == {"pad": 0, "via": 0, "tee": 0, "fanout": 0, "bend": 0, "through": 1}


def test_a_pair_of_teardrops_on_a_short_trace_has_no_acute_waist():
    """A via beside a wide pad: the pad's teardrop, as wide as there is room
    for, would meet the via's at its waist at 84 degrees."""
    from weaveengine.realize.teardrop import WAIST, _angle, teardrops
    board = Board.rectangle(40.0, 40.0, Rules(trace_width=0.2, clearance=0.2))
    via, pad = Pad.circle(1, 10, 10, 0.3, 1, is_via=True), Pad(2, 1, box(10.711, 8.631, 13.715, 10.535))
    board.pads += [via, pad]
    one, two = teardrops(board, {1: [via.centre, (10.59, 10.023), pad.centre]}, {1: 1}, {1: (1, 2)})[1]
    assert one[2] == two[2] and one[3] == two[3]  # (made together: they meet in one line across the trace)
    assert min(_angle(one[2], one[1], two[1]), _angle(one[3], one[4], two[4])) >= WAIST
