"""Pads and pour pieces joined to the rest of their net by the cheapest way (``plan.fanout``, design 11.4)."""
import math

from shapely.geometry import Point, box

from tests.test_pours import joined_after, ring
from weaveengine.board import Board, Pad, Rules
from weaveengine.plan.fanout import join, unjoined

GND, SIG, VCC = 0, 1, 2
FRONT, BACK = frozenset({0}), frozenset({1})


def poured(*layers) -> Board:
    """A board 40 by 30 with ground poured over it on ``layers``, and ground pads far off on the back, in its corners
    (the main piece: more pads than any other), through the board if it is poured on both sides."""
    board = Board.rectangle(40, 30, Rules(0.2, 0.2), ["F.Cu", "B.Cu"])
    board.net_names = {GND: "GND", SIG: "SIG", VCC: "VCC"}
    board.plane_nets.add(GND)
    board.pours += [(GND, at, box(1, 1, 39, 29)) for at in layers]
    board.pads += [Pad.circle(k, x, y, 0.6, GND, f"J{k}-1", None if len(layers) > 1 else BACK)
                   for k, (x, y) in enumerate([(5, 5), (35, 5), (5, 25), (35, 25)])]
    return board


def walled(board: Board, layer: int, radius: float = 3.0) -> tuple[dict, dict, dict]:
    """Three ground pads on the back in a ring of a signal's trace on ``layer`` round (20, 15)."""
    board.pads += [Pad.circle(10 + k, 19 + k, 15, 0.3, GND, f"C{k}-2", BACK) for k in range(3)]
    return {0: ring(20, 15, radius)}, {0: SIG}, {0: layer}


def test_a_cut_off_piece_is_stitched_to_the_plane_by_one_via_in_it():
    """Ground on both layers, joined by a pad through the board: the piece of the back's pour a ring of trace walls
    off, with three pads in it, gets one via in it, up to the front's pour."""
    board = poured(0, 1)
    lines, nets, layers = walled(board, 1)
    assert sorted(p.name for p in unjoined(board, lines, nets, layers)) == ["C0-2", "C1-2", "C2-2"]
    done = join(board, lines, nets, layers)
    assert done.kinds == {"stitch": 1} and done.traces == [] and not done.left
    assert math.dist(done.vias[0].centre, (20, 15)) < 3.0
    assert joined_after(board, lines, nets, layers, done.vias, done.traces) == []


def test_a_piece_with_no_plane_under_it_is_joined_by_a_jumper():
    """Ground only on the back (a two-layer board): the walled-off piece gets a via, a trace on the front over the
    ring, and a via down into the rest of the pour."""
    board = poured(1)
    lines, nets, layers = walled(board, 1)
    done = join(board, lines, nets, layers)
    assert done.kinds == {"jumper": 1} and len(done.vias) == 2 and not done.left
    [(net, layer, points)] = done.traces
    assert net == GND and layer == 0 and [math.dist(p, (20, 15)) < 3.0 for p in points] == [True, False]
    assert joined_after(board, lines, nets, layers, done.vias, done.traces) == []


def test_a_pad_with_no_room_beside_it_gets_a_via_in_it():
    """A ground pad on the front in a ring of trace too tight for a via beside it, the pour on the back: a via in
    the pad."""
    board = poured(1)
    board.pads.append(Pad.rect(10, 20, 15, 1.5, 1.5, GND, name="U1-1", layers=FRONT))
    lines, nets, layers = {0: ring(20, 15, 1.3)}, {0: SIG}, {0: 0}
    done = join(board, lines, nets, layers)
    assert done.kinds == {"via in pad": 1} and done.traces == [] and not done.left
    assert done.vias[0].shape.within(board.pads[-1].shape)


def test_a_join_that_would_cut_another_nets_pour_is_not_made():
    """A ground pad on the front between two traces, ground on the back, and VCC poured on an inner layer in two
    halves joined by a neck 0.6 mm wide under the pad and the traces: a via beside the pad, between the traces, would
    cut the neck (its hole in the fill there is a millimetre across). The via goes beyond the neck's end."""
    board = poured(1)
    board.unrouted = ["In1.Cu"]
    board.plane_nets.add(VCC)
    neck = box(18, 14.7, 22, 15.3)
    board.pours.append((VCC, 2, box(1, 1, 18, 29).union(box(22, 1, 39, 29)).union(neck)))
    board.pads += [Pad.circle(20, 5, 15, 0.5, VCC, "V1-1", frozenset({2})), Pad.circle(21, 35, 15, 0.5, VCC, "V2-1", frozenset({2})),
                   Pad.rect(10, 20, 15, 0.6, 0.6, GND, name="U1-1", layers=FRONT)]
    lines, nets, layers = {0: [(15, 14.1), (25, 14.1)], 1: [(15, 15.9), (25, 15.9)]}, {0: SIG, 1: SIG}, {0: 0, 1: 0}
    assert [p.name for p in unjoined(board, lines, nets, layers)] == ["U1-1"]
    done = join(board, lines, nets, layers)
    assert len(done.vias) == 1 and not done.left and abs(done.vias[0].centre[0] - 20) > 2.0
    assert joined_after(board, lines, nets, layers, done.vias, done.traces) == []


def starved() -> Board:
    """Ground poured on the front; a ground pad joined by a thermal relief whose four spokes (along its sides) end
    against pads of another net, while the fill comes close on its corners."""
    board = poured(0)
    for k in range(4):
        board.pads[k].layers = FRONT
    board.pads.append(Pad.rect(10, 20, 15, 1.0, 1.0, GND, name="U1-1", layers=FRONT))
    board.pads += [Pad.rect(20 + k, 20 + dx, 15 + dy, 0.8, 0.8, SIG, name=f"R{k}-1", layers=FRONT)
                   for k, (dx, dy) in enumerate([(1.6, 0), (-1.6, 0), (0, 1.6), (0, -1.6)])]
    return board


def test_a_starved_relief_is_changed_only_when_asked():
    board = starved()
    assert [p.name for p in unjoined(board)] == ["U1-1"]
    done = join(board, {}, {}, {}, reliefs=True)
    assert done.kinds == {"relief": 1} and not done.vias and not done.traces and not done.left
    assert list(done.reliefs) == [10] and done.reliefs[10] in ({"zone_connect": "solid"}, {"spoke_angle": 45.0})
    plain = join(board, {}, {}, {})
    assert not plain.reliefs and "relief" not in plain.kinds
