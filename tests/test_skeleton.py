"""Pours that stay whole (design 11.3): a skeleton in each pour keeps traces from cutting it."""
import numpy as np
from shapely.geometry import Point, box

from weaveengine import pours
from weaveengine.board import Board, Pad, Rules
from weaveengine.plan import skeleton
from weaveengine.plan.context import Options
from weaveengine.router import _layers, route_board

GND, SIG = 0, 1
FRONT = frozenset({0})


def bay(ground: tuple[float, float, float], top: tuple[float, float, float], layers=("F.Cu",)) -> Board:
    """A pour over the board, and a bay walled off by two keepouts with a pad of
    SIG at the end of each wall: the straight way between them closes the bay,
    and cuts off the ground pad ``ground`` (x, y, radius) in it from ``top``."""
    board = Board.rectangle(40, 30, Rules(0.2, 0.2), list(layers))
    board.net_names = {GND: "GND", SIG: "SIG"}
    board.pads += [Pad.circle(0, *ground, GND, "G", FRONT), Pad.circle(1, *top, GND, "H", FRONT),
                   Pad.circle(2, 17, 11, 0.6, SIG, "A", FRONT), Pad.circle(3, 23, 11, 0.6, SIG, "B", FRONT)]
    for x in (17, 23):  # (each wall stops 0.2 short of its pad: no trace and no fill gets between)
        board.add_keepout([(x - 0.5, 0), (x + 0.5, 0), (x + 0.5, 10.2), (x - 0.5, 10.2)], layers=FRONT)
    board.plane_nets.add(GND)
    board.pours += [(GND, at, box(0.5, 0.5, 39.5, 29.5)) for at in range(len(layers))]
    return board


def routed(board: Board, on: bool):
    result = route_board(board, options=Options(skeleton=on, portfolio=1), workers=1)
    return result, pours.of_result(result)


def test_the_skeleton_keeps_the_pad_in_the_bay_joined():
    result, [pour] = routed(bay((20, 9, 0.6), (20, 25, 0.6)), False)
    assert pour.cut_off == ["G"]  # the trace goes straight across the mouth of the bay
    result, [pour] = routed(bay((20, 9, 0.6), (20, 25, 0.6)), True)
    assert result.stats["routed"] == result.stats["connections"] == 1 and not result.violations
    assert result.stats["skeletons"] == result.stats["skeletons_routed"] == 1
    assert pour.cut_off == []  # the trace dips under the pad instead
    # The skeleton is not drawn: it is the pour's.
    assert result.polylines and all(result.wire_net[w] == SIG for w in result.polylines)


def test_where_the_skeleton_loses_a_via_takes_its_pad_to_the_plane_below():
    # The ground pads are against the board edge at each end: the skeleton would cut the board in two, the trace has to cross it.
    board = bay((20, 0.9, 0.5), (20, 29.1, 0.5), layers=("F.Cu", "B.Cu"))
    result, [front, _] = routed(board, True)
    assert result.stats["routed"] == 1 and result.stats["skeletons_routed"] == 0
    assert result.stats["fanout"] == 1  # (cut off on the front, it is joined to the plane on the back)
    assert [p for p in result.vias if p.fanout][0].net_id == GND
    assert front.cut_off == ["G"]


def test_without_the_option_there_are_no_skeletons():
    result, _ = routed(bay((20, 9, 0.6), (20, 25, 0.6)), False)
    assert result.stats["skeletons"] == 0


def test_a_skeleton_is_held_inside_its_pour():
    board = Board.rectangle(40, 30, Rules(0.2, 0.2), ["F.Cu", "B.Cu"])
    board.pads += [Pad.circle(0, 5, 5, 0.6, GND, "G1", FRONT), Pad.circle(1, 15, 5, 0.6, GND, "G2", FRONT),
                   Pad.circle(2, 35, 25, 0.6, GND, "G3", FRONT)]
    board.plane_nets.add(GND)
    outline = box(1, 1, 20, 10)
    board.pours.append((GND, 0, outline))
    layers = _layers(board, 1)
    assert skeleton.plan(board, layers) == [(GND, 0, 1, 0)]  # (G3 is outside the pour)
    skeleton.hold(board, layers, {GND})
    front, back = (skeleton.region(layer.pmap, GND) for layer in layers)
    pmap = layers[0].pmap
    inside = [outline.contains(Point(pmap.edge_mid_list[e])) for e in range(pmap.num_edges) if pmap.edge_kind_list[e] == 0]
    assert [bool(x) for x, kind in zip(front, pmap.edge_kind_list) if kind == 0] == inside
    assert not back.any() and skeleton.region(pmap, SIG) is None
    assert np.count_nonzero(front) < len(front)
