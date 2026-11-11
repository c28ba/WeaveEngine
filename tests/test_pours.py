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


def test_fanout_keeps_clear_of_a_tapered_trace_as_drawn():
    """A ring of another net's trace round a pad on the front leaves room for a via inside it at its
    narrow width, and none where the trace is drawn wider in pieces (``realize.taper``)."""
    from weaveengine.plan.fanout import fanout
    board = Board.rectangle(40, 30, Rules(0.2, 0.2), ["F.Cu", "B.Cu"])
    board.net_names = {GND: "GND", SIG: "SIG"}
    board.pads += [Pad.circle(0, 20, 15, 0.5, GND, "U1-1", frozenset({0})), Pad.circle(1, 5, 5, 0.6, GND, "J1-1", BACK)]
    board.plane_nets.add(GND)
    board.pours.append((GND, 1, box(1, 1, 39, 29)))
    ringed = {0: ring(20, 15, 1.6)}
    vias, traces, left = fanout(board, ringed, {0: SIG}, {0: 0})
    assert len(vias) == 1 and not left
    vias, traces, left = fanout(board, ringed, {0: SIG}, {0: 0}, widths={0: [(ringed[0], 0.8)]})
    assert vias == [] and [p.name for p in left] == ["U1-1"]


def test_a_zones_minimum_thickness_sets_the_slivers_its_fill_drops(tmp_path):
    """The .kicad_pcb zone's min_thickness is read per zone, and a fill is not narrower than it; the setting
    overrides it, and the skeleton is routed a little wider (``skeleton.SLACK``)."""
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
    assert skeleton.width(board, GND) == pytest.approx(0.30)
    board.pour_override = 0.0
    assert skeleton.width(board, GND) == pytest.approx(0.45)


def test_a_zones_clearance_thermal_reliefs_and_priority_are_read(tmp_path):
    """From the .kicad_pcb: each zone's clearance, thermal gap and spoke width, how it joins its pads and its
    priority; and a pad's own way of joining a zone, or its part's."""
    from weaveengine.io.kicad import read_kicad
    pcb = """(kicad_pcb (version 20240108) (generator "pcbnew")
  (layers (0 "F.Cu" signal) (31 "B.Cu" signal))
  (footprint "R" (layer "F.Cu") (at 5 5) (zone_connect 2) (property "Reference" "R1")
    (pad "1" smd rect (at -1 0) (size 1 1) (layers "F.Cu"))
    (pad "2" smd rect (at 1 0) (size 1 1) (layers "F.Cu") (zone_connect 0) (thermal_gap 0.3)))
  (zone (net 1) (net_name "GND") (layer "B.Cu") (priority 2) (connect_pads (clearance 0.5)) (min_thickness 0.4)
    (fill yes (thermal_gap 0.6) (thermal_bridge_width 0.7))
    (polygon (pts (xy 0 0) (xy 10 0) (xy 10 10) (xy 0 10))))
  (zone (net 1) (net_name "GND") (layer "F.Cu") (connect_pads yes (clearance 0.3)) (fill yes)
    (polygon (pts (xy 0 0) (xy 10 0) (xy 10 10) (xy 0 10))))
)
"""
    (tmp_path / "z.kicad_pcb").write_text(pcb, encoding="utf-8")
    kicad = read_kicad(str(tmp_path / "z.kicad_pcb"))
    back, front = sorted(kicad.zones, key=lambda z: z[1])
    assert back[1] == "B.Cu" and (back[3].clearance, back[3].gap, back[3].spoke, back[3].connect, back[3].priority) == (0.5, 0.6, 0.7, "thermal", 2)
    assert (front[3].clearance, front[3].connect, front[3].priority) == (0.3, "solid", 0)
    one, two = kicad.pads
    assert (one.zone_connect, one.thermal_gap) == ("solid", None) and (two.zone_connect, two.thermal_gap) == ("none", 0.3)


def walled(board: Board, x: float, y: float, half: float) -> dict:
    """Four traces of SIG round (x, y), each ``half`` from it, square: a box with its corners open."""
    return {0: [(x - half, y - half + 1), (x - half, y + half - 1)], 1: [(x + half, y - half + 1), (x + half, y + half - 1)],
            2: [(x - half + 1, y - half), (x + half - 1, y - half)], 3: [(x - half + 1, y + half), (x + half - 1, y + half)]}


def test_a_pad_joined_by_a_thermal_relief_is_cut_off_where_no_spoke_reaches_the_fill():
    """The fill keeps the relief's gap from the pad and joins it only by spokes along its sides, whose ends must be
    in the fill: four traces close round the pad, each leaving room for the pour between it and the pad's
    clearance but not past the gap, starve every spoke. The fill still reaches round them by the corners."""
    from weaveengine.board import Zone
    board = board_with_pour()
    board.pads[2] = Pad.rect(2, 20, 22, 1.0, 1.0, GND, name="C3-2", layers=BACK)
    lines = walled(board, 20, 22, 1.3)  # (0.8 from the pad's sides: the trace and the clearance take 0.3 of it)
    nets, layers = {w: SIG for w in lines}, {w: 1 for w in lines}
    board.pour_zone[(GND, 1)] = Zone(gap=0.6, spoke=0.4)
    assert pours.estimate(board, lines, nets, layers)[0].cut_off == ["C3-2"]
    board.pour_zone[(GND, 1)] = Zone(gap=0.2, spoke=0.4)   # (a gap small enough for the spokes to land)
    assert pours.estimate(board, lines, nets, layers)[0].cut_off == []
    board.pour_zone[(GND, 1)] = Zone(gap=0.6, spoke=0.4, connect="solid")
    assert pours.estimate(board, lines, nets, layers)[0].cut_off == []
    board.pads[2].zone_connect = "none"  # (a pad the zone does not join is not one it cuts off)
    assert pours.estimate(board, lines, nets, layers)[0].cut_off == []


def test_a_zones_own_clearance_and_a_zone_of_higher_priority():
    from weaveengine.board import Zone
    board = board_with_pour()
    lines = {0: [(1, 14.8), (39, 14.8)], 1: [(1, 15.9), (39, 15.9)]}   # (0.2 wide: 0.5 of fill between at the board's 0.2)
    nets, layers = {0: SIG, 1: SIG}, {0: 1, 1: 1}
    strip = lambda: any(p.shape.contains(Point(30, 15.35)) for p in pours.estimate(board, lines, nets, layers)[0].pieces)
    assert strip()
    board.pour_zone[(GND, 1)] = Zone(clearance=0.3)  # (0.3 left: still wider than the minimum 0.25)
    assert strip()
    board.pour_zone[(GND, 1)] = Zone(clearance=0.4)
    assert not strip()
    # A pour of SIG over the right half, of higher priority: GND's fill keeps its clearance from SIG's fill.
    board = board_with_pour()
    board.pours.append((SIG, 1, box(25, 1, 39, 29)))
    board.pour_zone[(SIG, 1)] = Zone(priority=1)
    ground, signal = pours.estimate(board, {}, {}, {})
    assert ground.cut_off == ["U1-2"] and not ground.pieces[0].shape.intersects(box(25, 1, 39, 29))
    board.pour_zone[(SIG, 1)] = Zone(priority=0)  # (equal priority: neither gives way, as before)
    assert pours.estimate(board, {}, {}, {})[0].cut_off == []


def test_pads_whose_copper_meets_are_joined_whatever_the_pour():
    """A pad ringed off in the pour on the back is joined all the same where its copper meets, on the front, a pad
    of the net that the pour reaches: KiCad sees one piece of copper."""
    board = board_with_pour()
    board.pads[2] = Pad.circle(2, 20, 22, 0.6, GND, "C3-2")  # (through the board)
    board.pads.append(Pad.rect(3, 22.5, 22, 7.0, 1.0, GND, name="C3-3", layers=frozenset({0})))  # (on the front, under it)
    board.pads.append(Pad.circle(4, 26, 22, 0.6, GND, "C3-4"))  # (through the board, outside the ring)
    lines, nets, layers = {0: ring(20, 22, 3)}, {0: SIG}, {0: 1}
    assert pours.estimate(board, lines, nets, layers)[0].cut_off == []
    board.pads.pop()
    assert pours.estimate(board, lines, nets, layers)[0].cut_off == ["C3-2"]
