"""The estimate of a pour's fill (design 11.3): a ring of traces cuts off a pad."""
import math

import pytest

import shapely
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


def joined_after(board, lines, wire_net, wire_layer, vias, traces, widths=None) -> list:
    """The pads of plane nets still not joined to the rest of their net with fan-out's ``vias`` and ``traces``."""
    from weaveengine.plan.fanout import unjoined
    import copy
    after = copy.copy(board)
    after.pads = board.pads + vias
    lines, wire_net, wire_layer = dict(lines), dict(wire_net), dict(wire_layer)
    for w, (net, layer, pts) in enumerate(traces, start=max(lines, default=0) + 1):
        lines[w], wire_net[w], wire_layer[w] = pts, net, layer
    return [p.name for p in unjoined(after, lines, wire_net, wire_layer, widths=widths)]


def test_fanout_puts_no_via_where_the_plane_has_no_fill():
    """A pad on the front has GND poured on the back, but a ring of another net's trace on the back cuts the
    pour round the pad off from the pad it joins: the pour's outline is there, its fill is not. No via inside
    the ring alone (it would join nothing): a trace on the front to a via outside it; and a ring too wide for
    that, a via into the island in it and a jumper from there over the ring."""
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
    assert len(vias) == 1 and not left and math.dist(vias[0].centre, (20, 15)) > 2.5  # (outside the ring)
    assert traces[0][1] == 0  # (its trace on the front)
    ringed = {0: ring(20, 15, 6.0)}
    vias, traces, left = fanout(board, ringed, {0: SIG}, {0: 1})
    assert len(vias) == 3 and not left and joined_after(board, ringed, {0: SIG}, {0: 1}, vias, traces) == []
    assert sum(math.dist(v.centre, (20, 15)) > 6.0 for v in vias) == 1  # (one outside the ring)


def test_fanout_keeps_clear_of_a_tapered_trace_as_drawn():
    """A ring of another net's trace round a pad on the front leaves room for a via inside it at its
    narrow width, and none where the trace is drawn wider in pieces (``realize.taper``): then the via goes in the
    pad."""
    from weaveengine.plan.fanout import fanout
    board = Board.rectangle(40, 30, Rules(0.2, 0.2), ["F.Cu", "B.Cu"])
    board.net_names = {GND: "GND", SIG: "SIG"}
    board.pads += [Pad.circle(0, 20, 15, 0.5, GND, "U1-1", frozenset({0})), Pad.circle(1, 5, 5, 0.6, GND, "J1-1", BACK)]
    board.plane_nets.add(GND)
    board.pours.append((GND, 1, box(1, 1, 39, 29)))
    ringed = {0: ring(20, 15, 1.6)}
    vias, traces, left = fanout(board, ringed, {0: SIG}, {0: 0})
    assert len(vias) == 1 and not left and math.dist(vias[0].centre, (20, 15)) < 1.6
    vias, traces, left = fanout(board, ringed, {0: SIG}, {0: 0}, widths={0: [(ringed[0], 0.8)]})
    assert not left and len(vias) == 1 and vias[0].shape.within(board.pads[0].shape.buffer(1e-6)) and traces == []  # (in the pad)
    assert joined_after(board, ringed, {0: SIG}, {0: 0}, vias, traces, {0: [(ringed[0], 0.8)]}) == []


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


def test_spokes_as_kicad_draws_them():
    """Four square-ended spokes from the pad's middle to its box grown by the gap, KiCad's error and half the fill's
    least width: along a pad's sides, turned with it, at 45 degrees on a round pad; no wider than the pad, and none
    narrower than the fill's least width."""
    from weaveengine.board import Zone
    zone, least = Zone(gap=0.5, spoke=0.3), 0.25
    grow = 0.5 + pours.ERROR + least / 2
    round_ = Pad.circle(0, 10, 10, 0.4, GND)
    ends = sorted((round(t.x - 10, 4), round(t.y - 10, 4)) for _, t in pours.spokes(round_, zone, least))
    d = round((0.4 + grow) / math.sqrt(2), 4)
    assert ends == sorted([(d, d), (-d, d), (-d, -d), (d, -d)])
    turned = Pad.rect(1, 10, 10, 2.0, 1.0, GND, rotation=30)
    turned.turn = 30.0
    tips = [t for _, t in pours.spokes(turned, zone, least)]
    a = math.radians(30)
    assert min(math.dist((t.x, t.y), (10 + (1 + grow) * math.cos(a), 10 + (1 + grow) * math.sin(a))) for t in tips) < 1e-6
    assert min(math.dist((t.x, t.y), (10 - (0.5 + grow) * math.sin(a), 10 + (0.5 + grow) * math.cos(a))) for t in tips) < 1e-6
    narrow = Pad.rect(2, 10, 10, 1.0, 0.2, GND)
    assert max(s.area for s, _ in pours.spokes(narrow, zone, 0.15)) == pytest.approx(0.2 * (0.5 + 0.5 + pours.ERROR + 0.075), rel=1e-3)
    assert pours.spokes(narrow, zone, least) == []  # (0.2 wide at most: narrower than the fill may be)


def test_pads_packed_closer_than_their_gaps_are_joined_only_by_spokes_that_meet():
    """Two pads in a box of traces with no fill inside, their reliefs overlapping: KiCad keeps a spoke whose end is
    in a spoke of the other pad that ends in it (facing each other, in line), not one that only ends in the other's
    relief."""
    from weaveengine.board import Zone
    for dy, together in ((0.0, True), (0.4, False)):
        board = board_with_pour()
        board.pads[2] = Pad.rect(2, 19.25, 22, 0.6, 0.6, GND, name="C3-1", layers=BACK)
        board.pads.append(Pad.rect(3, 20.75, 22 + dy, 0.6, 0.6, GND, name="C3-2", layers=BACK))  # (1.5 apart: a spoke's reach is 1.23)
        board.pour_zone[(GND, 1)] = Zone(gap=0.8, spoke=0.3)
        corners = [(18.4, 20.4), (21.6, 20.4), (21.6, 23.6), (18.4, 23.6)]  # (a closed box: the reliefs leave no fill inside)
        lines = {k: [corners[k], corners[(k + 1) % 4]] for k in range(4)}
        nets, layers = {w: SIG for w in lines}, {w: 1 for w in lines}
        [pour] = pours.estimate(board, lines, nets, layers)
        held = [set(x.pads) for x in pour.pieces if {"C3-1", "C3-2"} & set(x.pads)]
        assert sorted(pour.cut_off) == ["C3-1", "C3-2"]
        assert (held == [{"C3-1", "C3-2"}]) == together


def test_the_fill_keeps_the_boards_edge_clearance_and_fills_mask_openings():
    """The fill keeps the board's own copper-edge clearance from the edge, not what the traces were asked to keep;
    and a solder-mask opening, a rule for traces, is filled."""
    from weaveengine.board import Obstacle
    board = board_with_pour()
    board.rules.edge_clearance = 1.5
    reach = lambda: shapely.union_all([p.shape for p in pours.estimate(board, {}, {}, {})[0].pieces]).bounds[0]
    assert reach() == pytest.approx(1.5)
    board.pour_edge = 0.5
    assert reach() == pytest.approx(1.0)  # (the zone's own outline is 1 mm in)
    board.outline = box(0, 0, 40, 30).difference(Point(10, 15).buffer(1.0))  # (a hole in the board)
    gap = lambda: shapely.union_all([p.shape for p in pours.estimate(board, {}, {}, {})[0].pieces]).distance(Point(10, 15))
    assert gap() == pytest.approx(1.5, abs=0.01)
    board.obstacles.append(Obstacle(box(25, 10, 27, 12), -1, BACK, fill=False))
    assert any(p.shape.contains(Point(26, 11)) for p in pours.estimate(board, {}, {}, {})[0].pieces)
    # A hole with no copper is kept its hole clearance from, not the zone's clearance.
    from weaveengine.board import Zone
    board.pour_zone[(GND, 1)] = Zone(clearance=0.5)
    board.obstacles.append(Obstacle(Point(30, 20).buffer(1.0), -1, None, clearance=0.25, copper=True, hole=True))
    fill = shapely.union_all([p.shape for p in pours.estimate(board, {}, {}, {})[0].pieces])
    assert fill.distance(Point(30, 20)) == pytest.approx(1.25, abs=0.01)


def test_a_pour_keeps_the_clearance_of_its_nets_classes_from_each_net():
    board = board_with_pour()
    board.rules.clearance = 0.5  # (the most of any class)
    board.default_clearance = 0.2
    board.net_clearance = {SIG: 0.5, 5: 0.3}
    board.pads += [Pad.circle(3, 10, 20, 0.6, SIG, "R1-1", BACK), Pad.circle(4, 30, 20, 0.6, 5, "R2-1", BACK),
                   Pad.circle(5, 20, 4, 0.6, 6, "R3-1", BACK)]
    fill = shapely.union_all([p.shape for p in pours.estimate(board, {}, {}, {})[0].pieces])
    assert fill.distance(Point(10, 20)) == pytest.approx(0.5 + 0.6, abs=0.01)  # (the signal's class asks 0.5)
    assert fill.distance(Point(30, 20)) == pytest.approx(0.3 + 0.6, abs=0.01)  # (a class of 0.3)
    assert fill.distance(Point(20, 4)) == pytest.approx(0.2 + 0.6, abs=0.01)   # (the default class: 0.2)


def test_islands_the_zone_removes_are_not_counted():
    from weaveengine.board import Zone
    board = board_with_pour()
    lines, nets, layers = {0: ring(20, 12, 3)}, {0: SIG}, {0: 1}
    assert pours.estimate(board, lines, nets, layers)[0].islands == 1  # (not known: shown)
    board.pour_zone[(GND, 1)] = Zone(islands=0)
    assert pours.estimate(board, lines, nets, layers)[0].islands == 0
    board.pour_zone[(GND, 1)] = Zone(islands=2, island_area=10.0)  # (the island is about 21 mm2)
    assert pours.estimate(board, lines, nets, layers)[0].islands == 1
    board.pour_zone[(GND, 1)] = Zone(islands=2, island_area=30.0)
    assert pours.estimate(board, lines, nets, layers)[0].islands == 0
    board.pads = []  # (a zone joined to nothing at all is left whole, as KiCad leaves it)
    assert pours.estimate(board, lines, nets, layers)[0].islands == 2


def test_a_zones_island_removal_and_a_pads_angle_and_spoke_angle_are_read(tmp_path):
    from weaveengine.io.kicad import read_kicad
    pcb = """(kicad_pcb (version 20240108) (generator "pcbnew")
  (layers (0 "F.Cu" signal) (31 "B.Cu" signal))
  (footprint "R" (layer "F.Cu") (at 5 5 90) (property "Reference" "R1")
    (pad "1" smd rect (at -1 0 90) (size 1 1) (layers "F.Cu") (thermal_bridge_angle 45))
    (pad "2" smd rect (at 1 0 120) (size 1 1) (layers "F.Cu")))
  (zone (net 1) (net_name "GND") (layer "B.Cu") (fill yes (island_removal_mode 2) (island_area_min 4))
    (polygon (pts (xy 0 0) (xy 10 0) (xy 10 10) (xy 0 10))))
  (zone (net 1) (net_name "GND") (layer "F.Cu") (fill yes)
    (polygon (pts (xy 0 0) (xy 10 0) (xy 10 10) (xy 0 10))))
)
"""
    (tmp_path / "z.kicad_pcb").write_text(pcb, encoding="utf-8")
    kicad = read_kicad(str(tmp_path / "z.kicad_pcb"))
    back, front = sorted(kicad.zones, key=lambda z: z[1])
    assert (back[3].islands, back[3].island_area, front[3].islands) == (2, 4.0, 0)  # (KiCad's default: every island removed)
    one, two = kicad.pads
    assert (one.turn, one.spoke_angle, two.turn, two.spoke_angle) == (90.0, 45.0, 120.0, None)


def test_one_open_item_for_each_group_of_pads_cut_off_and_pieces_joined_by_any_copper_of_the_net():
    """The open items are counted as KiCad counts them, one per group of pads cut off together; a trace of the net from
    a cut-off piece to the main one joins it, as KiCad sees."""
    board = board_with_pour()
    board.pads.append(Pad.circle(3, 21.2, 22, 0.4, GND, "C3-1", BACK))  # (two pads in the one ring)
    lines, nets, layers = {0: ring(20.6, 22, 3)}, {0: SIG}, {0: 1}
    found = pours.estimate(board, lines, nets, layers)
    assert sorted(found[0].cut_off) == ["C3-1", "C3-2"] and pours.unjoined(found) == [[2, 3]]
    board.pours.append((GND, 0, box(1, 1, 39, 29)))  # (a trace of the net on the front, from in the ring to the plane)
    board.pads += [Pad.circle(4, 20, 22, 0.3, GND, "", None, True), Pad.circle(5, 30, 18, 0.3, GND, "", None, True)]
    lines, nets, layers = {0: ring(20.6, 22, 3), 1: [(20, 22), (30, 18)]}, {0: SIG, 1: GND}, {0: 1, 1: 1}
    found = pours.estimate(board, lines, nets, layers)
    assert not found[0].cut_off and pours.unjoined(found) == []


def test_a_zone_that_is_not_a_pour_takes_room_from_the_pours_it_outranks():
    """A teardrop, or copper on one pad of a routed net, is a zone KiCad fills too: of a higher priority, a pour keeps
    clear of it."""
    from weaveengine.board import Zone
    board = board_with_pour()
    board.areas.append((SIG, 1, box(14, 1, 15, 29)))  # (a strip across the board, above the ground pour)
    board.pour_zone[(SIG, 1)] = Zone(priority=5)
    [pour] = pours.estimate(board, {}, {}, {})
    assert len(pour.pieces) == 2 and not pour.pieces[0].shape.intersects(box(14, 1, 15, 29))
    board.pour_zone[(SIG, 1)] = Zone(priority=0)  # (below it: the pour fills first, and the strip keeps clear)
    [pour] = pours.estimate(board, {}, {}, {})
    assert len(pour.pieces) == 1


def test_a_pad_whose_spokes_reach_only_fill_that_meets_nothing_else_is_starved_once_some_of_it_is_joined():
    """KiCad's check of thermal reliefs: a spoke into fill that meets nothing but its pad counts only if no other
    spoke does, so joining one of four such stretches leaves the pad one spoke, and starved."""
    from weaveengine.board import Zone
    board = board_with_pour()
    pad = Pad(1, GND, box(19.5, 14.5, 20.5, 15.5), "C3-2", BACK)  # (square: four spokes, along its sides)
    board.pads = [Pad.circle(0, 8, 8, 0.6, GND, "U1-1", BACK), pad]
    board.pour_zone[(GND, 1)] = Zone(gap=0.3, spoke=0.3, connect="thermal")
    walls = [[(17, 12), (23, 12), (23, 18), (17, 18), (17, 12)]]  # (a box round the pad, cut in four from its corners)
    walls += [[(20 + 0.65 * dx, 15 + 0.65 * dy), (20 + 3 * dx, 15 + 3 * dy)] for dx in (-1, 1) for dy in (-1, 1)]
    lines = dict(enumerate(walls))
    nets, layers = {w: SIG for w in lines}, {w: 1 for w in lines}
    wires = pours.copper_of(board, lines, nets, layers)
    fill_of, _ = pours.fills(board, {}, {}, {}, wires=wires)
    [pour] = pours.classify(board, fill_of, wires)
    assert not pours.starved(board, pour, pad, wires)  # (four stretches that meet nothing else: all count)
    joined = wires + [(shapely.LineString([(22, 15), (25, 15)]).buffer(0.1), GND, 1)]  # (a trace from one of them out)
    fill_of, _ = pours.fills(board, {}, {}, {}, wires=joined)
    [pour] = pours.classify(board, fill_of, joined)
    assert pours.starved(board, pour, pad, joined)
