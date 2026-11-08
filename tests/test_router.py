import itertools
import math
import os

import pytest
from shapely.geometry import LineString, Point, Polygon

from bench.generators import channel, random_grid
from tests.conftest import demo_board
from weaveengine.board import Board, Pad, Rules
from weaveengine.plan.context import Layer, Options, decompose
from weaveengine.realize.teardrop import tip_of
from weaveengine.router import route_board
from weaveengine.topo import planar_map


def assert_clean(board, result):
    """Independent oracle (17.1): no two traces on a layer intersect, clearances
    hold for each trace's own width, every trace joins the pads of its connection."""
    for layer in result.layers:
        assert layer.state.check_invariants()
        assert not layer.state.overflowed_gates()
    assert result.violations == []
    board, rules = result.board, result.board.rules
    pads = {p.pad_id: p for p in board.pads}
    lines = {w: LineString(p) for w, p in result.polylines.items()}
    half = {w: rules.width(result.wire_net[w]) / 2.0 for w in lines}
    for (a, la), (b, lb) in itertools.combinations(lines.items(), 2):
        if result.wire_layer[a] != result.wire_layer[b]:
            continue
        if result.wire_net[a] != result.wire_net[b]:  # traces of one net may touch and share a trunk
            assert not la.crosses(lb), f"wires {a} and {b} cross"
            assert la.distance(lb) >= rules.clearance + half[a] + half[b] - 1.1e-3
    for w, line in lines.items():
        layer = result.wire_layer[w]
        for pad in board.pads_on(layer):
            if pad.net_id != result.wire_net[w]:
                assert line.distance(pad.shape) >= rules.clearance + half[w] - 1.1e-3
        for obs in board.obstacles_on(layer):
            assert line.distance(obs.shape) >= rules.clearance + half[w] - 1.1e-3
        assert board.outline.exterior.distance(line) >= rules.clearance + half[w] - 1.1e-3
        conn = result.connections[w]
        assert pads[conn.src].on(layer) and pads[conn.dst].on(layer)
        assert math.dist(line.coords[0], pads[conn.src].centre) < 1e-6
        assert math.dist(line.coords[-1], pads[conn.dst].centre) < 1e-6


def net_is_connected(result, net_id: int) -> bool:
    """Every pad of the net is joined through traces and shared pads (vias included)."""
    pads = [p.pad_id for p in result.board.pads if p.net_id == net_id and not p.is_via]
    parent = {}

    def find(x):
        while parent.setdefault(x, x) != x:
            x = parent[x]
        return x

    for w in result.polylines:
        conn = result.connections[w]
        if conn.net_id == net_id:
            parent[find(conn.src)] = find(conn.dst)
    return len({find(p) for p in pads}) == 1


def test_synthetic_demo_board_routes_completely():
    board = demo_board()
    result = route_board(board)
    assert result.unrouted == [] and result.stats["completion"] == 1.0
    assert_clean(board, result)
    assert 1.0 <= result.stats["length_ratio"] < 1.6


@pytest.mark.parametrize("board", [random_grid(6, 6, nets=10, seed=1, solvable=True),
                                   random_grid(8, 8, nets=16, seed=2, solvable=True),
                                   channel(6, seed=8)])
def test_solvable_boards_complete(board):
    result = route_board(board)
    assert result.unrouted == []
    assert_clean(board, result)


@pytest.mark.parametrize("options", [Options(), Options(global_selection=False, regret_order=False, lookahead=False, demand=False, ripup=False),
                                     Options(lookahead=False), Options(demand=False), Options(ripup=False)])
def test_every_configuration_gives_a_clean_result(options):
    board = random_grid(6, 6, nets=8, seed=3)
    result = route_board(board, options=options)
    assert_clean(board, result)
    assert result.stats["routed"] + len(result.unrouted) == result.stats["connections"]


def test_unroutable_connection_is_reported_not_forced():
    """A pad fenced in by a keepout ring cannot be reached on one layer."""
    board = Board.rectangle(20, 20, Rules(0.2, 0.2))
    board.pads.append(Pad.circle(0, 10, 10, 0.5, 0))
    board.pads.append(Pad.circle(1, 3, 3, 0.5, 0))
    for box in ([(7, 7), (13, 7), (13, 8), (7, 8)], [(7, 12), (13, 12), (13, 13), (7, 13)],
                [(7, 7), (8, 7), (8, 13), (7, 13)], [(12, 7), (13, 7), (13, 13), (12, 13)]):
        board.add_keepout(box)
    result = route_board(board)
    assert result.unrouted == [1] and result.polylines == {}


def test_multi_pin_net_uses_a_spanning_tree():
    board = Board.rectangle(30, 30, Rules(0.2, 0.2))
    for i, (x, y) in enumerate([(5, 5), (25, 5), (25, 25), (5, 25), (15, 15)]):
        board.pads.append(Pad.circle(i, x, y, 0.6, 0))
    board.pads.append(Pad.circle(5, 15, 5, 0.6, 1))
    board.pads.append(Pad.circle(6, 15, 25, 0.6, 1))
    pairs, buried = decompose(board, [Layer(0, "F.Cu", planar_map.build(board))])
    assert buried == [] and len(pairs) == 5  # 4 edges for the 5-pin net, 1 for the 2-pin net
    result = route_board(board)
    assert result.unrouted == []
    assert_clean(board, result)


def test_refinement_keeps_the_topology_valid():
    """Regression: putting a wire back after a failed refinement must use its
    present slots, not the ones recorded when it was first inserted."""
    board = random_grid(8, 8, nets=14, seed=4)
    result = route_board(board, drc_rounds=0, drop_violators=False)
    for layer in result.layers:
        assert layer.state.invariant_errors() == []
    assert not [v for v in result.violations if v.kind == "crossing"]


def test_wide_net_class_keeps_its_own_clearance():
    board = random_grid(6, 6, nets=8, seed=3)
    nets = sorted(board.nets())
    board.rules.net_width = {nets[0]: 0.5, nets[1]: 0.5}
    result = route_board(board)
    assert result.stats["routed"] >= 6
    assert_clean(board, result)  # checks every trace with its own width


def two_layer_wall_board() -> Board:
    """Front-only (surface-mount) pads either side of a front-layer wall: each net needs two vias."""
    front = frozenset({0})
    board = Board.rectangle(30, 20, Rules(0.2, 0.2), layers=["F.Cu", "B.Cu"])
    for i, y in enumerate((5, 10, 15)):
        board.pads.append(Pad.rect(i, 5, y, 1.2, 0.8, net_id=i, layers=front))
        board.pads.append(Pad.rect(10 + i, 25, y, 1.2, 0.8, net_id=i, layers=front))
    board.add_keepout([(14.5, 0), (15.5, 0), (15.5, 20), (14.5, 20)], layers=front)
    return board


def test_vias_carry_connections_under_a_wall():
    board = two_layer_wall_board()
    assert route_board(board, options=Options(vias=False)).stats["routed"] == 0
    result = route_board(board)
    assert result.unrouted == [] and result.stats["routed"] == 3
    assert len(result.vias) == 6 and len(board.pads) == 6  # the input board is not modified
    assert result.stats["wires_per_layer"] == {"F.Cu": 6, "B.Cu": 3}
    assert all(net_is_connected(result, n) for n in range(3))
    assert_clean(board, result)
    # A via keeps its copper clear of everything foreign on both layers, and of the board edge.
    rules = board.rules
    for via in result.vias:
        for other in result.board.pads:
            if other.net_id != via.net_id:
                assert via.shape.distance(other.shape) >= rules.clearance - 1e-6
        assert via.shape.distance(board.obstacles[0].shape) >= rules.clearance - 1e-6
        assert board.outline.exterior.distance(via.shape) >= rules.clearance - 1e-6


def test_no_via_touches_an_area_that_forbids_vias():
    """A rule area that forbids vias but not tracks (a DSN's via_keepout,
    ``Board.no_via``): the vias that stood in it go elsewhere, and traces
    still cross it."""
    band = Polygon([(8, 0), (14.4, 0), (14.4, 20), (8, 20)])   # the wall's left side, where vias go
    board = two_layer_wall_board()
    free = route_board(board, options=Options(portfolio=1), workers=1)
    assert any(v.shape.intersects(band) for v in free.vias)
    board.no_via = [band]
    result = route_board(board, options=Options(portfolio=1), workers=1)
    assert result.unrouted == [] and result.stats["routed"] == 3
    assert not any(v.shape.intersects(band) for v in result.vias)
    assert any(LineString(line).intersects(band) for line in result.polylines.values())
    assert all(net_is_connected(result, n) for n in range(3))
    assert_clean(board, result)
    with pytest.raises(ValueError):   # the routing made without the area, its vias in it, is not gone on from
        route_board(board, options=Options(portfolio=1), workers=1, resume=free.kept)


def test_through_hole_pads_let_connections_change_layer_without_vias():
    """Two nets that cross: one layer cannot hold both straight, two layers can."""
    board = Board.rectangle(20, 20, Rules(0.2, 0.2), layers=["F.Cu", "B.Cu"])
    for pad_id, x, y, net in [(0, 4, 4, 0), (1, 16, 16, 0), (2, 4, 16, 1), (3, 16, 4, 1)]:
        board.pads.append(Pad.circle(pad_id, x, y, 0.6, net))
    result = route_board(board)
    assert result.unrouted == [] and result.vias == []
    assert sorted(result.wire_layer.values()) == [0, 1]
    assert result.stats["length_ratio"] < 1.01
    assert_clean(board, result)


def test_a_layer_made_dear_is_used_only_where_it_must_be():
    """``CostParams.layer_cost`` (a layer that also carries a plane, say):
    net 0 may run on either layer and takes the front; with the front's
    lengths at five times their price it takes the back. Net 1 has pads on the
    front only and stays there, with no vias. What is measured is as before:
    only the price of a length on the front changes."""
    from weaveengine.topo.costs import CostParams
    from weaveengine.topo.search import route
    from weaveengine.topo.state import TopoState
    board = Board.rectangle(30, 30, Rules(0.2, 0.2), layers=["F.Cu", "B.Cu"])
    front, back = frozenset({0}), frozenset({1})
    board.pads += [Pad.circle(0, 3, 15, 0.6, 0), Pad.circle(1, 27, 15, 0.6, 0),                               # either layer
                   Pad.rect(2, 3, 5, 1, 1, 1, layers=front), Pad.rect(3, 27, 5, 1, 1, 1, layers=front),        # the front only
                   Pad.rect(4, 3, 25, 1, 1, 2, layers=back), Pad.rect(5, 27, 25, 1, 1, 2, layers=back)]        # the back only
    on = lambda result, net: {result.wire_layer[w] for w in result.polylines if result.wire_net[w] == net}
    plain = route_board(board, options=Options(portfolio=1), workers=1)
    dear = route_board(board, CostParams.for_map(planar_map.build(board), layer_cost={0: 5.0}), options=Options(portfolio=1), workers=1)
    assert [on(plain, n) for n in range(3)] == [{0}, {0}, {1}]
    assert [on(dear, n) for n in range(3)] == [{1}, {0}, {1}] and dear.unrouted == [] and dear.stats["vias"] == 0
    assert dear.stats["length"] == pytest.approx(plain.stats["length"])
    # On one map: a route costs twice its length where lengths are priced at twice; the length is the same.
    pmap = planar_map.build(board)
    once = route(pmap, TopoState(pmap), 0, 1, CostParams.for_map(pmap))
    pmap.price_lengths(2.0)
    twice = route(pmap, TopoState(pmap), 0, 1, CostParams.for_map(pmap))
    assert twice.length == pytest.approx(once.length) and twice.cost == pytest.approx(2.0 * once.cost)


def test_connections_change_layer_through_a_dear_layer():
    """Every connection of this board has to cross its wall on the back: with
    the back made dear, all are routed all the same. (A via's own triangle was
    then the cheapest place for the next via, a way that cannot be read back,
    and the connection was given up.)"""
    from bench.generators import smd_two_layer
    from weaveengine.topo.costs import CostParams
    board = smd_two_layer(6, seed=10)
    result = route_board(board, CostParams.for_map(planar_map.build(board), layer_cost={1: 5.0}), options=Options(portfolio=1), workers=1)
    assert result.unrouted == [] and result.stats["routed"] == result.stats["connections"] == 6
    assert_clean(board, result)


def test_a_detour_on_a_dear_layer_is_still_undone():
    """Refinement (``ripup._replan``) weighs a new plan, at the search's
    prices, against what the connection has at the same prices: a detour on
    a layer made dear is undone as on any other."""
    from weaveengine import parallel, router
    from weaveengine.plan import ripup
    from weaveengine.plan.context import Context
    from weaveengine.plan.path import Path
    from weaveengine.topo.costs import CostParams
    from weaveengine.topo.search import route
    front = frozenset({0})
    board = Board.rectangle(30, 30, Rules(0.2, 0.2), layers=["F.Cu", "B.Cu"])
    board.pads += [Pad.rect(0, 3, 15, 1, 1, 0, layers=front), Pad.rect(1, 27, 15, 1, 1, 0, layers=front),
                   Pad.rect(2, 15, 15, 1, 1, 1, layers=front), Pad.rect(3, 15, 25, 1, 1, 1, layers=front)]
    layers = [Layer(i, name, pmap) for i, (name, pmap) in enumerate(zip(board.layers, parallel.run(router._build_task, board, range(2), 1)))]
    pairs, _ = decompose(board, layers)
    ctx = Context(board, layers, pairs, CostParams.for_map(layers[0].pmap, layer_cost={0: 5.0}), Options(portfolio=1, slide=False), seed=0, workers=1)
    w = next(w for w, c in ctx.conns.items() if c.net_id == 0)
    conn, front_layer = ctx.conns[w], ctx.layers[0]
    direct = route(front_layer.pmap, front_layer.state, conn.src, conn.dst, ctx.params, weight=conn.weight, net=conn.net_id)
    assert direct.cost == pytest.approx(5.0 * direct.length)   # (the context prices the front as its parameters say)
    detour = route(front_layer.pmap, front_layer.state, conn.src, conn.dst, ctx.params, weight=conn.weight, net=conn.net_id,
                   penalty={g: 40.0 for g in direct.gates[1:-1]})   # (sent round)
    detour.layer = 0
    assert ctx.commit(conn, Path.on_one_layer(detour))
    assert ctx.copper()[w] > direct.length + 1.0
    assert ripup._replan(ctx, conn, ctx.copper()[w], 0, 1e-6) and ctx.copper()[w] == pytest.approx(direct.length)


def turn_at_pad(line, ring: float) -> float:
    """Largest change of direction (degrees) the trace makes within ``ring`` of where it starts."""
    worst = 0.0
    for a, b, c in zip(line, line[1:], line[2:]):
        if math.dist(line[0], b) > ring:
            break
        h1 = math.atan2(b[1] - a[1], b[0] - a[0])
        h2 = math.atan2(c[1] - b[1], c[0] - b[0])
        worst = max(worst, abs(math.degrees((h2 - h1 + math.pi) % (2 * math.pi) - math.pi)))
    return worst


@pytest.mark.parametrize("make", [demo_board, lambda: random_grid(6, 6, nets=10, seed=1, solvable=True)])
def test_trace_leaves_a_round_pad_without_a_kink(make):
    """A trace runs straight out of the pad centre: it does not turn where it crosses the pad's keep-off ring."""
    board = make()
    result = route_board(board)
    assert result.unrouted == []
    rules = board.rules
    pads = {p.pad_id: p for p in board.pads}
    for w, line in result.polylines.items():
        conn = result.connections[w]
        for pad_id, pts in ((conn.src, line), (conn.dst, line[::-1])):
            ring = 1.2 * (pads[pad_id].radius / math.cos(math.pi / 16) + rules.inflation)
            assert turn_at_pad(pts, ring) < 6.0, f"wire {w} turns at pad {pad_id}"
    assert_clean(board, result)


def test_hopping_a_trace_end_round_its_pad_keeps_the_topology():
    from weaveengine.realize.relax import relax
    from weaveengine.realize.terminals import hop, reversed_steps
    board = demo_board()
    result = route_board(board, options=Options(teardrops=False))
    state = result.state
    steps = state.wire_path[1]
    assert reversed_steps(reversed_steps(steps)) == steps
    # Push every end round both corners of its pad edge as far as it will go.
    moves = 0
    for wire in list(state.wire_path):
        for at_start in (True, False):
            for side in (0, 1):
                for _ in range(20):
                    path = state.wire_path[wire]
                    edge = path[0][0] if at_start else path[-1][0]
                    if not hop(state, wire, at_start, result.pmap.edge_v_list[edge][side]):
                        break
                    moves += 1
                    assert state.invariant_errors() == []
    assert moves > 10
    assert len(relax(state, board)) == 3


def test_teardrops_on_round_pads():
    from weaveengine.realize.teardrop import MAX_LENGTH, MAX_WIDTH, WIDTH
    board = demo_board()
    result = route_board(board)
    pads = {p.pad_id: p for p in board.pads}
    assert sum(len(t) for t in result.teardrops.values()) == 6  # both ends of three traces
    for w, drops in result.teardrops.items():
        conn = result.connections[w]
        line = LineString(result.polylines[w])
        for poly in drops:
            shape = Polygon(poly[:5])
            pad = min((pads[conn.src], pads[conn.dst]), key=lambda p: math.dist(p.centre, poly[0]))
            assert poly[0] == pad.centre
            half = board.rules.width(conn.net_id) / 2
            tip = tip_of(poly, half)
            # tangent to a circle of 90 % of the pad, but never wider than the limit
            r = min(WIDTH * pad.radius, MAX_WIDTH / 2)
            assert abs(math.dist(poly[1], poly[4]) - 2 * r * math.sin(math.acos(r / math.dist(pad.centre, tip)))) < 1e-6
            assert math.dist(poly[1], poly[4]) <= MAX_WIDTH + 1e-9
            # and no longer than the limit beyond the pad
            assert math.dist(pad.centre, tip) - pad.radius <= MAX_LENGTH + 1e-6
            # the sides touch the track's round end at the tip tangentially
            for side_pt, pad_pt in ((poly[2], poly[1]), (poly[3], poly[4])):
                assert abs(math.dist(side_pt, tip) - half) < 1e-6
                assert abs((side_pt[0] - tip[0]) * (pad_pt[0] - side_pt[0]) + (side_pt[1] - tip[1]) * (pad_pt[1] - side_pt[1])) < 1e-6
            assert line.distance(Point(tip)) < 1e-6
            for other in board.pads:
                if other.net_id != conn.net_id:
                    assert shape.distance(other.shape) >= 1.5 * board.rules.clearance
    assert route_board(board, options=Options(teardrops=False)).teardrops == {}


def test_teardrop_size_is_limited_on_large_pads():
    board = Board.rectangle(60, 30, Rules(0.2, 0.2))
    board.pads.append(Pad.circle(0, 12, 15, 6.0, net_id=0))      # a 12 mm pad
    board.pads.append(Pad.rect(1, 48, 15, 10.0, 8.0, net_id=0))  # and a 10 x 8 mm one
    result = route_board(board)
    drops = [poly for d in result.teardrops.values() for poly in d]
    assert len(drops) == 2
    for poly in drops:
        assert math.dist(poly[1], poly[4]) <= 2.0 + 1e-6            # default limit across
        assert Polygon(poly[:5]).area < 0.5 * 6.0 * 2.0 + 2.0 * 1.2      # nowhere near "half the board"
    wide = route_board(board, options=Options(teardrop_max_width=4.0, teardrop_max_length=3.0))
    assert max(math.dist(poly[1], poly[4]) for d in wide.teardrops.values() for poly in d) > 3.0


def test_teardrop_is_made_smaller_where_it_is_cramped():
    """A teardrop wants breathing room. Next to foreign copper it is shortened, then
    kept at the plain clearance, then narrowed: a small one rather than none."""
    board = Board.rectangle(30, 20, Rules(0.2, 0.2))
    board.pads.append(Pad.circle(0, 5, 10, 1.0, net_id=0))
    board.pads.append(Pad.circle(1, 25, 10, 1.0, net_id=0))
    open_board = route_board(board)
    assert sum(len(d) for d in open_board.teardrops.values()) == 2
    roomy = max(Polygon(poly[:5]).area for d in open_board.teardrops.values() for poly in d)
    # a foreign pad just above the trace, right outside pad 0
    board.pads.append(Pad.circle(2, 6.6, 10.75, 0.3, net_id=1))
    board.pads.append(Pad.circle(3, 6.6, 3, 0.3, net_id=1))
    cramped = route_board(board)
    assert cramped.unrouted == []
    near_pad0 = [poly for d in cramped.teardrops.values() for poly in d if math.dist(poly[0], (5, 10)) < 1e-6]
    assert len(near_pad0) == 1 and Polygon(near_pad0[0][:5]).area < 0.8 * roomy
    # What it adds to its pad keeps the clearance every trace keeps.
    added = Polygon(near_pad0[0][:5]).difference(board.pads[0].shape)
    assert added.distance(board.pads[2].shape) >= board.rules.clearance - 1e-6
    assert_clean(board, cramped)
    # Hemmed in from both sides it is smaller still, or left out; never nearer than the clearance.
    board.pads.append(Pad.circle(4, 6.6, 9.25, 0.3, net_id=1))
    shut = route_board(board)
    for w, d in shut.teardrops.items():
        for poly in d:
            if shut.wire_net[w] == 0 and math.dist(poly[0], (5, 10)) < 1e-6:
                added = Polygon(poly[:5]).difference(board.pads[0].shape)
                assert all(added.distance(board.pads[n].shape) >= board.rules.clearance - 1e-6 for n in (2, 4))
    assert_clean(board, shut)


def test_teardrops_on_a_short_trace_meet_in_one_line():
    """Two pads of a net close together. Two teardrops would reach past each
    other; each cut to half the trace by itself, their ends cross and leave a
    notch either side. Made together they are an hourglass: they meet in one
    line across the trace, as wide as the trace, and nowhere overlap."""
    from shapely.ops import unary_union
    from weaveengine.io.ses import teardrop_tracks
    board = Board.rectangle(12, 8, Rules(0.2, 0.2))
    board.pads.append(Pad.circle(0, 5.0, 5.0, 0.5, net_id=0))
    board.pads.append(Pad.rect(1, 5.3, 3.75, 1.6, 1.0, net_id=0))
    result = route_board(board, options=Options(teardrop_max_length=0.5, teardrop_max_width=1.0))
    assert result.unrouted == []
    (w, drops), = result.teardrops.items()
    assert len(drops) == 2
    one, two = (Polygon(poly[:5]) for poly in drops)
    assert one.is_valid and two.is_valid
    assert not any(pad.shape.contains(Point(tip_of(poly))) for poly in drops for pad in board.pads)
    # They share the line across the waist, which is as wide as the trace and square to it ...
    assert drops[0][2] == drops[1][2] and drops[0][3] == drops[1][3] and tip_of(drops[0]) == tip_of(drops[1])
    assert math.dist(drops[0][2], drops[0][3]) == pytest.approx(board.rules.trace_width)
    assert LineString(result.polylines[w]).distance(Point(tip_of(drops[0]))) < 1e-9
    # ... and touch nowhere else: no overlap, and no gap between them beside the trace.
    assert one.intersection(two).area < 1e-12
    waist = LineString([drops[0][2], drops[0][3]])
    assert one.intersection(two).difference(waist.buffer(1e-9)).is_empty
    # Each is wide at its own pad: an hourglass.
    assert all(math.dist(poly[1], poly[4]) > 3 * board.rules.trace_width for poly in drops)
    # Written out as traces, they are filled and nothing reaches outside them.
    track = LineString(result.polylines[w]).buffer(0.1)
    tracks = [LineString([a, b]).buffer(width / 2.0) for poly in drops for width, a, b in teardrop_tracks(poly, board.rules.trace_width)]
    solid = unary_union([one, two, board.pads[0].shape, board.pads[1].shape, track])
    assert all(solid.buffer(1e-5).contains(t) for t in tracks)
    assert unary_union(tracks + [board.pads[0].shape, board.pads[1].shape, track]).area > 0.999 * solid.area
    assert_clean(board, result)


def square_pad_board(width: float = 0.2) -> Board:
    board = Board.rectangle(30, 20, Rules(0.2, 0.2))
    for pad_id, x, y, net in [(0, 5, 5, 0), (1, 25, 8, 0), (2, 5, 15, 1), (3, 25, 13, 1)]:
        board.pads.append(Pad.rect(pad_id, x, y, 1.8, 1.8, net_id=net, rotation=30 if pad_id == 3 else 0))
    board.rules.net_width = {1: width} if width != 0.2 else {}
    return board


def test_teardrops_on_square_pads():
    board = square_pad_board()
    result = route_board(board)
    pads = {p.pad_id: p for p in board.pads}
    assert result.unrouted == [] and sum(len(t) for t in result.teardrops.values()) == 4
    for w, drops in result.teardrops.items():
        conn = result.connections[w]
        line = LineString(result.polylines[w])
        for poly in drops:
            pad = min((pads[conn.src], pads[conn.dst]), key=lambda p: math.dist(p.centre, poly[0]))
            shape = Polygon(poly[:5])
            assert shape.is_valid and poly[0] == pad.centre
            # the pad end sits on two corners of the pad shrunk to 90 %, inside the pad's copper
            for corner in (poly[1], poly[4]):
                assert pad.shape.contains(Point(corner))
                # on a corner of the pad shrunk towards its centre (by 90 %, or more to respect the width limit)
                scales = [math.dist(corner, pad.centre) / math.dist((x, y), pad.centre) for x, y in pad.shape.exterior.coords
                          if abs((corner[0] - pad.centre[0]) * (y - pad.centre[1]) - (corner[1] - pad.centre[1]) * (x - pad.centre[0])) < 1e-9
                          and (corner[0] - pad.centre[0]) * (x - pad.centre[0]) + (corner[1] - pad.centre[1]) * (y - pad.centre[1]) > 0]
                assert scales and max(scales) <= 0.9 + 1e-9
            assert math.dist(poly[1], poly[4]) <= 2.0 + 1e-6
            assert math.dist(poly[1], poly[4]) > 1.0  # much wider than the trace at the pad
            # the narrow end is the trace itself, outside the pad
            tip = tip_of(poly, 0.1)
            assert 0.15 < math.dist(poly[2], poly[3]) <= 0.2 + 1e-9
            assert line.distance(Point(tip)) < 1e-6 and not pad.shape.contains(Point(tip))
            # the trace runs down the middle of it
            assert shape.buffer(1e-6).contains(line.intersection(Point(pad.centre).buffer(math.dist(pad.centre, _mid(poly[2], poly[3])) - 1e-3)))


@pytest.mark.parametrize("width", [0.2, 0.6])
def test_trace_leaves_a_square_pad_without_a_jolt(width):
    """A trace owes its own pad no clearance: it must not be pushed off line by
    the corners of its own pad's keep-off ring (seen with wide traces)."""
    board = square_pad_board(width)
    result = route_board(board)
    assert result.unrouted == []
    for w, line in result.polylines.items():
        # nothing else is in the way on this board: every trace is one straight segment
        assert len(line) == 2, f"wire {w} bends: {line}"
    assert_clean(board, result)


def _mid(a, b):
    return ((a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0)


PIC = os.path.join(os.path.dirname(os.path.dirname(__file__)), "boards", "pic-programmer", "pic-programmer.dsn")


@pytest.mark.skipif(not os.path.exists(PIC), reason="example board not present")
def test_pic_programmer_board(tmp_path):
    """A real two-layer KiCad board (KiCad's pic_programmer demo): two net classes."""
    from weaveengine.io.dsn import parse_sexpr, read_dsn
    from weaveengine.io.ses import write_ses

    design = read_dsn(PIC)
    board = design.board
    assert len(board.pads) == 240 and board.layers == ["top_layer", "bottom_layer"]
    assert {design.net_names[n] for n in board.rules.net_width} == {"GND", "VCC"}
    result = route_board(board)
    assert result.unrouted == [] and result.stats["routed"] == result.stats["connections"] == 86
    assert all(net_is_connected(result, n) for n in board.nets())
    assert_clean(board, result)
    assert min(result.stats["wires_per_layer"].values()) > 10  # both layers are used

    out = str(tmp_path / "pic.ses")
    write_ses(design, result, out, teardrops=False)
    session = parse_sexpr(open(out, encoding="utf-8").read())
    routes = next(c for c in session if isinstance(c, list) and c[0] == "routes")
    nets = [c for c in next(c for c in routes if isinstance(c, list) and c[0] == "network_out") if isinstance(c, list)]
    wires = [w[1] for n in nets for w in n[2:] if w[0] == "wire"]
    assert len(wires) == len(result.polylines)
    assert {w[1] for w in wires} == {"top_layer", "bottom_layer"}
    vcc = next(n for n in nets if n[1] == "VCC")
    assert {w[1][2] for w in vcc[2:] if w[0] == "wire"} == {"8000"}  # 0.8 mm at 0.1 um


def test_result_does_not_depend_on_the_number_of_workers():
    """Section 22: for a given portfolio size the outcome is the same on one core and on many."""
    board = random_grid(8, 8, nets=14, seed=4)
    runs = [route_board(board, options=Options(portfolio=3), workers=w) for w in (1, 4)]
    assert runs[0].polylines == runs[1].polylines
    assert runs[0].unrouted == runs[1].unrouted and runs[0].wire_layer == runs[1].wire_layer
    alone = [route_board(board, options=Options(portfolio=1), workers=w) for w in (1, 4)]
    assert alone[0].polylines == alone[1].polylines
    for r in runs + alone:
        assert_clean(board, r)


def test_result_does_not_depend_on_the_process():
    """The same board and settings give the same routing, to the byte, in every
    run of the program: nothing depends on the order of a set or dict that
    differs between processes (Python's string hashing is seeded per process),
    nor on which raced variant finishes first. (lily58-pro was seen to route
    two ways in the board sweep; that was KiCad writing the DSN two ways, see
    ``bench/sweep/run.py``'s ``prepare``.)"""
    import subprocess
    import sys
    code = ("import hashlib\n"
            "from bench.generators import random_grid\n"
            "from weaveengine.router import route_board\n"
            "from weaveengine.plan.context import Options\n"
            "r = route_board(random_grid(8, 8, nets=14, seed=4), options=Options(portfolio=3), workers=3)\n"
            "print(hashlib.sha1(repr((sorted(r.polylines.items()), r.unrouted, [v.centre for v in r.vias])).encode()).hexdigest())\n")
    root = os.path.dirname(os.path.dirname(__file__))
    runs = {subprocess.run([sys.executable, "-c", code], cwd=root, env=dict(os.environ, PYTHONHASHSEED=seed),
                           capture_output=True, text=True, check=True).stdout.strip() for seed in ("1", "2", "3")}
    assert len(runs) == 1


def test_a_deadline_that_has_passed_stops_rip_up_and_further_heats(monkeypatch):
    """Past the time limit nothing more is done to better the routing, in the
    raced variants too, whether they are forked or started afresh (Windows):
    the deadline is a time on the clock, which every process reads alike.
    Commit makes a first routing, no round of rip-up runs, it is drawn and
    returned clean, and no second heat is started."""
    import time
    from weaveengine import parallel
    board = random_grid(8, 8, nets=14, seed=4)
    assert route_board(board, options=Options(portfolio=1), workers=1).stats["ripup_rounds"] > 0
    for start in ("fork", "spawn"):
        if start == "fork" and not parallel.can_fork():
            continue
        if start == "spawn":
            monkeypatch.setenv("WEAVEENGINE_START", "spawn")
        phases = []
        late = route_board(board, options=Options(portfolio=6), workers=3, deadline=time.time() - 1.0,
                           events=lambda e: phases.append(e["phase"]) if e["type"] == "progress" else None)
        assert late.stats["ripup_rounds"] == 0 and late.stats["routed"] > 0
        assert "racing variants 1-4 of 6" in phases and "racing variants 5-6 of 6" not in phases
        assert_clean(board, late)


def test_checkpoints_see_the_variants_as_they_finish_and_change_nothing():
    """``checkpoint`` is handed each raced variant's routing as it finishes, as a
    result ``write_ses`` can write; neither it nor a time limit that is not
    reached changes the routing."""
    import time
    board = random_grid(8, 8, nets=14, seed=4)
    seen = []
    watched = route_board(board, options=Options(portfolio=3), workers=3, checkpoint=lambda key, result: seen.append((key, result)))
    plain = route_board(board, options=Options(portfolio=3), workers=3)
    limited = route_board(board, options=Options(portfolio=3), workers=3, deadline=time.time() + 3600.0)
    assert watched.polylines == plain.polylines == limited.polylines and watched.unrouted == plain.unrouted == limited.unrouted
    assert 1 <= len(seen) <= 3
    for key, result in seen:
        assert key[0] == len(result.unrouted) == result.stats["connections"] - result.stats["routed"]
        assert set(result.polylines) == set(result.wire_layer) <= set(result.wire_net)
    best = min(seen, key=lambda s: s[0])[1]
    assert best.stats["routed"] == watched.stats["routed"]
    alone = []
    route_board(board, options=Options(portfolio=1), workers=1, checkpoint=lambda key, result: alone.append(key))
    assert len(alone) == 1  # a pass that is not raced: once, at its end


LILY = os.path.join(os.path.dirname(os.path.dirname(__file__)), "boards", "lily58-pro", "lily58-pro.dsn")
ALLOWANCE = 30.0  # s past the time limit for what follows it: the routing made legal, its geometry and check, the SES written and measured


@pytest.mark.skipif(not os.path.exists(LILY), reason="example board not present")
def test_a_time_limit_ends_the_run_in_time_and_writes_the_best_routing(tmp_path, capsys, monkeypatch):
    """lily58-pro takes about a minute on two workers. Given three seconds, the
    command line ends within them and ``ALLOWANCE``, having saved the best
    routing as it went, and the SES it writes holds every trace and via of the
    routing it ends with and keeps the rules."""
    import time
    from weaveengine import cli
    from weaveengine.io.check import read_session
    results, real = [], cli.route_board

    def route_board(board, params=None, **how):
        results.append(real(board, params, **how))
        return results[-1]
    monkeypatch.setattr(cli, "route_board", route_board)
    out = str(tmp_path / "lily.ses")
    start = time.time()
    cli.main([LILY, "-o", out, "--time-limit", "0.05", "--workers", "2", "--portfolio", "2", "--quiet", "--no-ses-teardrops"])
    assert time.time() - start < 3.0 + ALLOWANCE
    text = capsys.readouterr().out
    assert "saved the best routing so far" in text and "the time limit was reached" in text and "-> OK" in text
    result = results[0]
    assert result.stats["routed"] > 0
    wires, vias = read_session(out)
    assert len(wires) == len(result.polylines) and len(vias) == len(result.vias)


def test_racing_variants_never_does_worse_than_the_plain_run():
    board = random_grid(8, 8, nets=14, seed=4)
    plain = route_board(board, options=Options(portfolio=1))
    raced = route_board(board, options=Options(portfolio=4))
    assert raced.stats["routed"] >= plain.stats["routed"]
    assert_clean(board, raced)


def test_racing_without_fork_routes_as_with_it(monkeypatch):
    """Where processes cannot be forked (Windows), the raced variants run in
    processes started afresh, each given a copy of the board's state: the same
    routing comes out, whichever way the workers were made."""
    board = random_grid(8, 8, nets=14, seed=4)
    forked = route_board(board, options=Options(portfolio=3), workers=3)
    monkeypatch.setenv("WEAVEENGINE_START", "spawn")
    given = route_board(board, options=Options(portfolio=3), workers=3)
    assert given.polylines == forked.polylines and given.unrouted == forked.unrouted
    assert [v.centre for v in given.vias] == [v.centre for v in forked.vias]


def test_progress_is_reported():
    seen = []
    route_board(demo_board(), options=Options(portfolio=1), progress=lambda phase, done=0.0, total=1.0: seen.append(phase))
    phases = [p.split(": ")[1] for p in seen]
    assert all(p.startswith("pass 1: ") for p in seen)
    for wanted in ("candidates", "global selection", "commit", "geometry"):
        assert wanted in phases
    assert phases.index("candidates") < phases.index("commit") < phases.index("geometry")


def test_python_fallback_without_numba():
    """The compiled kernels are optional: the pure-Python loops give the same routing."""
    import subprocess
    import sys
    code = ("from tests.conftest import demo_board\n"
            "from weaveengine.router import route_board\n"
            "from weaveengine.plan.context import Options\n"
            "from weaveengine.topo import kernel\n"
            "r = route_board(demo_board(), options=Options(portfolio=1))\n"
            "print(kernel.AVAILABLE, len(r.unrouted), len(r.violations), sorted((w, len(p)) for w, p in r.polylines.items()))\n")
    root = os.path.dirname(os.path.dirname(__file__))
    out = {}
    for flag in ("", "1"):
        env = dict(os.environ, WEAVEENGINE_NO_NUMBA=flag) if flag else {k: v for k, v in os.environ.items() if k != "WEAVEENGINE_NO_NUMBA"}
        done = subprocess.run([sys.executable, "-c", code], cwd=root, env=env, capture_output=True, text=True, check=True)
        out[flag] = done.stdout.strip().split(" ", 1)
    assert out["1"][0] == "False"
    assert out[""][1] == out["1"][1]  # same wires, same shapes, compiled or not
    assert out["1"][1].startswith("0 0 ")


def copper(result) -> float:
    """Length of the copper that is drawn: traces that coincide count once."""
    return result.stats["copper"]


def test_traces_of_one_net_share_a_trunk():
    """Two connections of a net that leave a pad the same way run as one trace
    until they part, instead of side by side."""
    board = Board.rectangle(30, 40, Rules(0.2, 0.2))
    for pad_id, x, y in [(0, 5, 10), (1, 20, 2), (2, 20, 21.5)]:
        board.pads.append(Pad.circle(pad_id, x, y, 0.6, net_id=0))
    # a wall both connections from pad 0 have to go round the top of
    board.add_keepout([(12, 0), (13, 0), (13, 22), (12, 22)])
    result = route_board(board)
    assert result.unrouted == [] and net_is_connected(result, 0)
    assert {(c.src, c.dst) for c in result.connections.values() if c.parent is None} == {(0, 1), (0, 2)}
    # From pad 0 round the end of the wall there is one trace, whatever is joined to what beyond it.
    assert copper(result) < 44.0   # pad 0 to the wall's end, and on to each (41.8 if the second goes on from pad 2)
    assert_clean(board, result)


def test_teardrops_are_written_to_the_session_as_traces(tmp_path):
    """SES has no filled shapes: a teardrop goes out as ordinary traces fanned
    from where it starts on the track to its edge at the pad, outer lines first,
    filling it without reaching outside it."""
    from weaveengine.io.check import measure, read_session
    from weaveengine.io.dsn import Design
    from shapely.ops import unary_union
    from weaveengine.io.ses import teardrop_tracks, write_ses
    board = demo_board()
    board.pads.append(Pad.rect(900, 50, 52, 6.0, 2.5, net_id=5, rotation=20))   # an oblong pad at an angle
    board.pads.append(Pad.rect(901, 50, 8, 2.5, 6.0, net_id=5))
    with_drops, without = (str(tmp_path / n) for n in ("a.ses", "b.ses"))
    design = Design("demo", board, net_ids={f"N{n}": n for n in board.nets()})
    result = route_board(board)
    assert result.unrouted == [] and sum(len(d) for d in result.teardrops.values()) == 8
    pads = {p.pad_id: p for p in board.pads}
    count = 0
    for w, drops in result.teardrops.items():
        conn = result.connections[w]
        width = board.rules.width(conn.net_id)
        for poly in drops:
            pad = min((pads[conn.src], pads[conn.dst]), key=lambda p: math.dist(p.centre, poly[0]))
            tear = Polygon(poly[:5])
            tip = tip_of(poly, width / 2)
            tracks = teardrop_tracks(poly, width)
            assert len(tracks) >= 2
            assert all(t[0] == width and math.dist(t[1], tip) < 1e-9 for t in tracks)   # ordinary traces, all from the tip
            copper = [LineString([a, b]).buffer(w_ / 2.0) for w_, a, b in tracks]
            track = LineString(result.polylines[w]).buffer(width / 2.0)
            for c in copper:
                # all of it lies over the teardrop, the track or the pad itself: nothing reaches
                # past the pad, not even the round end of a line
                assert unary_union([tear, track, pad.shape]).buffer(1e-5).contains(c)
            # the first two are the outer lines, hard against the teardrop's two sides
            sides = [LineString([poly[2], poly[1]]), LineString([poly[3], poly[4]])]
            assert {min(range(2), key=lambda i: sides[i].distance(Point(t[2]))) for t in tracks[:2]} == {0, 1}
            # together they fill it
            assert unary_union(copper + [pad.shape, track]).area > 0.999 * unary_union([tear, pad.shape, track]).area
            count += len(tracks)
    write_ses(design, result, with_drops)
    write_ses(design, result, without, teardrops=False)
    assert len(read_session(without)[0]) == 4
    assert len(read_session(with_drops)[0]) == 4 + count
    assert measure(design, with_drops).ok  # the fill keeps every clearance too


def test_a_teardrop_fill_met_from_a_long_pads_side_ends_on_the_pad():
    """A QFP pin at 45 degrees met by its track from the side (hackrf-marzipan,
    U23-72): the teardrop is short and wide, its outer fill lines nearly along
    the pad's edge. Carried on to the chord's line, one ran 0.55 mm past the pad
    and KiCad called it dangling; every line now ends with its round end over the pad."""
    from weaveengine.io.ses import teardrop_tracks
    from weaveengine.realize.teardrop import _polygonal
    pad = Polygon([(135.7083, -139.1879), (135.9166, -138.9796), (134.8552, -137.9181), (134.6468, -138.1264)])
    width = 0.2794
    poly = _polygonal(pad.centroid.coords[0], list(pad.exterior.coords)[:-1], (134.9573, -138.6061), width / 2.0)
    tracks = teardrop_tracks(poly, width)
    assert len(tracks) >= 2
    assert all(pad.distance(Point(b)) < width / 4.0 for _, _, b in tracks)


def test_kernel_self_check_never_crashes_and_falls_back(monkeypatch):
    """A numba that loads but does not work must give a clear warning and a working router."""
    from weaveengine import accel
    from weaveengine.realize import kernel as relax_kernel
    from weaveengine.topo import kernel as search_kernel
    if not search_kernel.AVAILABLE:
        pytest.skip("numba not in use")
    good = accel.check(force=True)
    assert good.compiled and good.warning == "" and "numba" in good.message

    def broken(*args, **kwargs):
        raise RuntimeError("LLVM ERROR: simulated")

    real = [getattr(relax_kernel, name) for name in ("pull", "reach", "lifts", "inside", "blocked", "heading")]
    try:
        # the search kernel fails
        monkeypatch.setattr(search_kernel, "astar", broken)
        bad = accel.check(force=True)
        assert not bad.compiled and "search" in bad.message and "simulated" in bad.detail
        assert bad.warning.startswith("WARNING") and "slower" in bad.warning
        result = route_board(demo_board(), options=Options(portfolio=1))   # and routing still works
        assert result.unrouted == [] and result.violations == []
        monkeypatch.undo()
        # the relaxation kernel fails
        search_kernel.AVAILABLE = True
        broken.py_func = real[0].py_func   # as a real compiled function has
        monkeypatch.setattr(relax_kernel, "pull", broken)
        bad = accel.check(force=True)
        assert not bad.compiled and "relaxation" in bad.message
        assert relax_kernel.pull is not broken
    finally:
        monkeypatch.undo()
        relax_kernel.pull, relax_kernel.reach, relax_kernel.lifts, relax_kernel.inside, relax_kernel.blocked, relax_kernel.heading = real
        search_kernel.AVAILABLE = True
        assert accel.check(force=True).compiled


def test_wide_trace_teardrop_on_a_square_pad_stays_clear_of_a_neighbour(tmp_path):
    """Regression: the round end of a fill line used to poke past the pad's
    corner, closer to a foreign trace than the rules allow."""
    from weaveengine.io.check import measure
    from weaveengine.io.dsn import Design
    from weaveengine.io.ses import write_ses
    board = Board.rectangle(20, 14, Rules(0.2, 0.2))
    board.rules.net_width = {0: 0.5}
    board.pads.append(Pad.rect(0, 12, 8, 1.6, 1.8, net_id=0))
    board.pads.append(Pad.circle(1, 5, 2.5, 0.8, net_id=0))      # the wide trace arrives at the square pad's corner
    board.pads.append(Pad.circle(2, 9.6, 12, 0.5, net_id=1))     # a foreign trace passes just beside that corner
    board.pads.append(Pad.circle(3, 9.6, 2, 0.5, net_id=1))
    design = Design("t", board, net_ids={"N0": 0, "N1": 1})
    result = route_board(board, options=Options(teardrop_breathing=1.0))
    assert result.unrouted == []
    out = str(tmp_path / "t.ses")
    write_ses(design, result, out)
    m = measure(design, out)
    assert m.ok, (m.track_to_pad, m.track_to_track)


def sharp_corners(result, degrees: float = 12.0) -> int:
    count = 0
    for line in result.polylines.values():
        for a, b, c in zip(line, line[1:], line[2:]):
            h1 = math.atan2(b[1] - a[1], b[0] - a[0])
            h2 = math.atan2(c[1] - b[1], c[0] - b[0])
            count += abs(math.degrees((h2 - h1 + math.pi) % (2 * math.pi) - math.pi)) >= degrees
    return count


def test_sharp_corners_are_rounded_within_the_rules():
    board = demo_board()                      # traces bend round the keepout and each other's pads
    plain = route_board(board, options=Options(smooth=False))
    smooth = route_board(board, options=Options(smooth=True))
    assert sharp_corners(plain) >= 6
    # (what is left sits on the very corner of a keep-off: no arc fits inside it)
    assert sharp_corners(smooth) <= sharp_corners(plain) // 2
    assert smooth.stats["corners_rounded"] >= 6 and plain.stats["corners_rounded"] == 0
    # an arc inside a corner is a short cut: never longer, same connections, still clean
    assert smooth.stats["length"] <= plain.stats["length"] + 1e-6
    assert smooth.unrouted == [] and set(smooth.polylines) == set(plain.polylines)
    assert_clean(board, smooth)
    for w, line in smooth.polylines.items():  # the ends have not moved
        assert line[0] == plain.polylines[w][0] and line[-1] == plain.polylines[w][-1]


def test_corner_is_left_alone_where_an_arc_would_break_a_rule():
    from weaveengine.realize.smooth import smooth
    board = Board.rectangle(20, 20, Rules(0.2, 0.2))
    board.pads.append(Pad.circle(0, 2, 2, 0.4, net_id=0))
    board.pads.append(Pad.circle(1, 18, 18, 0.4, net_id=0))
    corner = [(2, 2), (18, 2), (18, 18)]       # a right angle at (18, 2)
    free, rounded, kept = smooth(board, {1: corner}, {1: 0})
    assert (rounded, kept) == (1, 0) and len(free[1]) > 10
    arc = LineString(free[1])
    assert arc.length < LineString(corner).length and arc.distance(Point(18, 2)) > 2.0   # a generous arc
    # the same corner hugging a foreign pad: only a small arc fits, and it keeps its clearance
    board.pads.append(Pad.circle(2, 17.0, 3.0, 0.5, net_id=1))
    tight, rounded, kept = smooth(board, {1: corner}, {1: 0})
    assert rounded == 1
    gap = LineString(tight[1]).distance(board.pads[2].shape) - 0.1
    assert gap >= board.rules.clearance and LineString(tight[1]).distance(Point(18, 2)) < 0.5
    # and with the foreign pad right in the corner, nothing fits: the corner stays
    board.pads[2] = Pad.circle(2, 17.55, 2.45, 0.1, net_id=1)
    stuck, rounded, kept = smooth(board, {1: corner}, {1: 0})
    assert (rounded, kept) == (0, 1) and stuck[1] == corner


def crossing_board(pairs: int = 4) -> Board:
    """Through-hole pads against the top and bottom edges of the board, each
    top pad joined to the bottom pad at the mirrored position. Every connection
    crosses every other and there is no way round a pad that sits against the
    edge, so each layer holds one of them and the rest need vias."""
    board = Board.rectangle(30, 24, Rules(0.2, 0.2), layers=["F.Cu", "B.Cu"])
    for i in range(pairs):
        x = 1.1 + i * 27.8 / (pairs - 1)
        board.pads.append(Pad.circle(i, x, 22.9, 0.8, net_id=i))
        board.pads.append(Pad.circle(pairs + i, 30 - x, 1.1, 0.8, net_id=i))
    return board


def test_vias_are_placed_during_the_pass():
    """M14: connections no single layer can hold are completed with vias in the
    same pass, legally, and the result is the same whether or not variants are raced."""
    board = crossing_board()
    assert route_board(board, options=Options(vias=False, portfolio=1), workers=1).stats["routed"] == 2
    result = route_board(board, options=Options(portfolio=1), workers=1)
    assert_clean(board, result)
    stats = result.stats
    assert stats["routed"] == 4 and 2 <= stats["vias"] <= 8 and stats["via_sites"] == stats["vias"]
    assert len(board.pads) == 8 and all(not p.is_via for p in board.pads)   # the input board is not modified
    assert len([p for p in result.board.pads if p.is_via]) >= stats["vias"]
    # Every via joins traces on both layers, and is drawn from its centre.
    for via in result.vias:
        ends = [w for w, c in result.connections.items() if via.pad_id in (c.src, c.dst) and w in result.polylines]
        assert {result.wire_layer[w] for w in ends} == {0, 1}
    assert all(net_is_connected(result, net) for net in range(4))


def test_raced_variants_hand_their_vias_back():
    """Section 12.5: the kept variant's via sites are made again on the parent's maps."""
    board = crossing_board()
    result = route_board(board, options=Options(portfolio=3), workers=3, seed=2)
    assert_clean(board, result)
    assert result.stats["routed"] == 4 and result.stats["vias"] >= 2
    for layer in result.layers:
        assert layer.state.check_invariants()
        assert len(layer.pmap.sites) == result.stats["via_sites"]
    # Vias that were moved (13.3) stand on the parent's maps where the variant's traces end.
    assert all(net_is_connected(result, net) for net in range(4))
    for via in result.vias:
        ends = [result.polylines[w][0 if c.src == via.pad_id else -1] for w, c in result.connections.items()
                if via.pad_id in (c.src, c.dst) and w in result.polylines]
        assert len(ends) == 2 and all(math.dist(p, via.centre) < 1e-6 for p in ends)
        assert all(math.dist(layer.pmap.sites[via.pad_id].centre, via.centre) < 1e-9 for layer in result.layers)


def test_vias_slide_to_where_their_traces_run_straighter():
    """M15: with sliding the same connections and vias, shorter copper, a clean check, every trace still on its via."""
    board = crossing_board()
    # Without Phase 4, so that the vias are where the negotiation left them.
    fixed = route_board(board, options=Options(portfolio=1, slide=False, refine=False), workers=1)
    slid = route_board(board, options=Options(portfolio=1, refine=False), workers=1)
    assert_clean(board, slid)
    assert slid.stats["routed"] == fixed.stats["routed"] == 4 and slid.stats["vias"] == fixed.stats["vias"]
    before = {v.pad_id: v.centre for v in fixed.vias}
    moved = [math.dist(v.centre, before[v.pad_id]) for v in slid.vias]
    assert max(moved) > 0.05 and slid.stats["length"] < fixed.stats["length"] - 0.05
    assert all(net_is_connected(slid, net) for net in range(4))
    for via in slid.vias:
        ends = [slid.polylines[w][0 if c.src == via.pad_id else -1] for w, c in slid.connections.items()
                if via.pad_id in (c.src, c.dst) and w in slid.polylines]
        assert len(ends) == 2 and all(math.dist(p, via.centre) < 1e-6 for p in ends)
    for layer in slid.layers:
        assert layer.state.check_invariants()


def test_vias_keep_clear_of_copper_on_a_layer_that_is_not_routed():
    """A layer left out of the routing (``Board.unrouted``) has no map, but a
    via goes through it: vias keep clear of fixed copper there, and traces on
    the routed layers pass over it."""
    from weaveengine.board import Obstacle
    band = Polygon([(8, 0), (14.4, 0), (14.4, 20), (8, 20)])   # the wall's left side, where vias go
    board = two_layer_wall_board()
    assert any(v.shape.intersects(band) for v in route_board(board, options=Options(portfolio=1), workers=1).vias)
    board.unrouted = ["In1.Cu"]
    board.obstacles.append(Obstacle(band, 7, frozenset({2})))   # copper of another net, on In1
    result = route_board(board, options=Options(portfolio=1), workers=1)
    assert result.unrouted == [] and result.stats["routed"] == 3
    assert all(v.shape.distance(band) >= board.rules.clearance - 1e-6 for v in result.vias)
    assert any(LineString(line).intersects(band) for line in result.polylines.values())
    assert_clean(board, result)


def test_a_pad_gets_a_via_down_to_its_plane_on_a_layer_that_is_not_routed():
    """Ground poured on In1, which is not routed (a board with its planes
    inside, routed on the outer layers): each front pad of ground gets a via
    into it, clear of the copper of other nets there."""
    from shapely.geometry import box as rect
    from weaveengine.board import Obstacle
    front = frozenset({0})
    board = Board.rectangle(30, 20, Rules(0.25, 0.2), layers=["F.Cu", "B.Cu"])
    board.unrouted = ["In1.Cu"]
    board.pads += [Pad.rect(0, 5, 10, 1.0, 1.4, 0, layers=front), Pad.rect(1, 25, 10, 1.0, 1.4, 0, layers=front),
                   Pad.rect(3, 5, 15, 1.0, 1.0, 1, layers=front), Pad.rect(4, 25, 15, 1.0, 1.0, 1, layers=front)]
    board.plane_nets = {0}
    board.pours = [(0, 2, rect(1, 1, 29, 19))]
    track = rect(5.6, 8, 6.4, 12)   # copper of another net on In1, right of pad 0, where its via goes otherwise
    board.obstacles.append(Obstacle(track, 5, frozenset({2})))
    result = route_board(board, options=Options(portfolio=1), workers=1)
    assert result.unrouted == [] and result.stats["fanout"] == 2 and result.stats["plane_left"] == []
    assert all(v.shape.distance(track) >= board.rules.clearance - 1e-6 for v in result.vias)


def fine_pitch_board() -> Board:
    """A row of pads 0.3 mm wide and 0.2 mm apart (their keep-off rings merge),
    each joined to a pad straight above it, with the far pads in mirrored order
    so that the traces have to fan across one another's exits."""
    board = Board.rectangle(12, 10, Rules(0.15, 0.16))
    for i in range(6):
        board.pads.append(Pad.rect(i, 4.75 + 0.5 * i, 2.0, 0.3, 0.85, net_id=i))
        board.pads.append(Pad.rect(10 + i, 2.0 + 1.6 * (5 - i), 8.0, 0.8, 0.8, net_id=i))
    return board


def test_a_trace_leaves_a_fine_pitch_pad_clear_of_its_neighbours():
    """13.1 step 4: the stub from a pad's centre to its keep-off ring must keep
    its clearance from the next pad. Over the gap between two pads it does not,
    so that part of the ring is no way out."""
    board = fine_pitch_board()
    pmap = planar_map.build(board)
    row = [p.pad_id for p in board.pads if p.pad_id < 6]
    assert all(pad in pmap.pad_edges for pad in row)                   # every pad can still be left
    restricted = {pmap.edge_owner_list[e] for e in pmap.exit_window}
    assert set(row) <= restricted                                      # ... but none of them just anywhere
    need = board.rules.clearance + board.rules.trace_width / 2.0
    pads = {p.pad_id: p for p in board.pads}
    for e, (a, b) in pmap.exit_window.items():
        pad = pads[pmap.edge_owner_list[e]]
        (ux, uy), (vx, vy) = (pmap.vxy[v] for v in pmap.edge_v_list[e])
        length = pmap.edge_len_list[e]
        for s in (a, (a + b) / 2.0, b):
            stub = LineString([pad.centre, (ux + s / length * (vx - ux), uy + s / length * (vy - uy))])
            assert all(stub.distance(o.shape) >= need - 2e-4 for o in board.pads if o.net_id != pad.net_id)
    result = route_board(board, options=Options(portfolio=1), workers=1, drc_rounds=0, drop_violators=False)
    assert result.unrouted == [] and result.violations == []          # clean at the first check, with no repair
    assert_clean(board, result)


def test_a_via_plan_that_does_not_fit_leaves_no_trace(monkeypatch):
    """12.3: a route through vias goes in whole or not at all. If it fails after
    its sites are made and its pieces laid, the maps and states are as before."""
    from weaveengine.plan.context import Context, Layer, decompose
    from weaveengine.plan.path import find
    from weaveengine.topo import sites
    board = crossing_board()
    layers = [Layer(i, name, planar_map.build(board, i)) for i, name in enumerate(board.layers)]
    ctx = Context(board, layers, decompose(board, layers)[0], options=Options(portfolio=1), workers=1)
    tried = 0
    for conn in list(ctx.conns.values()):
        path = find(ctx, conn, hard_cap=True)
        assert path is not None
        if path.vias and not tried:
            tried += 1
            before = [(l.state.snapshot(), len(sites.log(l.pmap)), sorted(l.pmap.sites), l.pmap.num_edges) for l in layers]

            def no(*args, **kwargs):
                raise ValueError("no room after all")

            monkeypatch.setattr(sites, "legalise", no)
            assert ctx.commit(conn, path) is False
            monkeypatch.undo()
            assert conn.wire_id in ctx.unrouted and not conn.pieces and len(ctx.conns) == 4
            for layer, (snap, log, live, edges) in zip(layers, before):
                now = layer.state.snapshot()
                assert now[2] == snap[2] and now[0][:edges] == snap[0] and now[1][:len(snap[1])] == snap[1]
                assert not any(now[0][edges:]) and len(sites.log(layer.pmap)) == log and sorted(layer.pmap.sites) == live
                assert layer.state.invariant_errors() == []
        assert ctx.commit(conn, path)
        assert all(layer.state.invariant_errors() == [] for layer in layers)
    assert tried and not ctx.unrouted
    # Ripping a connection through vias takes its pieces and its vias out together.
    split = next(c for c in ctx.conns.values() if c.pieces)
    pieces, pads = split.pieces, split.sites
    ctx.rip(pieces[1])
    assert split.wire_id in ctx.unrouted and not any(w in ctx.conns for w in pieces)
    assert all(pad not in layer.pmap.sites for pad in pads for layer in layers)
    assert all(layer.state.invariant_errors() == [] for layer in layers)


def test_a_connection_lifted_is_put_back_exactly():
    """Phase 4 tries another plan for a connection through vias and, if it is no better, has everything as it was."""
    from weaveengine import parallel, router
    from weaveengine.plan.context import Context, Layer, decompose
    from weaveengine.plan.path import find
    from weaveengine.topo import sites
    from weaveengine.topo.costs import CostParams
    board = crossing_board(6)
    layers = [Layer(i, name, pmap) for i, (name, pmap) in enumerate(zip(board.layers, parallel.run(router._build_task, board, range(2), 1)))]
    pairs, _ = decompose(board, layers)
    ctx = Context(board, layers, pairs, CostParams.for_map(layers[0].pmap), Options(portfolio=1, slide=False), 0, 1, None, None)
    router._route_once(ctx, 4, True)
    through = [c for c in ctx.conns.values() if c.parent is None and c.sites]
    assert len(through) >= 2

    sizes = [(l.pmap.num_edges, l.pmap.num_triangles) for l in ctx.layers]

    def everything():
        """All that a connection's coming and going touches. (The tables keep the room a refused plan made them
        take, empty: compared as far as they went before. A step's stored place on its gate is the one it was put
        in at, and says nothing later: left out.)"""
        out = []
        for l, (edges, tris) in zip(ctx.layers, sizes):
            l.pmap.catch_up()
            orders, corners = l.state.snapshot()[:2]
            assert not any(orders[edges:]) and not any(any(c) for c in corners[tris:])
            out.append((orders[:edges], corners[:tris], {w: [step[:3] for step in steps] for w, steps in l.state.wire_path.items()},
                        l.state.cap[:edges].round(9).tolist(), l.state.load[:edges].round(9).tolist(),
                        l.pmap.tri_v_list[:tris], l.pmap.edge_v_list[:edges], l.pmap.edge_t_list[:edges],
                        [round(x, 9) for x in l.pmap.edge_len_list[:edges]], l.pmap.trans[:2 * edges],
                        {pad: (s.centre, s.net, s.keep, sorted(s.spokes)) for pad, s in l.pmap.sites.items()},
                        {w: p.gates for w, p in l.paths.items()}))
        return out, {w: (c.layer, c.pieces, c.sites, c.parent, c.src, c.dst) for w, c in ctx.conns.items()}, sorted(ctx.unrouted)

    before = everything()
    tried = 0
    for conn in through:
        # Lifted and put straight back.
        saved = ctx.lift(conn)
        assert not conn.routed and everything() != before
        ctx.put_back(conn, saved)
        assert everything() == before
        # Searching while it is out changes nothing either; nor does a plan put in and refused once it is in.
        saved = ctx.lift(conn)
        plan = find(ctx, conn, hard_cap=True, congestion=False)
        asked = []
        assert plan is not None and not ctx.commit(conn, plan, within=lambda pieces: asked.append(len(pieces)) or False)
        tried += bool(asked)
        ctx.put_back(conn, saved)
        assert everything() == before
    assert tried >= 2   # the plans did go in, with their vias, before they were refused
    for layer in ctx.layers:
        assert layer.state.check_invariants()



def test_a_routing_is_gone_on_from_and_never_made_worse():
    """Routing a board that is routed already goes on from that routing: its
    state comes back with the result, survives being handed to another
    process, and the outcome is the earlier one unless something is gained."""
    import pickle
    board = crossing_board()
    first = route_board(board, options=Options(portfolio=1, vias=False), workers=1)   # a first routing with room left to better it
    assert first.stats["improved"] is None and first.kept is not None and first.stats["routed"] == 2
    kept = pickle.loads(pickle.dumps(first.summary().kept))
    again = route_board(board, options=Options(portfolio=1), workers=1, resume=kept)
    assert_clean(board, again)
    assert again.stats["improved"] is True and again.stats["routed"] == 4 and again.stats["vias"] >= 2
    assert all(net_is_connected(again, net) for net in range(4))
    # Nothing more to gain: the routing comes back as it was, vias and all.
    once_more = route_board(board, options=Options(portfolio=1), workers=1, resume=again.kept)
    if once_more.stats["improved"] is False:
        assert once_more.kept["lines"] == again.kept["lines"] and once_more.stats["vias"] == again.stats["vias"]
    assert once_more.kept["key"] <= again.kept["key"]
    assert_clean(board, once_more)
    # With raced variants too; and each of them starts from the routing, not from nothing.
    raced = route_board(board, options=Options(portfolio=3), workers=3, resume=first.kept)
    assert raced.kept["key"] <= first.kept["key"]
    assert_clean(board, raced)
    # Another board, or other rules, and there is nothing to go on from.
    other = crossing_board()
    other.rules.clearance += 0.05
    with pytest.raises(ValueError, match="changed"):
        route_board(other, options=Options(portfolio=1), workers=1, resume=first.kept)


def test_a_pad_no_trace_can_reach_leaves_its_connection_open_and_counted():
    """A pad inside a keepout (or packed closer than the clearance allows, or off
    the board) cannot be reached: its connection is open, and counted as such,
    not left out of the count."""
    from shapely.geometry import box
    from weaveengine.board import Obstacle
    board = Board.rectangle(30, 10, Rules(0.2, 0.2))
    board.pads += [Pad.rect(0, 3, 5, 1, 1, net_id=0), Pad.rect(1, 27, 5, 1, 1, net_id=0), Pad.rect(2, 15, 5, 1, 1, net_id=0)]
    board.obstacles.append(Obstacle(box(13, 3, 17, 7)))
    result = route_board(board, options=Options(portfolio=1), workers=1)
    assert result.stats["buried_pads"] == [2]
    assert result.stats["connections"] == 2 and result.stats["routed"] == 1 and result.stats["unreachable"] == 1


def test_pads_whose_copper_touches_are_joined_already():
    """Two pads of a net that overlap (front and back of a pin, a custom pad's
    pieces) are one piece of copper: no connection is made between them."""
    board = Board.rectangle(20, 10, Rules(0.2, 0.2))
    board.pads += [Pad.rect(0, 5, 5, 1.0, 1.0, 0), Pad.rect(1, 5.6, 5, 1.0, 1.0, 0), Pad.circle(2, 15, 5, 0.5, 0)]
    pairs, buried = decompose(board, [Layer(0, "F.Cu", planar_map.build(board))])
    assert buried == [] and len(pairs) == 1 and {pairs[0][1], pairs[0][2]} != {0, 1}
    result = route_board(board)
    assert result.unrouted == [] and len(result.polylines) == 1


def pins_board(min_width: float) -> Board:
    """Three pins 0.2 wide at 0.398 pitch, each to a pad of its own far off. A
    trace joins a pin at its centre, 0.298 from the next pin's edge: room for a
    0.2 trace at 0.19 clearance, not for the class's 0.25, nor for 0.2 with all
    of the 0.01 margin (0.3)."""
    rules = Rules(0.25, 0.19 + 0.01, margin=0.01, min_width=min_width)
    board = Board.rectangle(20, 10, rules)
    for net, y in enumerate((4.602, 5.0, 5.398)):
        board.pads.append(Pad.rect(2 * net, 10, y, 0.875, 0.2, net))
    board.pads += [Pad.circle(1, 3, 2, 0.5, 0), Pad.circle(3, 17, 5, 0.5, 1), Pad.circle(5, 3, 8, 0.5, 2)]
    return board


def test_a_net_is_narrowed_to_reach_a_fine_pitch_pin():
    plain = route_board(pins_board(0.2), options=Options(narrow=False))
    assert plain.stats["buried_pads"] == [0, 2, 4] and plain.stats["narrowed"] == {}
    unknown = route_board(pins_board(0.0))  # no least width known: nothing is narrowed
    assert unknown.stats["buried_pads"] == [0, 2, 4]
    board = pins_board(0.2)
    result = route_board(board)
    assert result.stats["buried_pads"] == [] and result.unrouted == []
    assert result.stats["narrowed"] == {n: pytest.approx(0.2) for n in range(3)}  # drawn at the least width the rules allow,
    assert result.board.rules.width(1) == pytest.approx(0.19)                    # routed with half the margin
    assert board.rules.net_width == {} and board.rules.net_drawn == {}   # the board routed is left as it was
    assert_clean(board, result)


def test_fixed_copper_of_a_plane_net_joins_its_pads_to_the_plane():
    """Ground poured on the back. Pad 0 is joined to it already, by a locked
    trace to a locked via: it gets nothing. Pad 1 has a locked via of its net
    close by in the pour: it is joined to that by a short trace, no new via.
    Pad 2's only room for a via, a gap in a keepout on the back, holds a
    locked via too near the pour's edge to share: a new via there would be
    drilled into it, so the pad is left (and named)."""
    from shapely.geometry import box as rect
    from weaveengine.board import Obstacle, circle_polygon
    front = frozenset({0})
    board = Board.rectangle(30, 20, Rules(0.25, 0.2), layers=["F.Cu", "B.Cu"])
    board.pads += [Pad.rect(0, 5, 10, 1.0, 1.4, 0, layers=front), Pad.rect(1, 25, 5, 1.0, 1.4, 0, layers=front),
                   Pad.rect(2, 27.5, 15, 1.0, 1.4, 0, layers=front),
                   Pad.rect(3, 10, 3, 1.0, 1.0, 1, layers=front), Pad.rect(4, 20, 3, 1.0, 1.0, 1, layers=front)]
    board.plane_nets = {0}
    board.pours = [(0, 1, rect(1, 1, 29, 19))]
    r = board.rules.via_diameter / 2
    board.obstacles += [Obstacle(LineString([(5, 10), (7, 10)]).buffer(0.125), 0, front),   # pad 0's locked trace
                        Obstacle(circle_polygon(7, 10, r), 0, via=True),                     # and via
                        Obstacle(circle_polygon(25, 6.5, r), 0, via=True),                   # beside pad 1
                        Obstacle(circle_polygon(28.85, 15, r), 0, via=True),                 # right of pad 2, at the pour's edge
                        Obstacle(rect(20, 10, 29.5, 20).difference(rect(28, 14.4, 29.2, 15.6)), layers=frozenset({1}))]
    result = route_board(board, options=Options(portfolio=1), workers=1)
    assert result.unrouted == [] and result.stats["fanout"] == 0 and result.stats["plane_left"] == [2]
    plane = {k: line for w, line in result.polylines.items() if result.wire_net[w] == 0
             for k in range(3) if math.dist(line[0], board.pads[k].centre) < 1e-9}   # the trace from each pad
    assert set(plane) == {1} and math.dist(plane[1][-1], (25, 6.5)) < 1e-9          # pad 1's, to the locked via
    # Without that locked via, pad 2 gets one of its own in the gap.
    board.obstacles.pop(-2)
    again = route_board(board, options=Options(portfolio=1), workers=1)
    assert again.stats["fanout"] == 1 and again.stats["plane_left"] == []
    assert next(v for v in again.vias if v.fanout).centre[0] > 28


def test_copper_kicad_does_not_join_leaves_a_pad_unreached():
    """An opening in the solder mask (or text) over pads of a plane net is given
    that net, but joins nothing: a pad the plane reaches only through it is
    still given a way there. A locked trace over the same place does join it."""
    from shapely.geometry import box as rect
    from weaveengine.board import Obstacle
    from weaveengine.plan.fanout import _lonely
    front = frozenset({0})
    board = Board.rectangle(20, 20, Rules(0.25, 0.2), layers=["F.Cu", "B.Cu"])
    board.pads += [Pad.rect(0, 5, 10, 1, 1, 0, layers=front), Pad.rect(1, 12, 10, 1, 1, 0, layers=front)]
    board.plane_nets = {0}
    board.pours = [(0, 0, rect(10, 5, 15, 15))]                                               # reaches pad 1 only
    board.obstacles.append(Obstacle(rect(4.5, 9.5, 12.5, 10.5), 0, front, copper=True))      # the mask open over both
    assert [p.pad_id for p in _lonely(board)] == [0]
    board.obstacles.append(Obstacle(rect(4.5, 9.9, 12.5, 10.1), 0, front))
    assert _lonely(board) == []


def test_a_pad_its_plane_does_not_reach_gets_a_via_down_to_it():
    """Ground poured on the back only: each front pad of ground gets a via
    beside it, inside the pour, joined by a short trace; a pad the pour reaches
    on its own layer gets none, and the signal is routed as before."""
    from shapely.geometry import box as rect
    from weaveengine.plan.fanout import covered
    board = Board.rectangle(30, 20, Rules(0.25, 0.2), layers=["F.Cu", "B.Cu"])
    front, back = frozenset({0}), frozenset({1})
    board.pads += [Pad.rect(0, 5, 10, 1.0, 1.4, 0, layers=front), Pad.rect(1, 25, 10, 1.0, 1.4, 0, layers=front),
                   Pad.rect(2, 15, 5, 1.0, 1.4, 0, layers=back),
                   Pad.rect(3, 5, 15, 1.0, 1.0, 1, layers=front), Pad.rect(4, 25, 15, 1.0, 1.0, 1, layers=front)]
    board.plane_nets = {0}
    board.pours = [(0, 1, rect(1, 1, 29, 19))]
    assert [covered(board, p) for p in board.pads[:3]] == [False, False, True]
    result = route_board(board)
    assert result.unrouted == [] and result.stats["fanout"] == 2 and result.stats["plane_left"] == []
    placed = [v for v in result.vias if v.fanout]
    rules = board.rules
    for pad in board.pads[:2]:
        via = min(placed, key=lambda v: math.dist(v.centre, pad.centre))
        assert pad.shape.distance(Point(via.centre)) < rules.via_diameter / 2 + 0.5   # beside it
        stub = [line for w, line in result.polylines.items() if result.wire_net[w] == 0 and math.dist(line[0], pad.centre) < 1e-9]
        assert stub and math.dist(stub[0][-1], via.centre) < 1e-9 and result.wire_layer[[w for w, l in result.polylines.items() if l is stub[0]][0]] == 0
    signal = [LineString(l).buffer(rules.width(1) / 2) for w, l in result.polylines.items() if result.wire_net[w] == 1]
    for via in placed:  # clear of the signal's copper by the clearance
        assert all(s.distance(via.shape) >= rules.clearance - 1e-6 for s in signal)


def test_a_via_down_to_a_plane_keeps_out_of_an_area_that_forbids_vias():
    """A pad its plane does not reach gets its via on another side of it when
    the nearest place for one is in an area no via may touch (``Board.no_via``)."""
    from shapely.geometry import box as rect
    front = frozenset({0})
    board = Board.rectangle(30, 20, Rules(0.25, 0.2), layers=["F.Cu", "B.Cu"])
    board.pads += [Pad.rect(0, 5, 10, 1.0, 1.4, 0, layers=front), Pad.rect(1, 25, 10, 1.0, 1.4, 0, layers=front),
                   Pad.rect(3, 5, 15, 1.0, 1.0, 1, layers=front), Pad.rect(4, 25, 15, 1.0, 1.0, 1, layers=front)]
    board.plane_nets = {0}
    board.pours = [(0, 1, rect(1, 1, 29, 19))]
    area = rect(5.2, 8, 8, 12)   # the right of pad 0, where its via goes otherwise
    beside = lambda result: min((v for v in result.vias if v.fanout), key=lambda v: math.dist(v.centre, (5, 10)))
    assert beside(route_board(board, options=Options(portfolio=1), workers=1)).shape.intersects(area)
    board.no_via = [area]
    result = route_board(board, options=Options(portfolio=1), workers=1)
    assert result.unrouted == [] and result.stats["fanout"] == 2 and result.stats["plane_left"] == []
    assert not any(v.shape.intersects(area) for v in result.vias)
    assert board.pads[0].shape.distance(Point(beside(result).centre)) < board.rules.via_diameter / 2 + 0.5   # still beside it
