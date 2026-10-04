import itertools
import math
import os

import pytest
from shapely.geometry import LineString, Point, Polygon

from bench.generators import channel, random_grid
from tests.conftest import demo_board
from weaveengine.board import Board, Pad, Rules
from weaveengine.plan.context import Layer, Options, decompose
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
        assert not la.crosses(lb), f"wires {a} and {b} cross"
        if result.wire_net[a] != result.wire_net[b]:
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
    board = demo_board()
    result = route_board(board)
    pads = {p.pad_id: p for p in board.pads}
    assert sum(len(t) for t in result.teardrops.values()) == 6  # both ends of three traces
    for w, drops in result.teardrops.items():
        conn = result.connections[w]
        line = LineString(result.polylines[w])
        for poly in drops:
            shape = Polygon(poly)
            pad = min((pads[conn.src], pads[conn.dst]), key=lambda p: math.dist(p.centre, poly[0]))
            assert poly[0] == pad.centre
            # widest at the pad (90 % of its diameter), narrowing to the trace width
            assert abs(math.dist(poly[1], poly[4]) - 2 * 0.9 * pad.radius * math.sin(math.acos(0.9 * pad.radius / math.dist(pad.centre, _mid(poly[2], poly[3]))))) < 1e-6
            assert abs(math.dist(poly[2], poly[3]) - board.rules.width(conn.net_id)) < 1e-6
            assert line.distance(Point(_mid(poly[2], poly[3]))) < 1e-6
            for other in board.pads:
                if other.net_id != conn.net_id:
                    assert shape.distance(other.shape) >= board.rules.clearance
    assert route_board(board, options=Options(teardrops=False)).teardrops == {}


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
            shape = Polygon(poly)
            assert shape.is_valid and poly[0] == pad.centre
            # the pad end sits on two corners of the pad shrunk to 90 %, inside the pad's copper
            for corner in (poly[1], poly[4]):
                assert pad.shape.contains(Point(corner))
                assert any(math.dist(corner, (pad.centre[0] + 0.9 * (x - pad.centre[0]), pad.centre[1] + 0.9 * (y - pad.centre[1]))) < 1e-9
                           for x, y in pad.shape.exterior.coords)
            assert math.dist(poly[1], poly[4]) > 1.0  # much wider than the trace at the pad
            # the narrow end is the trace itself, outside the pad
            assert abs(math.dist(poly[2], poly[3]) - 0.2) < 1e-6
            assert line.distance(Point(_mid(poly[2], poly[3]))) < 1e-6 and not pad.shape.contains(Point(_mid(poly[2], poly[3])))
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


RAM = os.path.join(os.path.dirname(os.path.dirname(__file__)), "boards", "Word of RAM.dsn")


@pytest.mark.skipif(not os.path.exists(RAM), reason="example board not present")
def test_word_of_ram_board(tmp_path):
    """A real two-layer KiCad board: through-hole relays, two net classes."""
    from weaveengine.io.dsn import parse_sexpr, read_dsn
    from weaveengine.io.ses import write_ses

    design = read_dsn(RAM)
    board = design.board
    assert len(board.pads) == 128 and board.layers == ["F.Cu", "B.Cu"]
    assert {design.net_names[n] for n in board.rules.net_width} == {"+12V", "GND"}
    result = route_board(board)
    assert result.unrouted == [] and result.stats["routed"] == result.stats["connections"] == 85
    assert all(net_is_connected(result, n) for n in board.nets())
    assert_clean(board, result)
    assert min(result.stats["wires_per_layer"].values()) > 10  # both layers are used

    out = str(tmp_path / "ram.ses")
    write_ses(design, result, out)
    session = parse_sexpr(open(out).read())
    routes = next(c for c in session if isinstance(c, list) and c[0] == "routes")
    nets = [c for c in next(c for c in routes if isinstance(c, list) and c[0] == "network_out") if isinstance(c, list)]
    wires = [w[1] for n in nets for w in n[2:] if w[0] == "wire"]
    assert len(wires) == len(result.polylines)
    assert {w[1] for w in wires} == {"F.Cu", "B.Cu"}
    gnd = next(n for n in nets if n[1] == "GND")
    assert {w[1][2] for w in gnd[2:] if w[0] == "wire"} == {"5000"}  # 0.5 mm at 0.1 um
