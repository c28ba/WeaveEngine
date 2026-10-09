"""A connection and the copper its net already has (design section 11): it
joins a trace of its net where that saves copper, runs through a pad of its
net that is in its way, and changes layer where its net already does."""
import math

import pytest
from shapely.geometry import LineString, Point

from tests.test_router import assert_clean, copper, net_is_connected
from weaveengine import parallel, router
from weaveengine.board import Board, Pad, Rules
from weaveengine.plan.context import Context, Layer, Options, decompose
from weaveengine.router import route_board
from weaveengine.topo import planar_map
from weaveengine.topo.costs import CostParams
from weaveengine.topo.search import route
from weaveengine.topo.state import TopoState

FRONT, BACK = frozenset({0}), frozenset({1})


def wall_board() -> Board:
    """Pads 0 and 1 of a net left of a long wall's two sides, 2 and 3 beyond its
    end. The net's tree joins 0 to 1, the nearest, which the wall sends all the
    way round; 1 to 3 runs straight along the wall."""
    board = Board.rectangle(50, 30, Rules(0.2, 0.2))
    for pad_id, x, y in [(0, 5, 11), (1, 5, 19), (2, 45, 11), (3, 45, 19)]:
        board.pads.append(Pad.circle(pad_id, x, y, 0.6, net_id=0))
    board.add_keepout([(0, 14.5), (40, 14.5), (40, 15.5), (0, 15.5)])
    return board


def pad_at_the_wall() -> Board:
    """Pads 0 and 1 of net 0 either side of a wall, and a large pad 2 of the
    same net standing off its end, where the way from 0 to 1 goes. Net 1 has to
    get round the wall's end as well, between the wall and the pad."""
    board = Board.rectangle(40, 20, Rules(0.2, 0.2))
    board.pads += [Pad.circle(0, 10, 8, 0.6, 0), Pad.circle(1, 10, 12, 0.6, 0), Pad.rect(2, 32, 10, 2.5, 5, 0),
                   Pad.circle(3, 20, 8, 0.5, 1), Pad.circle(4, 20, 12, 0.5, 1)]
    board.add_keepout([(0, 9.6), (30, 9.6), (30, 10.4), (0, 10.4)])
    return board


def filled(board: Board, step: float = 4.0) -> Board:
    """The board with unconnected through-hole pads on a grid, clear of its own
    pads: they keep the triangles small, as on a real board. (In a large
    triangle the search cannot tell where a via stands, 13.3.)"""
    width, height = board.outline.bounds[2:]
    own = [p.centre for p in board.pads]
    spots = [(step / 2 + i * step, step / 2 + j * step) for i in range(int(width / step)) for j in range(int(height / step))]
    for n, spot in enumerate(s for s in spots if all(math.dist(s, c) > 2.0 for c in own)):
        board.pads.append(Pad.circle(1000 + n, *spot, 0.3))
    return board


def one_front_two_back() -> Board:
    """Front pad 0 is joined to back pads 1 and 2, which are further from each other than from it."""
    board = Board.rectangle(30, 24, Rules(0.2, 0.2), layers=["F.Cu", "B.Cu"])
    board.pads += [Pad.rect(0, 10, 12, 1.2, 0.8, 0, layers=FRONT), Pad.rect(1, 20, 6, 1.2, 0.8, 0, layers=BACK),
                   Pad.rect(2, 20, 18, 1.2, 0.8, 0, layers=BACK)]
    return filled(board)


def routed(board: Board) -> Context:
    """The board routed, as the context the router worked in."""
    layers = [Layer(i, name, pmap) for i, (name, pmap) in enumerate(zip(board.layers, parallel.run(router._build_task, board, range(len(board.layers)), 1)))]
    pairs, _ = decompose(board, layers)
    ctx = Context(board, layers, pairs, CostParams.for_map(layers[0].pmap), Options(portfolio=1, slide=False), 0, 1, None, None)
    router._route_once(ctx, 4, True)
    return ctx


def test_a_search_takes_the_detour_that_leads_to_its_nets_trace():
    """Beside a trace of its net a wire is nearly free, and the search's
    estimate of what is left has to know that: taken as a full length, it turns
    the search away from every way that first goes further from the goal."""
    board = wall_board()
    pmap = planar_map.build(board)
    state = TopoState(pmap)
    state.insert(1, route(pmap, state, 1, 3, net=0).steps, net=0)   # straight along the far side of the wall
    alone = route(pmap, state, 0, 1)
    found = route(pmap, state, 0, 1, net=0)
    # Round the wall's end and all the way back is new trace twice the wall's
    # length. Round the end and then with the trace that is there, it is half that.
    assert alone.cost > 72 and alone.length == pytest.approx(alone.cost)
    assert found.cost < 0.6 * alone.cost and len(set(found.gates) & {step[0] for step in state.wire_path[1]}) > 5
    assert found.length > 72      # as long a way, most of it not new


def test_a_connection_joins_its_nets_trace_instead_of_running_beside_it():
    board = wall_board()
    result = route_board(board)
    assert result.unrouted == [] and net_is_connected(result, 0)
    assert_clean(board, result)
    # Along each side of the wall and across its end: 40 + 8 + 40. Before, the
    # connection 0 to 1 went round the wall and back beside the other: 120.
    assert copper(result) < 90
    for line in result.polylines.values():   # nothing runs back between the wall and the trace along it
        assert LineString(line).distance(Point(20, 17)) > 1.5


def test_a_trace_runs_through_a_pad_of_its_net_that_is_in_its_way():
    board = pad_at_the_wall()
    result = route_board(board)
    assert result.unrouted == [] and net_is_connected(result, 0) and net_is_connected(result, 1)
    assert_clean(board, result)
    # The connection from 0 to 1 is in two pieces that end on pad 2 ...
    whole = next(c for c in result.connections.values() if c.parent is None and (c.src, c.dst) == (0, 1))
    assert [(result.connections[w].src, result.connections[w].dst) for w in whole.pieces] == [(0, 2), (2, 1)]
    # ... and no trace goes round the far side of the pad.
    beyond = LineString([(33.5, 0), (33.5, 20)])
    assert not any(LineString(line).intersects(beyond) for line in result.polylines.values())
    assert copper(result) < 70     # round the pad it was 87
    # The other net cannot do that: it keeps clear of the pad and goes round the wall's end.
    foreign = LineString(next(line for w, line in result.polylines.items() if result.wire_net[w] == 1))
    assert foreign.distance(board.pad(2).shape) >= board.rules.clearance + board.rules.trace_width / 2 - 1e-3


def test_connections_of_a_net_share_a_via():
    board = one_front_two_back()
    result = route_board(board, options=Options(portfolio=1), workers=1)
    assert result.unrouted == [] and net_is_connected(result, 0)
    assert_clean(board, result)
    assert len(result.vias) == 1   # each connection had its own
    via = result.vias[0]
    ends = [(w, result.wire_layer[w]) for w, c in result.connections.items() if via.pad_id in (c.src, c.dst) and w in result.polylines]
    assert sorted(layer for _, layer in ends).count(1) == 2 and any(layer == 0 for _, layer in ends)   # both back traces start from it
    for w, _ in ends:
        conn, line = result.connections[w], result.polylines[w]
        assert math.dist(line[0 if conn.src == via.pad_id else -1], via.centre) < 1e-6


def test_a_shared_via_stays_while_a_trace_is_on_it():
    ctx = routed(one_front_two_back())
    site = ctx.layers[0].pmap.sites
    assert len(site) == 1
    pad = next(iter(site))
    users = [c for c in ctx.conns.values() if c.parent is None and pad in c.sites]
    assert len(users) == 2
    saved = ctx.lift(users[0])
    assert pad in site and site[pad].active                       # the other connection's traces end on it
    assert all(layer.state.check_invariants() for layer in ctx.layers)
    ctx.put_back(users[0], saved)
    assert users[0].routed and all(layer.state.check_invariants() for layer in ctx.layers)
    for conn in users:
        ctx.rip(conn.wire_id)
    assert not site and not any(layer.state.wire_path for layer in ctx.layers)   # gone with the last


def test_a_through_hole_pad_of_the_net_is_the_layer_change():
    """Front pad 0, back pad 2, and a through-hole pad 1 of their net beside 0:
    the tree joins 0 to 1 and 0 to 2, and the second needs no via."""
    board = Board.rectangle(24, 24, Rules(0.2, 0.2), layers=["F.Cu", "B.Cu"])
    board.pads += [Pad.rect(0, 10, 10, 1.2, 0.8, 0, layers=FRONT), Pad.circle(1, 13, 10, 0.6, 0), Pad.rect(2, 10, 16, 1.2, 0.8, 0, layers=BACK)]
    result = route_board(filled(board), options=Options(portfolio=1), workers=1)
    assert result.unrouted == [] and result.vias == [] and net_is_connected(result, 0)
    assert_clean(board, result)
    whole = next(c for c in result.connections.values() if c.parent is None and (c.src, c.dst) == (0, 2))
    assert [(result.connections[w].src, result.connections[w].dst, result.wire_layer[w]) for w in whole.pieces] == [(0, 1, 0), (1, 2, 1)]


def test_pieces_that_share_a_gate_are_put_back_as_they_were():
    """A connection through a pad of its net may leave the pad beside the piece
    it came by. Lifted and put back, every gate has its wires in the order it had."""
    ctx = routed(pad_at_the_wall())
    whole = next(c for c in ctx.conns.values() if len(c.pieces) == 2)

    def orders():
        return [[list(row) for row in layer.state.gate_order] for layer in ctx.layers], [layer.state.load.round(9).tolist() for layer in ctx.layers]

    before = orders()
    for conn in [c for c in ctx.conns.values() if c.parent is None]:
        saved = ctx.lift(conn)
        assert orders() != before
        ctx.put_back(conn, saved)
        assert orders() == before and all(layer.state.check_invariants() for layer in ctx.layers)
    assert whole.routed
