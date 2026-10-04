import numpy as np
import pytest

from bench.generators import suite
from tests.conftest import demo_board
from weaveengine.board import Board, Pad, Rules
from weaveengine.geom.capacity import edge_capacity
from weaveengine.geom.inflate import preprocess
from weaveengine.geom.triangulate import check_triangulation, triangulate
from weaveengine.topo import planar_map
from weaveengine.topo.planar_map import GATE, TERMINAL, WALL_EDGE


@pytest.mark.parametrize("name,board", list(suite().items()) + [("demo", demo_board())])
def test_triangulation_checks(name, board):
    """17.1: every boundary segment is an edge, no triangle in a hole, areas add up."""
    fs = preprocess(board)
    verts, tris, v_obs, segs, _ = triangulate(fs)
    assert check_triangulation(fs, verts, tris, segs) == []
    assert (v_obs >= 0).all()  # no Steiner points: every vertex lies on an obstacle


def test_scipy_fallback_matches():
    board = demo_board()
    fs = preprocess(board)
    verts, tris, _, segs, _ = triangulate(fs, force_fallback=True)
    assert check_triangulation(fs, verts, tris, segs) == []


def test_edge_kinds_and_terminal_ownership():
    board = demo_board()
    pmap = planar_map.build(board)
    assert set(pmap.pad_edges) == {p.pad_id for p in board.pads}
    for e in range(pmap.num_edges):
        two_sided = pmap.edge_t_list[e][1] >= 0
        assert (pmap.edge_kind_list[e] == GATE) == two_sided
        assert (pmap.edge_owner_list[e] >= 0) == (pmap.edge_kind_list[e] == TERMINAL)
        u, v = pmap.edge_v_list[e]
        assert u < v
    # Terminal edges hug their own pad at the centreline keep-off distance.
    pads = {p.pad_id: p for p in board.pads}
    for pad_id, edges in pmap.pad_edges.items():
        for e in edges:
            from shapely.geometry import Point
            assert pads[pad_id].shape.distance(Point(pmap.edge_mid_list[e])) >= board.rules.inflation - 1e-6


def test_overlapping_inflated_pads_merge():
    board = Board.rectangle(10, 10, Rules(0.2, 0.2))
    board.pads.append(Pad.circle(0, 4.6, 5, 0.3, 0))
    board.pads.append(Pad.circle(1, 5.4, 5, 0.3, 1))  # gap 0.2 < 2 * inflation: no centreline fits
    pmap = planar_map.build(board)
    assert pmap.pad_obs[0] == pmap.pad_obs[1]


def test_capacity_formula():
    """cap = floor(w / (t + s)) + 1, walls 0, with the slanted-edge distance terms."""
    from shapely.geometry import LinearRing
    verts = np.array([[0.0, 0.0], [3.0, 4.0], [0.0, 1.05]])
    edge_v = np.array([[0, 1], [0, 2], [0, 2]])
    edge_len = np.array([5.0, 1.05, 1.05])
    kind = np.array([GATE, GATE, WALL_EDGE])
    v_obs = np.array([1, 2, 3])
    geom = {1: LinearRing([(0, 0), (-1, 0), (-1, -1)]),
            2: LinearRing([(3, 4), (0, 0.9), (0, 4)]),   # passes 0.9 from vertex 0
            3: LinearRing([(0, 1.05), (-1, 2), (0, 2)])}
    cap = edge_capacity(verts, edge_v, edge_len, kind, v_obs, geom, pitch=0.5)
    assert cap.tolist() == [2, 3, 0]  # floor(0.9/0.5)+1, floor(1.05/0.5)+1, wall
