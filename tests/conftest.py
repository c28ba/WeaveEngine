import random

import pytest

from weaveengine.board import Board, Pad, Rules
from weaveengine.topo import planar_map


def grid_board(n: int = 5, pitch: float = 10.0, seed: int = 1, radius: float = 1.0) -> Board:
    """n x n jittered pads, each on its own net id (tests pick the pairs)."""
    rng = random.Random(seed)
    board = Board.rectangle(n * pitch, n * pitch, Rules(0.3, 0.3))
    for i in range(n):
        for j in range(n):
            board.pads.append(Pad.circle(i * n + j, pitch / 2 + pitch * i + rng.uniform(-1, 1),
                                         pitch / 2 + pitch * j + rng.uniform(-1, 1), radius, i * n + j))
    return board


def demo_board() -> Board:
    board = Board.rectangle(100.0, 60.0, Rules(trace_width=1.0, clearance=0.8))
    for pad_id, x, y, net in [(101, 10, 50, 1), (102, 90, 10, 1), (201, 10, 10, 2), (202, 90, 50, 2), (301, 10, 30, 3), (302, 90, 30, 3)]:
        board.pads.append(Pad.circle(pad_id, x, y, 1.5, net))
    board.add_keepout([(40, 20), (60, 20), (60, 40), (40, 40)])
    return board


@pytest.fixture
def grid():
    board = grid_board()
    return board, planar_map.build(board)
