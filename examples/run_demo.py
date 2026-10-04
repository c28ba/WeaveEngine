"""Smallest demo: two nets whose straight lines cross."""
import os

from weaveengine.board import Board, Pad, Rules
from weaveengine.router import route_board
from weaveengine.viz.svg import export_result

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "output")


def main():
    board = Board.rectangle(30.0, 20.0, Rules(trace_width=0.3, clearance=0.3))
    for pad_id, x, y, net in [(100, 5.0, 5.0, 1), (200, 25.0, 15.0, 1), (300, 5.0, 15.0, 2), (400, 25.0, 5.0, 2)]:
        board.pads.append(Pad.circle(pad_id, x, y, radius=0.8, net_id=net))

    result = route_board(board)
    print(f"Routed {result.stats['routed']}/{result.stats['connections']} connections, "
          f"{len(result.violations)} DRC violations, length ratio {result.stats['length_ratio']:.3f}")
    export_result(result, os.path.join(OUT, "routed_mesh.svg"), mesh=True)
    print("Saved routed_mesh.svg")


if __name__ == "__main__":
    main()
