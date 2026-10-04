"""Three nets through a field of unconnected obstacle pins."""
import os

from weaveengine.board import Board, Pad, Rules
from weaveengine.router import route_board
from weaveengine.viz.svg import export_result

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "output")


def create_complex_board() -> Board:
    board = Board.rectangle(50.0, 40.0, Rules(trace_width=0.15, clearance=0.15))
    pads = [
        (101, 5.0, 5.0, 1), (102, 45.0, 5.0, 1),     # net 1: left to right along the bottom
        (201, 5.0, 35.0, 2), (202, 45.0, 35.0, 2),   # net 2: left to right along the top
        (301, 25.0, 2.0, 3), (302, 25.0, 38.0, 3),   # net 3: bottom-centre to top-centre
    ]
    for pad_id, x, y, net in pads:
        board.pads.append(Pad.circle(pad_id, x, y, radius=0.6, net_id=net))
    # Unconnected pins in the middle channel.
    for i, (x, y) in enumerate([(15, 15), (35, 15), (15, 25), (35, 25), (25, 20), (25, 10)]):
        board.pads.append(Pad.circle(900 + i, x, y, radius=0.6))
    return board


def main():
    board = create_complex_board()
    result = route_board(board)
    print(f"Routed {result.stats['routed']}/{result.stats['connections']} connections, "
          f"{len(result.violations)} DRC violations, length ratio {result.stats['length_ratio']:.3f}")
    export_result(result, os.path.join(OUT, "complex_mesh.svg"), mesh=True)
    print("Exported to complex_mesh.svg")


if __name__ == "__main__":
    main()
