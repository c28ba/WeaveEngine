"""Synthetic board demo: three nets whose airwires cross, around a central keepout."""
import json
import os

from weaveengine.board import Board, Pad, Rules
from weaveengine.router import route_board
from weaveengine.viz.svg import export_result

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "output")


def build_board() -> Board:
    board = Board.rectangle(100.0, 60.0, Rules(trace_width=1.0, clearance=0.8))
    for pad_id, x, y, net in [
        (101, 10.0, 50.0, 1), (102, 90.0, 10.0, 1),
        (201, 10.0, 10.0, 2), (202, 90.0, 50.0, 2),
        (301, 10.0, 30.0, 3), (302, 90.0, 30.0, 3),
    ]:
        board.pads.append(Pad.circle(pad_id, x, y, radius=1.5, net_id=net))
    board.add_keepout([(40.0, 20.0), (60.0, 20.0), (60.0, 40.0), (40.0, 40.0)])
    return board


def main():
    board = build_board()
    result = route_board(board)
    stats = result.stats

    print(f"Routed {stats['routed']}/{stats['connections']} connections, "
          f"{len(result.violations)} DRC violations, length ratio {stats['length_ratio']:.3f}, "
          f"{stats['time_total']:.2f}s")
    for wire_id, conn in result.connections.items():
        status = "UNROUTED" if wire_id in result.unrouted else f"{len(result.polylines[wire_id])} points"
        print(f"  net {conn.net_id}: pad {conn.src} -> pad {conn.dst}: {status}")

    export_result(result, os.path.join(OUT, "synthetic_board.svg"), mesh=True)
    with open(os.path.join(OUT, "route_analysis.json"), "w") as f:
        json.dump({
            "stats": stats,
            "invariants_valid": result.state.check_invariants(),
            "violations": [vars(v) for v in result.violations],
            "nets": {
                str(wire_id): {
                    "net": conn.net_id, "from_pad": conn.src, "to_pad": conn.dst,
                    "status": "UNROUTED" if wire_id in result.unrouted else "ROUTED",
                    "gates": list(result.state.wire_path.get(wire_id, []) and [s[0] for s in result.state.wire_path[wire_id]]),
                    "polyline": result.polylines.get(wire_id, []),
                } for wire_id, conn in result.connections.items()
            },
        }, f, indent=2)
    print("Wrote synthetic_board.svg and route_analysis.json to examples/output")


if __name__ == "__main__":
    main()
