"""Design-rule check (design section 13.2). shapely in batch, via STRtree."""
from dataclasses import dataclass

import shapely
from shapely.geometry import LineString

from weaveengine.board import Board

TOL = 1e-3  # mm. Traces are simplified to within half a micron, so allow one micron


@dataclass
class Violation:
    kind: str                  # "clearance", "outline", "spacing" or "crossing"
    wires: tuple[int, ...]
    distance: float
    required: float
    at: tuple[float, float]
    layer: int = 0


def check(board: Board, polylines: dict[int, list[tuple[float, float]]], wire_net: dict[int, int], layer: int = 0) -> list[Violation]:
    """Checks the traces of one layer against that layer's copper, the board edge and each other."""
    rules = board.rules
    out: list[Violation] = []
    ids = [w for w, line in polylines.items() if len(line) >= 2]
    if not ids:
        return out
    lines = [LineString(polylines[w]) for w in ids]
    half = [rules.width(wire_net.get(w, -1)) / 2.0 for w in ids]
    widest = max(half)
    edge = rules.outline_inset - rules.base_width / 2.0

    # Trace centreline to foreign copper and keepouts: at least s + t/2.
    copper = [(p.shape, p.net_id) for p in board.pads_on(layer)] + [(o.shape, o.net_id) for o in board.obstacles_on(layer)]
    if copper:
        tree = shapely.STRtree([c[0] for c in copper])
        li, ci = tree.query(lines, predicate="dwithin", distance=rules.clearance + widest - TOL)
        for i, c in zip(li.tolist(), ci.tolist()):
            net = copper[c][1]
            if net >= 0 and net == wire_net.get(ids[i], -1):
                continue
            need = rules.clearance + half[i]
            dist = lines[i].distance(copper[c][0])
            if dist < need - TOL:
                out.append(Violation("clearance", (ids[i],), dist, need, _closest(lines[i], copper[c][0]), layer))

    # Trace to board edge.
    boundary = board.outline.exterior
    for i, line in enumerate(lines):
        need = edge + half[i]
        dist = line.distance(boundary)
        if dist < need - TOL or not board.outline.contains(line):
            out.append(Violation("outline", (ids[i],), dist, need, _closest(line, boundary), layer))

    # Wire to wire: independent crossing oracle (17.1) for every pair, and
    # copper-to-copper spacing s between different nets.
    tree = shapely.STRtree(lines)
    a_idx, b_idx = tree.query(lines, predicate="dwithin", distance=rules.clearance + 2 * widest - TOL)
    for i, j in zip(a_idx.tolist(), b_idx.tolist()):
        if i >= j:
            continue
        pair = (ids[i], ids[j])
        need = rules.clearance + half[i] + half[j]
        if lines[i].crosses(lines[j]):
            pt = lines[i].intersection(lines[j]).representative_point()
            out.append(Violation("crossing", pair, 0.0, need, (pt.x, pt.y), layer))
        elif wire_net.get(pair[0], -1) != wire_net.get(pair[1], -2):
            dist = lines[i].distance(lines[j])
            if dist < need - TOL:
                out.append(Violation("spacing", pair, dist, need, _closest(lines[i], lines[j]), layer))
    return out


def _closest(a, b) -> tuple[float, float]:
    p = shapely.shortest_line(a, b).interpolate(0.5, normalized=True)
    return (p.x, p.y)
