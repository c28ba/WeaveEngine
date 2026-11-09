"""Traces tapered to fine-pitch pins (post-processing).

``router.reach`` routes the net of a pin too fine for its trace at a narrower
width. Here such a trace is drawn at the net's own width again, narrowing only
over its last stretch to the fine-pitch pin, where nothing wider fits. Only
where the wide part keeps its clearances from everything else (checked here
against the pads, keepouts, board edge and the other traces as drawn); else the
trace is left as it was, narrow all along."""
import math

import shapely
from shapely.geometry import LineString, Point

from weaveengine.board import Board, Rules

TOL = 1e-3          # mm, as ``drc``
STRETCHES = (1, 2, 4, 8)   # candidate lengths of the narrow stretch beyond the pad, in trace widths


def _length(line) -> float:
    return sum(math.dist(a, b) for a, b in zip(line, line[1:]))


def _cut(line: list, dist: float) -> tuple[list, list]:
    """The polyline split at arc length ``dist`` from its start (a vertex is put there)."""
    for i, (a, b) in enumerate(zip(line, line[1:])):
        seg = math.dist(a, b)
        if dist <= seg:
            f = dist / seg if seg else 0.0
            p = (a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1]))
            return line[:i + 1] + [p], [p] + line[i + 1:]
        dist -= seg
    return list(line), [line[-1]]


def taper(board: Board, full: Rules, lines: dict, wire_net: dict, wire_layer: dict, ends: dict,
          narrowed: dict, fine: set) -> dict[int, list]:
    """wire -> its pieces [(points, width)] in order, for each trace drawn partly
    wider than the narrow stretch at the end(s) on a pad of ``fine``. ``board``:
    the routed board, with the narrowed rules and the vias as pads. ``full``:
    the rules before the nets were narrowed. ``ends``: wire -> its two pads.
    ``narrowed``: net -> the width its traces are drawn at."""
    rules = board.rules
    pads = {p.pad_id: p for p in board.pads}
    todo = {w: net for w, net in wire_net.items()
            if net in narrowed and w in lines and w in wire_layer and len(lines[w]) >= 2
            and full.width(net) > narrowed[net] + 1e-9 and any(p in fine for p in ends[w])}
    if not todo:
        return {}
    width = {w: rules.drawn(net) for w, net in wire_net.items() if w in lines}  # as drawn now, updated as traces widen
    edge = rules.outline_inset - rules.base_width / 2.0
    boundary = board.outline.boundary
    near = {}
    for layer in {wire_layer[w] for w in todo}:
        copper = [(p.shape, p.net_id, rules.apart(p.clearance)) for p in board.pads_on(layer)]
        copper += [(o.shape, o.net_id, rules.apart(o.clearance)) for o in board.obstacles_on(layer)]
        near[layer] = (copper, shapely.STRtree([c[0] for c in copper]) if copper else None)
    out: dict[int, list] = {}
    for w in sorted(todo):
        net, line, layer = todo[w], lines[w], wire_layer[w]
        wide = full.width(net)
        a_fine, b_fine = (p in fine for p in ends[w])
        total = _length(line)
        copper, tree = near[layer]
        others = [(v, LineString(lines[v])) for v in lines if v != w and wire_layer.get(v) == layer and wire_net[v] != net]
        otree = shapely.STRtree([o[1] for o in others]) if others else None

        def fits(body: list) -> bool:
            shape = LineString(body)
            half = wide / 2.0
            reach = max([c[2] for c in copper], default=0.0) + half
            if tree is not None:
                for i in tree.query(shape, predicate="dwithin", distance=reach).tolist():
                    shp, cnet, apart = copper[i]
                    if cnet >= 0 and cnet == net:
                        continue
                    if shape.distance(shp) < apart + half - TOL:
                        return False
            if not board.outline.contains(shape) or shape.distance(boundary) < edge + half - TOL:
                return False
            if otree is not None:
                for i in otree.query(shape, predicate="dwithin", distance=rules.clearance + wide + max(width.values())).tolist():
                    v, other = others[i]
                    if shape.distance(other) < rules.clearance + half + width[v] / 2.0 - TOL:
                        return False
            return True

        def reach_of(pad_id: int) -> float:
            pad = pads.get(pad_id)
            return 0.0 if pad is None else max(math.dist(pad.centre, p) for p in pad.shape.convex_hull.exterior.coords)

        for stretch in STRETCHES:
            a = reach_of(ends[w][0]) + stretch * wide if a_fine else 0.0
            b = reach_of(ends[w][1]) + stretch * wide if b_fine else 0.0
            if a + b >= total - wide:
                break  # no room left for a wide stretch worth the name
            body = _cut(line, a)[1] if a else line
            body = _cut(body, _length(body) - b)[0] if b else body
            if fits(body):
                pieces = []
                if a:
                    pieces.append((_cut(line, a)[0], narrowed[net]))
                pieces.append((body, wide))
                if b:
                    pieces.append((_cut(line[::-1], b)[0][::-1], narrowed[net]))
                out[w] = pieces
                width[w] = wide
                break
    return out
