"""Traces tapered to fine-pitch pins (post-processing).

``router.reach`` routes the net of a pin too fine for its trace at a narrower
width. Here such a trace is drawn at the net's own width again, narrowing only
over its last stretch to the fine-pitch pin, where nothing wider fits. Only
where the wide part keeps its clearances from everything else (checked here
against the pads, keepouts, board edge and the other traces as drawn); else the
trace is left as it was, narrow all along.

The change of width is a wedge, narrowing evenly from the wide width to the
narrow one (a reverse teardrop), where a straight stretch of the trace has room
for it; the SES has one width to a wire, so it is written as a fan of tracks
(``io.ses.wedge_tracks``). Else the width steps."""
import math

import shapely
from shapely.geometry import LineString, Point

from weaveengine.board import Board, Rules
from weaveengine.geom.margin import edge as margin_edge

TOL = 1e-3          # mm, as ``drc``
WEDGE = 3.0        # length of a wedge, in the difference of the widths it spans (and at least a trace width)
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


def wedge_polygon(p0, p1, narrow: float, wide: float) -> list:
    """The wedge from ``p0`` (``narrow`` across) to ``p1`` (``wide`` across): four corners."""
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    d = math.hypot(dx, dy)
    nx, ny = -dy / d, dx / d
    h0, h1 = narrow / 2.0, wide / 2.0
    return [(p0[0] + nx * h0, p0[1] + ny * h0), (p1[0] + nx * h1, p1[1] + ny * h1),
            (p1[0] - nx * h1, p1[1] - ny * h1), (p0[0] - nx * h0, p0[1] - ny * h0)]


def with_wedges(teardrops: dict, wedges: dict) -> dict:
    """The teardrops and the wedges together, wire -> outlines, as filled shapes to draw."""
    return {w: list(teardrops.get(w, [])) + list(wedges.get(w, [])) for w in {*teardrops, *wedges}}


def taper(board: Board, full: Rules, lines: dict, wire_net: dict, wire_layer: dict, ends: dict,
          narrowed: dict, fine: set) -> tuple[dict[int, list], dict[int, list]]:
    """(pieces, wedges). pieces: wire -> [(points, width)] in order, for each trace
    of a narrowed net drawn at the net's width again: all along, or but for a
    narrow stretch at each end on a pad of ``fine``. wedges: wire -> the wedges
    (polygons) between a narrow piece and the wide one, if there are any (else
    the width steps). ``board``: the routed board, with the narrowed rules and
    the vias as pads. ``full``: the rules before the nets were narrowed.
    ``ends``: wire -> its two pads. ``narrowed``: net -> the width its traces
    are drawn at."""
    rules = board.rules
    pads = {p.pad_id: p for p in board.pads}
    todo = {w: net for w, net in wire_net.items()
            if net in narrowed and w in lines and w in wire_layer and len(lines[w]) >= 2
            and full.width(net) > narrowed[net] + 1e-9}
    if not todo:
        return {}, {}
    width = {w: rules.drawn(net) for w, net in wire_net.items() if w in lines}  # as drawn now, updated as traces widen
    edge = rules.outline_inset - rules.base_width / 2.0
    boundary = margin_edge(board)  # (and the Margin drawings, if held as edge)
    near = {}
    for layer in {wire_layer[w] for w in todo}:
        copper = [(p.shape, p.net_id, rules.apart(p.clearance)) for p in board.pads_on(layer)]
        copper += [(o.shape, o.net_id, rules.apart(o.clearance)) for o in board.obstacles_on(layer)]
        near[layer] = (copper, shapely.STRtree([c[0] for c in copper]) if copper else None)
    out: dict[int, list] = {}
    wedges: dict[int, list] = {}
    for w in sorted(todo):
        net, line, layer = todo[w], lines[w], wire_layer[w]
        wide, narrow = full.width(net), narrowed[net]
        a_fine, b_fine = (p in fine for p in ends[w])
        total = _length(line)
        copper, tree = near[layer]
        others = [(v, LineString(lines[v])) for v in lines if v != w and wire_layer.get(v) == layer and wire_net[v] != net]
        otree = shapely.STRtree([o[1] for o in others]) if others else None

        def fits(shape, half: float) -> bool:
            """Whether ``shape`` (a line drawn ``half`` to each side, or a polygon with 0) keeps its clearances."""
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
                for i in otree.query(shape, predicate="dwithin", distance=rules.clearance + 2 * half + max(width.values())).tolist():
                    v, other = others[i]
                    if shape.distance(other) < rules.clearance + half + width[v] / 2.0 - TOL:
                        return False
            return True

        def reach_of(pad_id: int) -> float:
            pad = pads.get(pad_id)
            return 0.0 if pad is None else max(math.dist(pad.centre, p) for p in pad.shape.convex_hull.exterior.coords)

        def straight(part: list) -> bool:
            a, b = part[0], part[-1]
            d = math.dist(a, b)
            return d > 1e-9 and all(abs((b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0])) / d < 1e-6 for p in part)

        def attempt(stretch: int, run: float):
            """The pieces and wedges for narrow stretches of ``stretch`` trace widths, each followed by a wedge ``run`` long."""
            a = reach_of(ends[w][0]) + stretch * wide if a_fine else 0.0
            b = reach_of(ends[w][1]) + stretch * wide if b_fine else 0.0
            if a + b + (a_fine + b_fine) * run >= total - wide and (a_fine or b_fine):
                return None  # no room left for a wide stretch worth the name
            drops, pieces = [], []
            ca, cb = a + run * a_fine, b + run * b_fine
            for first, pts in ((a, line), (b, line[::-1])):
                if run and first:
                    mid = _cut(_cut(pts, first)[1], run)[0]
                    if not straight(mid):
                        return None
                    poly = wedge_polygon(mid[0], mid[-1], narrow, wide)
                    if not fits(shapely.Polygon(poly), 0.0):
                        return None
                    drops.append(poly)
            body = _cut(line, ca)[1] if ca else line
            body = _cut(body, _length(body) - cb)[0] if cb else body
            if not fits(LineString(body), wide / 2.0):
                return None
            if a:
                pieces.append((_cut(line, a)[0], narrow))
            pieces.append((body, wide))
            if b:
                pieces.append((_cut(line[::-1], b)[0][::-1], narrow))
            return pieces, drops

        run = max(WEDGE * (wide - narrow), wide)
        tries = [(st, run) for st in STRETCHES] + [(st, 0.0) for st in STRETCHES] if a_fine or b_fine else [(1, 0.0)]
        for stretch, length in tries:
            got = attempt(stretch, length)
            if got is not None:
                out[w] = got[0]
                if got[1]:
                    wedges[w] = got[1]
                width[w] = wide
                break
    return out, wedges
