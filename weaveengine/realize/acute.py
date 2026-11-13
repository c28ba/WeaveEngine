"""Acute angles where copper of one net meets (design 13.3b).

The copper of a net on a layer is its traces at their drawn width, its pads
(vias among them) and its teardrops, as one shape. Where two of these meet,
the outline turns into the copper; the empty wedge outside that corner is
where etchant is trapped when the board is made (an acid trap), and it looks
wrong. Every such corner whose wedge is under the least angle is a meeting,
measured a few microns along the outline either side (so the facets of a
drawn arc, or a notch of no depth, do not count), and named by what meets
there:

- ``pad``: a trace and its pad, or two traces at a pad;
- ``via``: a trace and its via, or traces at a via;
- ``tee``: traces of the net away from any pad or via (a branch off a trunk
  the net's wires share, or two crossing);
- ``fanout``: any of these where a fan-out via or trace (``plan.fanout``)
  is one of the two;
- ``bend``: a trace's own corner (one trace on both sides).

Those are acid traps: copper of one layer. A via also looks sharp when the
traces it joins on different layers turn back on each other (13.3a's
measure): ``through`` counts the vias where even the straightest pair of
them, read ``ALONG`` from the via, meets under the least angle. No etchant
is trapped there; it is how the board looks.
"""
import math
from dataclasses import dataclass

import shapely
import shapely.ops
from shapely.geometry import LineString, Point, Polygon
from shapely.geometry.polygon import orient

from weaveengine.board import Board

LEAST = 90.0        # degrees: the least angle copper of one net may meet at (``Options.least_angle``)
REACH = 0.02        # mm along the outline either side of a corner at which its angle is read
ROUND = 32          # segments per quarter circle of the traces' ends and round pads, as measured
ALONG = 0.5         # mm along each trace from a via at which its way is read (``through``)
TOL = 0.5           # degrees: a meeting at the least angle less this is none (a branch square to its trunk meets at 90)
KINDS = ("pad", "via", "tee", "fanout", "bend", "through")


@dataclass(frozen=True)
class Meeting:
    kind: str
    layer: int
    net: int
    at: tuple[float, float]
    angle: float                     # degrees: the empty wedge's
    wires: tuple[int, ...]           # the traces that meet there (one for a pad, two or more otherwise)
    pad: int | None = None           # the pad or via, if it is one of the two, or the one the traces end on there
    drop: bool = False               # a teardrop (or a taper's wedge) is one of the two


def _walk(ring: list, i: int, step: int, reach: float):
    """The point ``reach`` along the closed ring (without its repeated last
    point) from its vertex i, forwards (step 1) or back (-1)."""
    n = len(ring)
    here, left = ring[i], reach
    for k in range(1, n):
        nxt = ring[(i + step * k) % n]
        d = math.dist(here, nxt)
        if d >= left:
            f = left / d
            return (here[0] + f * (nxt[0] - here[0]), here[1] + f * (nxt[1] - here[1]))
        left -= d
        here = nxt
    return here


def corners(shape, least: float = LEAST, reach: float = REACH) -> list[tuple[tuple[float, float], float, tuple, tuple]]:
    """The corners of ``shape``'s outline that turn into it with an empty wedge
    under ``least`` degrees: (corner, angle, the point ``reach`` back along
    the outline, the point as far on)."""
    out = []
    polys = getattr(shape, "geoms", [shape])
    for poly in polys:
        if poly.is_empty or poly.geom_type != "Polygon":
            continue
        poly = orient(poly, 1.0)  # (copper on the left of every ring)
        for ring in (poly.exterior, *poly.interiors):
            pts = list(ring.coords)[:-1]
            n = len(pts)
            for i in range(n):
                a, b, c = pts[i - 1], pts[i], pts[(i + 1) % n]
                if (b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0]) >= 0.0:
                    continue  # (turns left: a corner of the copper, not into it)
                p, q = _walk(pts, i, -1, reach), _walk(pts, i, 1, reach)
                ux, uy, vx, vy = p[0] - b[0], p[1] - b[1], q[0] - b[0], q[1] - b[1]
                angle = math.degrees(math.atan2(abs(ux * vy - uy * vx), ux * vx + uy * vy))
                if ux * vy - uy * vx < 0.0:  # (the wedge between them is the empty side only if it opens to the right)
                    angle = 360.0 - angle
                if angle < least - TOL:
                    out.append((b, angle, p, q))
    return out


def meetings(board: Board, lines: dict, wire_net: dict, wire_layer: dict, ends: dict, drawn: dict | None = None,
             widths: dict | None = None, least: float = LEAST) -> list[Meeting]:
    """Every place copper of one net meets itself under ``least`` degrees,
    on the routed ``board`` (with the vias as pads). ``lines``: the traces as
    drawn; ``ends``: wire -> its two pads (a trace without is a fan-out
    trace); ``drawn``: wire -> the filled shapes drawn with it (teardrops,
    wedges: ``taper.with_wedges``); ``widths``: wire -> [(points, width)] for
    a trace drawn in pieces (``realize.taper``)."""
    out: list[Meeting] = []
    for layer in sorted(set(wire_layer.values())):
        by_net: dict[int, list[int]] = {}
        for w, li in wire_layer.items():
            if li == layer and w in lines and len(lines[w]) > 1:
                by_net.setdefault(wire_net[w], []).append(w)
        pads_here = board.pads_on(layer)
        for net, wires in sorted(by_net.items()):
            out += net_meetings(board, layer, net, {w: lines[w] for w in wires}, ends, pads_here, drawn, widths, least)
    out += through(board, lines, wire_layer, ends, least)
    return out


def net_meetings(board: Board, layer: int, net: int, lines: dict, ends: dict, pads_here: list, drawn: dict | None = None,
                 widths: dict | None = None, least: float = LEAST) -> list[Meeting]:
    """``meetings`` of one net on one layer: ``lines``, its traces there;
    ``pads_here``, the pads on the layer."""
    rules, drawn, widths = board.rules, drawn or {}, widths or {}
    pieces: list[tuple[object, str, int]] = []  # (shape, "trace", "drop" or "pad", id)
    for w, line in lines.items():
        for points, width in widths.get(w, [(line, rules.drawn(net))]):
            if len(points) > 1:
                pieces.append((LineString(points).buffer(width / 2.0, quad_segs=ROUND), "trace", w))
        for poly in drawn.get(w, []):
            pieces.append((Polygon(poly[:5]).buffer(0), "drop", w))
    pads = {}
    for p in pads_here:
        if p.net_id == net:
            shape = Point(p.centre).buffer(p.radius, quad_segs=ROUND) if p.radius is not None else p.shape
            pieces.append((shape, "pad", p.pad_id))
            pads[p.pad_id] = p
    if not pieces:
        return []
    found = corners(shapely.union_all([s for s, _, _ in pieces]), least)
    if not found:
        return []
    tree = shapely.STRtree([s.boundary for s, _, _ in pieces])
    out: list[Meeting] = []
    for at, angle, p, q in found:
        if any(math.dist(at, m.at) < 2.0 * REACH for m in out):
            continue  # (one corner drawn as two vertices)
        sides = []
        for point in (p, q):
            hit = tree.query_nearest(Point(point))
            sides.append(pieces[int(hit[0])][1:] if len(hit) else ("trace", -1))
        near = tree.query(Point(at).buffer(REACH), predicate="intersects")
        touching = {pieces[int(i)][1:] for i in near} | set(sides)
        traces = tuple(sorted({i for k, i in touching if k != "pad"}))
        if not traces:
            continue  # (pads of the net by themselves: the board's own shapes)
        pad_ids = [i for k, i in sides if k == "pad"]
        if not pad_ids:  # (two traces: at a pad if both end on one that is near)
            common = set.intersection(*[set(ends.get(w, ())) for w in traces]) if traces else set()
            pad_ids = [i for i in common if i in pads and pads[i].shape.distance(Point(at)) < rules.drawn(net) + REACH]
        pad = pad_ids[0] if pad_ids else None
        if any(w not in ends for w in traces) or (pad is not None and pads[pad].fanout):
            kind = "fanout"
        elif pad is not None:
            kind = "via" if pads[pad].is_via else "pad"
        elif len(traces) == 1:
            kind = "bend"
        else:
            kind = "tee"
        out.append(Meeting(kind, layer, net, (at[0], at[1]), angle, traces, pad, any(k == "drop" for k, _ in sides)))
    return out


def _towards(line: list, start: bool, along: float):
    """The point ``along`` the polyline from its first point (``start``) or its last."""
    pts = line if start else line[::-1]
    left = along
    for a, b in zip(pts, pts[1:]):
        d = math.dist(a, b)
        if d >= left:
            f = left / d
            return (a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1]))
        left -= d
    return pts[-1]


def through(board: Board, lines: dict, wire_layer: dict, ends: dict, least: float = LEAST) -> list[Meeting]:
    """The vias whose traces on different layers meet under ``least``
    degrees, even the straightest pair of them (read ``ALONG`` from the via)."""
    vias = {p.pad_id: p for p in board.pads if p.is_via}
    at: dict[int, list[tuple[int, int, float]]] = {}  # via -> (wire, layer, the way it leaves)
    for w, (a, b) in ends.items():
        if w not in lines or w not in wire_layer or len(lines[w]) < 2:
            continue
        for pad, start in ((a, True), (b, False)):
            if pad in vias:
                c = vias[pad].centre
                x, y = _towards(lines[w], start, ALONG)
                at.setdefault(pad, []).append((w, wire_layer[w], math.atan2(y - c[1], x - c[0])))
    out = []
    for pad, legs in sorted(at.items()):
        best, pair = -1.0, ()
        for i, (w, li, u) in enumerate(legs):
            for x, lj, v in legs[i + 1:]:
                if li != lj:
                    angle = math.degrees(abs((u - v + math.pi) % (2.0 * math.pi) - math.pi))
                    if angle > best:
                        best, pair = angle, (w, x)
        if 0.0 <= best < least - TOL:
            via = vias[pad]
            out.append(Meeting("through", -1, via.net_id, via.centre, best, tuple(sorted(pair)), pad))
    return out


# -- blunting: traces redrawn so that fewer meetings of these kinds are left ----------------------------------------

STEP = 0.01          # mm between the points tried along a trace for the end of its new stretch
LONGEST = 5.0        # mm along a trace from where it first leaves the other copper to where it may be met again
SLACK = 1e-4         # mm kept beyond each clearance, as ``smooth`` (on it, as the routing's own traces stand, a
                     # redrawn trace left an island of pour on sweep-half-swept that the estimate does not see)
BLUNTED = ("pad", "via", "tee")


def _oriented(line: list, start) -> list:
    """The polyline running from the end nearer ``start``."""
    return line if math.dist(line[0], start) <= math.dist(line[-1], start) else line[::-1]


def _out_to(line: list, near) -> tuple[tuple[float, float], float] | None:
    """The point along ``line`` (and how far along it is), of those every
    ``STEP``, just past the last at which ``near(point)`` that is within
    ``LONGEST`` of where it first is not; None if it never is not."""
    path = LineString(line)
    points = [path.interpolate(k * STEP) for k in range(1, int(path.length / STEP) + 1)]
    flags = [near(p) for p in points]
    first = next((k for k, f in enumerate(flags) if not f), None)
    if first is None:
        return None
    reach = first + int(LONGEST / STEP)
    last = max((k for k, f in enumerate(flags[:reach]) if f), default=-1)
    if last + 1 >= len(points):
        return None
    p = points[last + 1]
    return (p.x, p.y), (last + 2) * STEP


def _rest(line: list, d: float) -> list:
    """The polyline from ``d`` along it to its end, without its first point."""
    out, run = [], 0.0
    for a, b in zip(line, line[1:]):
        run += math.dist(a, b)
        if run > d + 1e-9:
            out.append(b)
    return out


def _into_pad(line: list, shape, gap: float, half: float) -> list[list]:
    """``line`` (from its pad's anchor) with its first stretch replaced, in
    each way there is, the shortest first: out of the pad (``shape``: with the
    pads of its net it overlaps) square to one of its edges, ``gap`` out, and
    on to the point where the trace is ``gap`` off the pad for the last time
    near it. (An edge too short for the trace's width is no way out.)"""
    hit = _out_to(line, lambda p: shape.contains(p) or shape.boundary.distance(p) < gap)
    if hit is None:
        return []
    y, d = hit
    rest = _rest(line, d)
    out = []
    polys = getattr(shape, "geoms", [shape])
    for poly in polys:
        ring = list(orient(poly, 1.0).exterior.coords)
        for a, b in zip(ring, ring[1:]):
            length = math.dist(a, b)
            if length < 2.0 * (half + SLACK):
                continue
            tx, ty = (b[0] - a[0]) / length, (b[1] - a[1]) / length
            nx, ny = ty, -tx  # (outwards: the copper is on the left)
            along = (y[0] - a[0]) * tx + (y[1] - a[1]) * ty
            along = min(max(along, half + SLACK), length - half - SLACK)
            x = (a[0] + tx * along, a[1] + ty * along)
            inner = (x[0] - nx * (half + SLACK), x[1] - ny * (half + SLACK))
            out_at = (x[0] + nx * gap, x[1] + ny * gap)
            if not shape.contains(Point(inner)) or shape.contains(Point(out_at)):
                continue
            ux, uy = y[0] - out_at[0], y[1] - out_at[1]
            head = [line[0], inner] if math.dist(line[0], inner) > 1e-6 else [line[0]]
            new = head + [out_at] + ([y] if math.hypot(ux, uy) > 1e-6 else []) + rest
            out.append((LineString(new).length, new))
    return [new for _, new in sorted(out)]


def _off_trace(line: list, trunk: list, gap: float) -> list | None:
    """``line`` (from the end it shares with ``trunk``) along ``trunk``, and
    then square off it to the point of its own where it is ``gap`` from it
    for the last time near it."""
    path = LineString(trunk)
    hit = _out_to(line, lambda p: path.distance(p) < gap)
    if hit is None:
        return None
    y, d = hit
    along = path.project(Point(y))
    if along <= 1e-6 or along >= path.length - 1e-6:
        return None  # (square off the trunk's end, or before it begins: no branch)
    head = list(shapely.ops.substring(path, 0.0, along).coords)
    return head + [y] + _rest(line, d)


def _pad_at(pad, pads_here: list):
    """(the pad's shape with those of its net on the layer that overlap it, their ids)."""
    near = [p for p in pads_here if p.net_id == pad.net_id and p.shape.intersects(pad.shape)]
    return shapely.union_all([p.shape for p in near]), {p.pad_id for p in near}


def _tries(m: Meeting, lines: dict, ends: dict, pads: dict, pads_here: list, rules) -> list[tuple[int, list]]:
    """(wire, new line) to try for meeting ``m``, the one that adds least to its trace first."""
    out = []
    half = rules.width(m.net) / 2.0
    for w in m.wires:
        for end in ends.get(w, ()):
            if m.pad is None or end not in pads or pads[end].is_via:
                continue
            shape, ids = _pad_at(pads[end], pads_here)
            if m.pad in ids:
                out += [(w, new) for new in _into_pad(_oriented(lines[w], pads[end].centre), shape, rules.clearance + half, half)]
    for w in m.wires:
        for x in m.wires:
            shared = set(ends.get(w, ())) & set(ends.get(x, ())) & set(pads) if x != w else set()
            for pad in shared:
                new = _off_trace(_oriented(lines[w], pads[pad].centre), _oriented(lines[x], pads[pad].centre),
                                 rules.clearance + 2.0 * half)
                if new is not None:
                    out.append((w, new))
    out = [(w, new if math.dist(new[0], lines[w][0]) <= math.dist(new[-1], lines[w][0]) else new[::-1]) for w, new in out]  # (as it ran)
    return sorted(out, key=lambda t: LineString(t[1]).length - LineString(lines[t[0]]).length)  # (what adds least first)


class _Room:
    """What a redrawn trace of one layer must keep clear of: foreign copper,
    keepouts, the board edge and foreign traces (as ``smooth.fits``); and
    nothing foreign may lie between it and the trace it replaces."""

    def __init__(self, board: Board, layer: int, lines: dict, wire_net: dict):
        rules = board.rules
        self.rules, self.wire_net, self.lines = rules, wire_net, lines
        self.copper = [(p.shape, p.net_id, rules.apart(p.clearance)) for p in board.pads_on(layer)]
        self.copper += [(o.shape, o.net_id, rules.apart(o.clearance)) for o in board.obstacles_on(layer)]
        self.copper_tree = shapely.STRtree([c[0] for c in self.copper]) if self.copper else None
        self.farthest = max((c[2] for c in self.copper), default=rules.clearance)
        self.edge = rules.outline_inset - rules.base_width / 2.0
        self.boundary, self.outline = board.outline.boundary, board.outline
        self.widest = max([rules.trace_width, *rules.net_width.values()])
        self.update()

    def update(self) -> None:
        self.ids = list(self.lines)
        self.shapes = [LineString(self.lines[w]) for w in self.ids]
        self.tree = shapely.STRtree(self.shapes)

    def fits(self, w: int, new: list) -> bool:
        rules, net = self.rules, self.wire_net[w]
        half = rules.width(net) / 2.0
        was = _oriented(self.lines[w], new[0])
        line = LineString(new).difference(LineString(was).buffer(1e-6))  # (what is new of it)
        if line.is_empty:
            return True
        swept = shapely.make_valid(Polygon(was + new[::-1]))  # (between the trace as it was and as it would be)
        if line.distance(self.boundary) < self.edge + half + SLACK or not self.outline.contains(line):
            return False
        if self.copper_tree is not None:
            for i in self.copper_tree.query(line, predicate="dwithin", distance=self.farthest + half + SLACK).tolist():
                shape, other, apart = self.copper[i]
                if (other != net or other < 0) and line.distance(shape) < apart + half + SLACK:
                    return False
            for i in self.copper_tree.query(swept, predicate="intersects").tolist():
                if self.copper[i][1] != net or net < 0:
                    return False
        for j in self.tree.query(line, predicate="dwithin", distance=rules.clearance + half + self.widest / 2.0 + SLACK).tolist():
            other = self.ids[j]
            if self.wire_net[other] != net and \
                    line.distance(self.shapes[j]) < rules.clearance + half + rules.width(self.wire_net[other]) / 2.0 + SLACK:
                return False
        for j in self.tree.query(swept, predicate="intersects").tolist():
            if self.wire_net[self.ids[j]] != net:
                return False
        return True


def blunt(board: Board, lines: dict, wire_net: dict, wire_layer: dict, ends: dict, least: float = LEAST,
          drops: tuple | None = None) -> dict:
    """The traces to redraw so that their net meets itself under ``least``
    degrees fewer times (design 13.3b), as wire -> new line. A trace into its
    pad at a slant leaves the pad square to its edge and bends to its old way
    where it is a clearance off the pad. A trace that leaves a pad, a via or a
    trunk it shares with another of its net at a slant runs along the other
    to where it would be a clearance off it, and squares off to its old way
    there: its branch point moves out along the trunk. A change is kept only
    if its net then meets itself under ``least`` fewer times on that layer,
    and the new stretch keeps every clearance and passes nothing foreign on
    the other side. ``drops``: the teardrops' limits (``teardrop.teardrops``'s
    max_length, max_width, breathing), if they are to be made: the meetings
    are counted with the teardrops the traces would get (two teardrops of a
    via can leave a notch where their traces do not), which are made
    afterwards, round the traces as they are left."""
    from weaveengine.realize.teardrop import teardrops
    out: dict[int, list] = {}
    pads = {p.pad_id: p for p in board.pads}
    for layer in sorted(set(wire_layer.values())):
        here = {w: lines[w] for w, li in wire_layer.items() if li == layer and w in lines and len(lines[w]) > 1}
        if not here:
            continue
        pads_here = board.pads_on(layer)
        room = None

        def drops_of(mine: dict, wires) -> dict:
            """The teardrops ``wires`` would get. (A trace's teardrops keep clear of other nets' traces only, so
            redrawing one trace of a net changes no other's.)"""
            if not drops:
                return {}
            got = teardrops(board, {**here, **mine}, wire_net, ends, layer, *drops, only=set(wires))
            return {w: got.get(w, []) for w in wires}

        for net in sorted({wire_net[w] for w in here}):
            mine = {w: line for w, line in here.items() if wire_net[w] == net}
            drawn = drops_of(mine, mine)
            now = net_meetings(board, layer, net, mine, ends, pads_here, drawn, least=least)
            tried: set = set()
            while True:
                left = [m for m in now if m.kind in BLUNTED and (m.at, m.wires) not in tried]
                if not left:
                    break
                m = min(left, key=lambda m: m.angle)
                tried.add((m.at, m.wires))
                for w, new in _tries(m, mine, ends, pads, pads_here, board.rules):
                    if room is None:
                        room = _Room(board, layer, here, wire_net)
                    if not room.fits(w, new):
                        continue
                    trial = {**mine, w: new}
                    with_new = {**drawn, **drops_of(trial, (w,))}
                    again = net_meetings(board, layer, net, trial, ends, pads_here, with_new, least=least)
                    if len(again) >= len(now):
                        continue
                    mine[w] = here[w] = out[w] = new
                    drawn = with_new
                    room.update()
                    now = again
                    break
    return out


def count(found: list[Meeting]) -> dict[str, int]:
    """Meetings by kind (every kind, so that a count of none shows)."""
    return {kind: sum(1 for m in found if m.kind == kind) for kind in KINDS}
