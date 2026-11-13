"""Drawings on KiCad's Margin layer held as board edge (``Board.margin_edge``).

Two ways (``MODES``): ``edge``, as the outline, by everything; and ``added``,
by the copper drawn once the routing is done (teardrops, rounded corners,
tapers, blunted meetings, arcs, the vias and traces to the planes; and trace
ends are ``trim``-med), while the traces and their vias may cross the
drawings. Boards draw there shield-can outlines their own routing crosses
(Solo's: 184 tracks and 15 vias within the edge clearance of them in the
designers' routing), and held as edge by the routing those cut the board
into pieces: Solo A routes 104 of 155 connections for 132, and with only the
vias held 129. ``added`` routes as ``off`` does.

KiCad's check holds copper to the edge clearance from every drawing on the
Margin layer, as from the outline. Boards draw there nearer their own pads
than that (the outline of a shield can through a row of pads, say), so a pad
can sit within the edge clearance of a drawing itself: KiCad flags the pad,
and every track that touches it. Held as edge all round, such a pad could not
be reached; with room left round it, traces run along the drawing there.

So each pad near a drawing keeps one way out, the shadow it casts straight
away from the drawing (from the nearest point of it through the pad's
anchor; both ways across a drawing through the anchor). A trace leaves the
pad there and nowhere else, and is within the edge clearance only for as
long as it must be: one track to the pad, not a run along the drawing. A
trace whose end can stop inside the pad clear of the drawing does
(``trim``), so a pad that only reaches into the edge clearance is joined with
no error at all.

Everything is in the trace centreline's terms (as the map, design section 5):
``keep_out`` is where a base-width trace's centreline may not be.
"""
import math

import shapely
from shapely import affinity
from shapely.geometry import LineString, Point

QUAD = 16      # segments per quarter circle of the buffers here (their sag is under a micron at these sizes)
SAG = 1e-3     # mm added to every buffer, so that the polygon holds the true round offset


MODES = ("edge", "added", "off")


def lines(board):
    """The Margin drawings as one geometry, if anything is held clear of them (``MODES``), else None."""
    if board.margin_edge == "off" or not board.margin:
        return None
    cached = board.__dict__.get("_margin_lines")
    if cached is None:
        cached = board.__dict__["_margin_lines"] = shapely.union_all(board.margin)
    return cached


def edge(board):
    """The board edge, its cutouts and (if held as edge) the Margin drawings: what copper keeps the edge clearance from."""
    drawn = lines(board)
    return board.outline.boundary if drawn is None else shapely.union(board.outline.boundary, drawn)


def edge_clearance(rules) -> float:
    return rules.outline_inset - rules.base_width / 2.0


def held(board):
    """The Margin drawings, if the traces are held clear of them too (``edge``), else None."""
    return lines(board) if board.margin_edge == "edge" else None


def ways_out(board, layer: int):
    """The pads' ways out on ``layer`` (one geometry, or None): each pad that
    comes within the edge clearance and a trace width of a drawing, swept
    straight away from the nearest point of it until it is that far off."""
    drawn = held(board)
    if drawn is None:
        return None
    cache = board.__dict__.setdefault("_margin_ways", {})
    if layer in cache:
        return cache[layer]
    rules = board.rules
    reach = edge_clearance(rules) + rules.base_width
    parts = shapely.get_parts(drawn)
    tree = shapely.STRtree(parts)
    band = drawn.buffer(rules.outline_inset + SAG, quad_segs=QUAD)
    # The other copper, inflated as the map inflates it (``geom.inflate``): what a way out may be blocked by.
    pads = board.pads_on(layer)
    copper = [(p.pad_id, p.shape.buffer(rules.inflation + rules.apart(p.clearance) - rules.clearance)) for p in pads]
    copper += [(-1, o.shape.buffer(rules.inflation + rules.apart(o.clearance) - rules.clearance)) for o in board.obstacles_on(layer)]
    others = shapely.STRtree([c[1] for c in copper])
    shadows = []
    for pad in pads:
        if pad.shape.distance(drawn) >= reach:
            continue
        c = Point(pad.centre)
        near = parts[tree.nearest(c)]
        q = near.interpolate(near.project(c))
        dx, dy = c.x - q.x, c.y - q.y
        d = math.hypot(dx, dy)
        if d > 1e-6:
            ways = [(dx / d, dy / d)]
        else:  # (a drawing through the anchor: both ways across it)
            s = near.project(c)
            a, b = near.interpolate(max(s - 0.01, 0.0)), near.interpolate(min(s + 0.01, near.length))
            tx, ty = b.x - a.x, b.y - a.y
            t = math.hypot(tx, ty) or 1.0
            ways = [(-ty / t, tx / t), (ty / t, -tx / t)]
        x0, y0, x1, y1 = pad.shape.bounds
        far = reach + math.hypot(x1 - x0, y1 - y0)
        mine = [shapely.convex_hull(shapely.union(pad.shape, affinity.translate(pad.shape, ux * far, uy * far))) for ux, uy in ways]
        if not any(_leads_out(pad, way, copper, others, band) for way in mine):
            # Its way out is shut by other copper (a pin of a part whose body lies away from the drawing): it
            # is left any way, with room round it, and its traces may run within the edge clearance.
            mine = [pad.shape.buffer(reach, quad_segs=QUAD)]
        shadows += mine
    cache[layer] = shapely.union_all(shadows) if shadows else None
    return cache[layer]


def _leads_out(pad, way, copper, others, band) -> bool:
    """Whether a trace can leave ``pad`` along ``way`` until it is clear of
    ``band``: the room the other copper leaves on it reaches from the pad's
    keep-off ring to beyond the band."""
    blocked = [copper[i][1] for i in others.query(way, predicate="intersects").tolist()]  # (its own: the trace starts on its edge)
    room = way.difference(shapely.union_all(blocked))
    ring = next(shape for pad_id, shape in copper if pad_id == pad.pad_id).buffer(2.0 * SAG)
    return any(part.intersects(ring) and not part.difference(band).is_empty for part in shapely.get_parts(room))


def keep_out(board, layer: int):
    """Where a base-width trace's centreline may not be on ``layer`` for the
    Margin drawings (None if they are not held as edge): within the edge
    clearance and half the width of one, except on the pads' ways out."""
    drawn = held(board)
    if drawn is None:
        return None
    band = drawn.buffer(board.rules.outline_inset + SAG, quad_segs=QUAD)
    ways = ways_out(board, layer)
    return band if ways is None else band.difference(ways)


def too_near(board, layer: int, line, half: float, slack: float) -> float | None:
    """How near the trace ``line`` (``half`` its width) comes to a Margin
    drawing off the pads' ways out, if that is under the edge clearance less
    ``slack``; else None."""
    drawn = held(board)
    if drawn is None:
        return None
    need = edge_clearance(board.rules) + half
    if line.distance(drawn) >= need - slack:
        return None
    ways = ways_out(board, layer)
    rest = line if ways is None else line.difference(ways)
    if rest.is_empty:
        return None
    dist = rest.distance(drawn)
    return dist if dist < need - slack else None


def trim_ends(board, polylines: dict, wire_net: dict, wire_layer: dict, ends: dict) -> tuple[int, int]:
    """Each trace's ends on a pad within the edge clearance of a Margin drawing
    ``trim``-med, in ``polylines`` (wire -> points; ``ends``: wire -> its two pads).
    Returns how many ends start within it, and how many of them were trimmed clear."""
    drawn = lines(board)
    if drawn is None:
        return 0, 0
    pads = {p.pad_id: p for p in board.pads}
    near = count = 0
    for w, (a, b) in ends.items():
        line = polylines.get(w)
        if line is None or w not in wire_layer:
            continue
        half = board.rules.drawn(wire_net[w]) / 2.0
        for pad_id, flip in ((a, False), (b, True)):
            pad = pads.get(pad_id)
            pts = list(line[::-1]) if flip else list(line)
            if pad is None or math.dist(pts[0], pad.centre) > 1e-6:
                continue  # (not a trace from this pad's anchor)
            if Point(pts[0]).distance(drawn) >= edge_clearance(board.rules) + half:
                continue
            near += 1
            cut = trim(board, wire_layer[w], pad, pts, half)
            if cut is not pts:
                line = cut[::-1] if flip else cut
                count += 1
        polylines[w] = line
    return near, count


def trim(board, layer: int, pad, line: list, half: float) -> list:
    """``line`` (a trace from the anchor of ``pad``) ending inside the pad
    where it is first clear of the Margin drawings, if the pad reaches that
    far; else as it is. KiCad joins a track to a pad its end is inside."""
    drawn = lines(board)
    if drawn is None or len(line) < 2:
        return line
    need = edge_clearance(board.rules) + half
    if Point(line[0]).distance(drawn) >= need:
        return line
    inside = pad.shape.buffer(-SAG)
    if inside.is_empty:
        return line
    # Along the trace from the pad, the first point whose disc is clear.
    band = drawn.buffer(need + SAG, quad_segs=QUAD)
    for k, (a, b) in enumerate(zip(line, line[1:])):
        seg = LineString([a, b])
        if seg.length == 0.0:
            continue
        clear = seg.difference(band)
        if clear.is_empty:
            continue
        p = seg.interpolate(min(seg.project(Point(g.coords[0])) for g in shapely.get_parts(clear)))
        if not inside.contains(p):
            return line  # (the pad does not reach out of the edge clearance along this trace)
        rest = list(line[k + 1:])
        return ([] if math.dist((p.x, p.y), rest[0]) < 1e-9 else [(p.x, p.y)]) + rest
    return line
