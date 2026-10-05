"""Geometry realisation (design section 13.1): the rubber band with thickness."""
import math

import numpy as np
from shapely.geometry import LineString

from weaveengine.board import Board
from weaveengine.realize import drc, kernel
from weaveengine.topo import sites
from weaveengine.topo.planar_map import TERMINAL
from weaveengine.topo.state import TopoState

Polyline = list[tuple[float, float]]
ARC_STEP = math.radians(6.0)  # an arc turns at most this much per segment (as in smooth.py)
ARC_OUT = 4e-4  # mm: the most the corners of a drawn arc stand out from its circle (DRC allows a micron)


def relax(state: TopoState, board: Board, cuts: dict[int, dict] | None = None,
          report: dict | None = None) -> dict[int, Polyline]:
    """Returns wire id -> polyline, from pad centre to pad centre.

    A wire with some wires between it and a vertex must stay their combined
    spacing away from that vertex: a disc round the vertex, whose radius depends
    only on the topology. Each wire is pulled taut against the discs of the
    vertices it passes (``kernel.pull``), independently of every other wire.
    Wires pulled against the same vertices then run side by side at exactly the
    spacing they owe, as parallel lines and concentric arcs.

    ``cuts`` gives a wire more discs to keep clear of than its own triangles
    tell it about (13.2): wire id -> discs, as ``realize`` collects them.

    A wire leaves its pad anywhere in the window of its pad edge. Foreign wires
    passing that pad keep their distance from the point it leaves through, so
    the wires near such points are pulled again once the points are known, and
    a last time with every such point held where it is.

    With ``report["detect_only"]`` set, nothing is returned but
    ``report["clamped"]``: the ends that press against a corner of their pad
    edge, as (wire, is its start, corner). Otherwise ``report["touched"]`` gets
    the discs each trace touches: (place among the wire's discs, x, y, radius,
    side: 1 on its left, -1 on its right, 0 for its two ends, where it comes
    on, where it goes off).
    """
    pmap = state.map
    d = board.rules.pitch
    order, vxy, edge_v, lens = state.gate_order, pmap.vxy, pmap.edge_v_list, pmap.edge_len_list
    owner, pad_net, kind = pmap.edge_owner_list, pmap.pad_net, pmap.edge_kind_list
    mids, cen, centre, exits = pmap.edge_mid_list, pmap.tri_cen, pmap.pad_centre, pmap.exit_window

    ends = {w: (owner[steps[0][0]], owner[steps[-1][0]]) for w, steps in state.wire_path.items()}
    wire_net = {w: pad_net.get(a, -1) for w, (a, _) in ends.items()}

    # Half of each wire's width beyond the base width the map was inflated for.
    half = {w: board.rules.extra(net) for w, net in wire_net.items()}

    def spacing(w1: int, w2: int) -> float:
        """Centreline distance two neighbouring wires must keep. Wires of one
        net owe each other nothing: side by side they merge into one trace."""
        if wire_net[w1] == wire_net[w2] and wire_net[w1] >= 0:
            return 0.0
        return d + half[w1] + half[w2]

    # Corners of each wire's own two pads. A wire owes its own pad no clearance,
    # so it is not held away from them (that would bend it right after the pad).
    ring = {pad: {v for e in edges for v in edge_v[e]} for pad, edges in pmap.pad_edges.items()}
    own = {w: ring[a] | ring[b] for w, (a, b) in ends.items()}

    # Vertices of active via sites (12.2): a wire of another net keeps the
    # via's keep-off from them, whatever lies in between.
    via_keep = sites.keep_at(pmap)
    hole = {v: site for site in pmap.sites.values() for v in site.verts}
    kept: dict[tuple[int, int], list[float]] = {}

    def keep(e: int, vertex: int) -> list[float]:
        """Distance each wire of gate e must keep from ``vertex``, one of the
        gate's ends: a spacing for every wire in between, on top of what the
        innermost one keeps. Listed in the order of the gate."""
        if (e, vertex) not in kept:
            row = order[e] if vertex == edge_v[e][0] else order[e][::-1]
            via, net = via_keep.get(vertex, (0.0, -1))
            total, out = 0.0 if vertex in own[row[0]] else half[row[0]], []
            for i, w in enumerate(row):
                if i:
                    total += spacing(row[i - 1], w)
                if via and wire_net[w] != net and vertex not in own[w]:
                    total = max(total, via + half[w])
                out.append(total)
            kept[e, vertex] = out if row is order[e] else out[::-1]
        return kept[e, vertex]

    def point(e: int, s: float) -> tuple[float, float]:
        (ux, uy), (wx, wy) = vxy[edge_v[e][0]], vxy[edge_v[e][1]]
        return (ux + s * (wx - ux) / lens[e], uy + s * (wy - uy) / lens[e])

    def angle(p, a, b) -> float:
        """Angle at p between a and b."""
        ax, ay, bx, by = a[0] - p[0], a[1] - p[1], b[0] - p[0], b[1] - p[1]
        return abs(math.atan2(ax * by - ay * bx, ax * bx + ay * by))

    def corner(t: int, vertex: int, round_via: bool) -> float:
        """Angle of triangle t at ``vertex``; for a via gone round as one
        disc, the angle the triangle takes up as seen from the via."""
        a, b = (q for q in pmap.tri_v_list[t] if q != vertex)
        if not round_via:
            return angle(vxy[vertex], vxy[a], vxy[b])
        site = hole[vertex]
        return 0.0 if hole.get(a) is site or hole.get(b) is site else angle(site.centre, vxy[a], vxy[b])

    def sleeve(w: int) -> list[tuple]:
        """The discs wire w passes, in order: (x, y, radius, on its left, vertex,
        gate, place on the gate, angle its route sweeps there). The first two
        and the last two are the ends of the windows on its pad edges."""
        steps = state.wire_path[w]
        out, was, at = [], (-1, -1), [None, None]
        for i, (e, t, _, _) in enumerate(steps):
            u, v = edge_v[e]
            (mx, my), (ux, uy) = mids[e], vxy[u]
            if i == 0:
                cx, cy = cen[steps[1][1]]
                hx, hy = cx - mx, cy - my      # heading into the first triangle
            else:
                cx, cy = cen[t]
                hx, hy = mx - cx, my - cy      # heading out of the triangle just crossed
                for side in (0, 1):
                    if at[side] is not None:
                        out[at[side]][7] += corner(t, was[side], out[at[side]][8])
            u_left = hx * (uy - my) - hy * (ux - mx) > 0
            if kind[e] == TERMINAL:
                a, b = exits.get(e, (0.0, lens[e]))
                out.append([*point(e, a if u_left else b), 0.0, True, -1, e, 0, 0.0])
                out.append([*point(e, b if u_left else a), 0.0, False, -1, e, 0, 0.0])
                for side in (0, 1):  # round a corner of the pad it ends on, the wire can turn on into the pad
                    if at[side] is not None and was[side] in (u, v):
                        out[at[side]][7] += angle(vxy[was[side]], vxy[u + v - was[side]], centre[ends[w][1]])
                continue
            now = (u, v) if u_left else (v, u)
            k = order[e].index(w)
            for side in (0, 1):
                vertex, r = now[side], keep(e, now[side])[k]
                if vertex == was[side]:
                    continue
                round_via = r > 0.0 and vertex in hole
                if round_via and hole.get(was[side]) is hole[vertex] and at[side] is not None and out[at[side]][8]:
                    out[at[side]][7] += corner(t, vertex, True)  # the next corner of the same via's hole
                    continue
                # A via is one round thing to go round, not the three corners of its hole.
                out.append([*(hole[vertex].centre if round_via else vxy[vertex]), r, side == 0, vertex, e, k, corner(t, vertex, round_via), round_via])
                at[side] = len(out) - 1
                if i == 1 and vertex in edge_v[steps[0][0]]:  # a corner of the pad it starts on: it can come out of the pad round it
                    a, b = edge_v[steps[0][0]]
                    out[-1][7] += angle(vxy[vertex], vxy[a + b - vertex], centre[ends[w][0]])
            was = now
        return [tuple(disc[:8]) for disc in out]

    def beside(e: int, vertex: int, k: int, other: int) -> float:
        """Distance the wire at place k of gate e keeps from where ``other``
        leaves its pad at ``vertex``: a spacing for every wire in between."""
        row = order[e][:k + 1] if vertex == edge_v[e][0] else order[e][:k - 1 if k else None:-1]
        row = row[row.index(other):] if other in row else [other] + row
        return sum(spacing(a, b) for a, b in zip(row, row[1:]))

    def guarded(w: int, discs: list[tuple], near: dict) -> list[tuple] | None:
        """``discs`` with one more for every point near them where a foreign
        wire leaves its pad, or None if there is no such point."""
        out, found = [], False
        done: tuple[set, set] = (set(), set())  # per side: the points guarded since the wire came by them
        for disc in discs:
            x, y, r, left, vertex, e, k, swept, place = disc
            before, after = [], []
            if vertex not in near:
                done[left].clear()
            for other, px, py, (cx, cy) in near.get(vertex, ()):
                if other == w or (wire_net[other] >= 0 and wire_net[other] == wire_net[w]) or (other, px, py) in done[left]:
                    continue
                done[left].add((other, px, py))
                room = beside(e, vertex, k, other)
                if math.hypot(px - x, py - y) + room <= r + 1e-9:
                    continue  # the vertex keeps the wire further away already
                # Round its pad, the point lies before the vertex or after it.
                turn = (x - cx) * (py - cy) - (y - cy) * (px - cx)
                (after if (turn > 0) == left else before).append((abs(turn), (px, py, room, left, -1, e, k, swept, place)))
            found = found or bool(before or after)
            out += [g for _, g in sorted(before, reverse=True)] + [disc] + [g for _, g in sorted(after)]
        return out if found else None

    lines: dict[int, Polyline] = {}
    touch: dict[int, list[tuple]] = {}
    where: dict[int, list[float]] = {}  # per wire: where on its two pad edges it leaves

    def pull(todo: dict[int, list[tuple]], held: bool) -> None:
        """Pulls the wires of ``todo`` taut against their discs: fills ``lines``,
        and ``where`` unless the ends are held where they are."""
        rows, places, ptr = [], [], [0]
        for w, discs in todo.items():
            src, dst = state.wire_path[w][0][0], state.wire_path[w][-1][0]
            if held:
                a, b, discs = point(src, where[w][0]), point(dst, where[w][1]), discs[2:-2]
            else:
                a, b = centre[ends[w][0]], centre[ends[w][1]]
            rows.append((*a, 0.0, False, math.pi))
            # A wire that passes a point straight has half a turn of its
            # triangles there: what they sweep beyond that is how far its trace
            # can turn round the point. Round a disc, allow a quarter turn more.
            rows += [(x, y, r if left else -r, left, max(0.0, swept - math.pi) + math.pi / 2.0) for x, y, r, left, _, _, _, swept, _ in discs]
            rows.append((*b, 0.0, False, math.pi))
            places += [-1, *(disc[8] for disc in discs), len(sleeves[w])]
            ptr.append(len(rows))
        if not rows:
            return
        table = np.array(rows, dtype=np.float64)
        x, y, r, _, most = (np.ascontiguousarray(table[:, i]) for i in range(5))
        out_ptr, out = np.zeros(len(ptr), dtype=np.int64), np.zeros(len(rows), dtype=np.int64)
        come, go = np.zeros((len(rows), 2)), np.zeros((len(rows), 2))
        args = (np.array(ptr, dtype=np.int64), x, y, r, table[:, 3] > 0, most, out_ptr, out, come, go)
        try:
            kernel.pull(*args)
        except Exception as error:  # the compiled kernel failed: never crash, fall back for good
            from weaveengine import accel
            accel.failed("relaxation", error)
            kernel.pull(*args)
        come, go, out_ptr, touched = come.tolist(), go.tolist(), out_ptr.tolist(), out.tolist()
        for i, w in enumerate(todo):
            line: Polyline = []
            for j in range(out_ptr[i], out_ptr[i + 1]):
                line += _arc(rows[touched[j]], come[j], go[j])
            touch[w] = [(places[q], rows[q][0], rows[q][1], abs(rows[q][2]), (1 if rows[q][3] else -1) if 0 <= places[q] < len(sleeves[w]) else 0, come[j], go[j])
                        for j, q in ((j, touched[j]) for j in range(out_ptr[i], out_ptr[i + 1]))]
            if held:
                lines[w] = [centre[ends[w][0]]] + line + [centre[ends[w][1]]]
                continue
            lines[w] = line
            where[w] = []
            for e, (ax, ay), (bx, by) in ((state.wire_path[w][0][0], line[0], line[1]), (state.wire_path[w][-1][0], line[-1], line[-2])):
                # Where the first (last) run crosses the pad edge.
                (ux, uy), (wx, wy) = vxy[edge_v[e][0]], vxy[edge_v[e][1]]
                dx, dy, rx, ry = (wx - ux) / lens[e], (wy - uy) / lens[e], bx - ax, by - ay
                den = dx * ry - dy * rx
                lo, hi = exits.get(e, (0.0, lens[e]))
                where[w].append((lo + hi) / 2.0 if abs(den) < 1e-12 else min(max(((ax - ux) * ry - (ay - uy) * rx) / den, lo), hi))

    def leaving() -> dict[int, list[tuple]]:
        """Pad-boundary vertices near a point where a wire leaves that pad:
        vertex -> (wire, the point, the pad's centre)."""
        near: dict[int, list[tuple]] = {}
        reach = 2.0 * d  # the corners of the pad near enough for the point to matter there
        for w, (s0, s1) in where.items():
            for e, s, pad in ((state.wire_path[w][0][0], s0, ends[w][0]), (state.wire_path[w][-1][0], s1, ends[w][1])):
                px, py = point(e, s)
                for v in ring[pad]:
                    if math.hypot(vxy[v][0] - px, vxy[v][1] - py) < reach:
                        near.setdefault(v, []).append((w, px, py, centre[pad]))
        return near

    sleeves = {}
    for w in state.wire_path:
        sleeves[w] = discs = [(*disc, i) for i, disc in enumerate(sleeve(w))]
        # Discs from the repair loop. Each goes between the two discs the trace touched
        # either side of the trouble: after the last one on its own side that the
        # trace passes before it (and never among the window ends, the first two and last two).
        for place, until, x, y, r, left, ox, oy, dx, dy in sorted((cuts or {}).get(w, {}).values(), reverse=True):
            i = min(max(place, 1), len(discs) - 3)
            far = (x - ox) * dx + (y - oy) * dy
            for j in range(i + 1, min(until, len(discs) - 2)):
                if discs[j][3] == left and (discs[j][0] - ox) * dx + (discs[j][1] - oy) * dy < far:
                    i = j
            discs.insert(i + 1, (x, y, r, left, -1, -1, 0, 1.5 * math.pi, place))
    pull(sleeves, False)
    if report is not None and report.get("detect_only"):
        report["clamped"] = _clamped(state, lines, where, ends, d)
        return {}
    near = leaving()
    again = {w: g for w, discs in sleeves.items() if (g := guarded(w, discs, near)) is not None}
    if again:
        pull(again, False)
        near = leaving()
        pull({w: guarded(w, sleeves[w], near) or sleeves[w] for w in again}, True)

    if report is not None:
        report["touched"] = touch
    result: dict[int, Polyline] = {}
    pressed: list[tuple[int, bool, tuple[float, float]]] = []
    for w, line in lines.items():
        result[w] = _simplify(line)
        # Ends pressed against a corner of their pad edge (they could not move
        # round the pad because another wire of the same pad is in the way).
        for first in (True, False):
            e = state.wire_path[w][0 if first else -1][0]
            s = where[w][0 if first else 1]
            if (s < 1e-6 or s > lens[e] - 1e-6) and len(result[w]) > 3:
                pressed.append((w, first, point(e, s)))
    if pressed:
        _merge_pressed_ends(result, pressed, pmap, wire_net, half, board)
    return result


def _arc(disc, come, go) -> Polyline:
    """The part of a trace on one disc: from where it comes on to where it goes
    off, as a polyline round the outside of the circle."""
    cx, cy, r = disc[0], disc[1], disc[2]
    if abs(r) < 1e-9:
        return [(cx, cy)]
    turn = 1.0 if r > 0 else -1.0  # a disc on the left is passed counter-clockwise
    r = abs(r)
    start = math.atan2(come[1] - cy, come[0] - cx)
    span = (turn * (math.atan2(go[1] - cy, go[0] - cx) - start)) % (2.0 * math.pi)
    if span > disc[4]:
        span = 0.0  # barely touched: a hair's turn the other way
    n = max(1, math.ceil(span / min(ARC_STEP, 2.0 * math.acos(r / (r + ARC_OUT)))))
    out = r / math.cos(span / (2 * n))
    mid = [(cx + out * math.cos(a), cy + out * math.sin(a)) for a in (start + turn * (j + 0.5) * span / n for j in range(n))]
    return [tuple(come)] + mid + [tuple(go)]


def _clamped(state: TopoState, lines, where, ends, pitch: float) -> list[tuple[int, bool, int]]:
    """Ends pressed against a corner of their pad edge: the line from the pad
    centre to where the wire is heading, once it has left the pad's
    surroundings, misses the edge."""
    pmap = state.map
    found = []
    for w, line in lines.items():
        for first in (True, False):
            e = state.wire_path[w][0 if first else -1][0]
            u, v = pmap.edge_v_list[e]
            (ux, uy), (wx, wy), L = pmap.vxy[u], pmap.vxy[v], pmap.edge_len_list[e]
            dx, dy = (wx - ux) / L, (wy - uy) / L
            s = where[w][0 if first else 1]
            px, py = ux + s * dx, uy + s * dy
            cx, cy = line[0 if first else -1]
            at_via = pmap.edge_owner_list[e] in pmap.sites
            # A via site's edge is microns long: look a trace width away at least.
            ahead = max(0.5 * L, pitch) if at_via else 0.5 * L
            far = next((q for q in (line[1:] if first else line[-2::-1]) if math.hypot(q[0] - px, q[1] - py) > ahead), None)
            if far is None:
                continue
            rx, ry = far[0] - cx, far[1] - cy
            den = dx * ry - dy * rx
            if abs(den) < 1e-12:
                continue
            s = ((cx - ux) * ry - (cy - uy) * rx) / den
            if at_via and ((ux - cx) * dy - (uy - cy) * dx) / -den < 0.0:
                # The wire heads away from this edge of the via's hole (any
                # of the three may have been the search's start): the line
                # test above would name either end. Go round the nearer way.
                s = -1.0 if (ux - cx) * rx + (uy - cy) * ry > (wx - cx) * rx + (wy - cy) * ry else 2.0 * L
            if s < -0.02 * L or s > 1.02 * L:
                found.append((w, first, u if s < 0 else v))
    return found


def _merge_pressed_ends(result, pressed, pmap, wire_net, half, board) -> None:
    """Straightens wire ends that are pressed against a corner of their pad's
    keep-off ring: the corner point is dropped, so the trace runs from the pad
    centre straight to its next vertex. That takes it over the wires of its own
    net that held it there (which is fine: one net, one piece of copper). It is
    only done where the new segment keeps its distance from every foreign trace
    and stays in free space or over its own pad."""
    import shapely
    import shapely.prepared
    ids = list(result)
    lines = [LineString(result[w]) for w in ids]
    tree = shapely.STRtree(lines)
    rules = board.rules
    vias = [p for p in board.pads_on(pmap.layer) if p.is_via]
    for w, first, corner in pressed:
        line = result[w]
        i = next((j for j, p in enumerate(line) if math.hypot(p[0] - corner[0], p[1] - corner[1]) < 1e-6), None)
        if i is None or not (1 <= i <= len(line) - 2) or (first and i != 1) or (not first and i != len(line) - 2):
            continue
        new = line[:i] + line[i + 1:]
        seg = LineString([line[i - 1], line[i + 1]])
        # The new segment must lie in free space, or over the wire's own pad.
        # (The free-space shape is large: prepare it once per map, and only test
        # the part of the segment that is outside the pad's keep-off ring.)
        free = pmap.__dict__.get("_free_prepared")
        if free is None:
            free = pmap.__dict__["_free_prepared"] = shapely.prepared.prep(pmap.free_space.free.buffer(1e-6))
        own = pmap.free_space.inflated_pads.get(_end_pad(pmap, line, first))
        outside = seg if own is None else seg.difference(own)
        if not outside.is_empty and not free.contains(outside):
            continue
        # ... and clear of the vias, which are not part of the free space.
        need = rules.clearance + rules.width(wire_net[w]) / 2.0
        ok = all(p.net_id == wire_net[w] or seg.distance(p.shape) >= need for p in vias)
        for j in tree.query(seg, predicate="dwithin", distance=rules.pitch + 2 * max(half.values(), default=0.0) + 1e-3).tolist():
            other = ids[j]
            if other != w and wire_net[other] != wire_net[w]:
                if seg.distance(lines[j]) < rules.pitch + half[w] + half[other] - 1e-6:
                    ok = False
                    break
        if ok:
            result[w] = _simplify(new)


def _end_pad(pmap, line, first: bool):
    """Pad whose centre the given end of the polyline sits on."""
    p = line[0] if first else line[-1]
    for pad, c in pmap.pad_centre.items():
        if abs(c[0] - p[0]) < 1e-9 and abs(c[1] - p[1]) < 1e-9:
            return pad
    return None


def _simplify(line: Polyline, eps: float = 1e-9) -> Polyline:
    """Drop duplicate and collinear points."""
    out: Polyline = []
    for p in line:
        if out and math.hypot(p[0] - out[-1][0], p[1] - out[-1][1]) < eps:
            continue
        while len(out) >= 2:
            (x1, y1), (x2, y2) = out[-2], out[-1]
            cross = (x2 - x1) * (p[1] - y1) - (y2 - y1) * (p[0] - x1)
            dot = (x2 - x1) * (p[0] - x2) + (y2 - y1) * (p[1] - y2)
            if abs(cross) < eps and dot > 0:
                out.pop()
            else:
                break
        out.append(p)
    return out


def polyline_length(line: Polyline) -> float:
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(line, line[1:]))


def realize(state: TopoState, board: Board, repair_rounds: int = 4):  # one layer
    """Relax, check, and where a trace comes out too close to another or to a
    via, give it the discs it did not know of and pull again (13.2).

    Returns (polylines, violations, wire_net).
    """
    pmap = state.map
    wire_net = {w: pmap.pad_net.get(pmap.edge_owner_list[steps[0][0]], -1) for w, steps in state.wire_path.items()}
    vias = [p for p in board.pads_on(pmap.layer) if p.is_via]
    cuts: dict[int, dict] = {}
    best = None
    for _ in range(repair_rounds + 1):
        report: dict = {}
        lines = relax(state, board, cuts, report)
        violations = drc.check(board, lines, wire_net, pmap.layer)
        key = (len(violations), sum(v.required - v.distance for v in violations))
        if best is None or key < best[2]:
            best = (lines, violations, key)
        touched, more = report.get("touched", {}), False
        for v in violations:
            if v.kind == "spacing":
                # Each of the two keeps clear of what the other is pulled against there.
                for a, b in (v.wires, v.wires[::-1]):
                    i, foot, along = _foot(touched[b], v.at)
                    _, mine, _ = _foot(touched[a], v.at)
                    side = along[0] * (mine[1] - foot[1]) - along[1] * (mine[0] - foot[0])  # which side of b the wire a is on
                    for _, x, y, r, on, _, _ in touched[b][i:i + 2]:
                        if on * side <= 0.0:  # only what lies on the far side of b (or is an end of b)
                            more |= _cut(cuts, a, touched[a], v.at, x, y, r + v.required)
            elif v.kind == "clearance":
                for p in vias:
                    reach = p.radius / math.cos(math.pi / 16.0) + v.required
                    if p.net_id != wire_net[v.wires[0]] and math.dist(p.centre, v.at) < reach:
                        more |= _cut(cuts, v.wires[0], touched[v.wires[0]], v.at, *p.centre, reach)
        if not more:
            break
    return best[0], best[1], wire_net


def _foot(touched, at):
    """Where a trace is nearest to ``at``: (the disc it touched last before
    that, the point on the trace, its direction there)."""
    best = None
    for i, (_, x, y, r, on, come, go) in enumerate(touched):
        if r > 0.0:  # on the arc round this disc
            dx, dy = at[0] - x, at[1] - y
            n = math.hypot(dx, dy)
            turn = (math.atan2(dy, dx) - math.atan2(come[1] - y, come[0] - x)) * on % (2.0 * math.pi)
            span = (math.atan2(go[1] - y, go[0] - x) - math.atan2(come[1] - y, come[0] - x)) * on % (2.0 * math.pi)
            if n > 0.0 and turn <= span and (best is None or abs(n - r) < best[0]):
                best = (abs(n - r), i, (x + r * dx / n, y + r * dy / n), (-on * dy, on * dx))
        if i + 1 < len(touched):  # on the straight run to the next
            (ax, ay), (bx, by) = go, touched[i + 1][5]
            dx, dy = bx - ax, by - ay
            n2 = dx * dx + dy * dy
            t = 0.0 if n2 == 0.0 else min(1.0, max(0.0, ((at[0] - ax) * dx + (at[1] - ay) * dy) / n2))
            px, py = ax + t * dx, ay + t * dy
            gap = math.hypot(at[0] - px, at[1] - py)
            if best is None or gap < best[0]:
                best = (gap, i, (px, py), (dx, dy))
    return best[1:]


def _cut(cuts, w: int, touched, at, x: float, y: float, r: float) -> bool:
    """Gives wire w a disc to keep clear of, by where its trace runs now: after
    the disc it touched last, on the side the disc's centre lies. False if it
    had that already."""
    i, foot, along = _foot(touched, at)
    left = along[0] * (y - foot[1]) - along[1] * (x - foot[0]) > 0.0
    known = cuts.setdefault(w, {})
    key = (round(x, 5), round(y, 5), left)
    if key in known and known[key][4] >= r - 1e-9:
        return False
    known[key] = (touched[i][0], touched[min(i + 1, len(touched) - 1)][0], x, y, r, left, *foot, *along)
    return True
