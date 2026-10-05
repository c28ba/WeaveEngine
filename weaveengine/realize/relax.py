"""Geometry realisation (design section 13.1): taut string with ordering."""
import math

import numpy as np
from shapely.geometry import LineString

from weaveengine.board import Board
from weaveengine.realize import drc, kernel
from weaveengine.realize.funnel import string_pull
from weaveengine.topo import sites
from weaveengine.topo.planar_map import TERMINAL
from weaveengine.topo.state import TopoState

Polyline = list[tuple[float, float]]
ARC_STEP = math.pi / 12.0
ARC_BULGE = 1.0 / math.cos(ARC_STEP / 2.0)  # how far the corners of a polygon arc stand out from its circle
STRAIGHTEN = 5e-4  # mm
MIN_SIN = 1.0 / 3.0  # caps the oblique-gate spacing at 3 pitches
MAX_SLANT = 6.0  # cap on how far a slanted gate is searched for wall clearance


def relax(state: TopoState, board: Board, tol: float = 1e-4, max_sweeps: int = 120, arcs: bool = True,
          spacing_scale: dict[int, float] | None = None, terminals: dict[int, list[float]] | None = None,
          slide: int = 3, report: dict | None = None) -> dict[int, Polyline]:
    """Returns wire id -> polyline (pad centre, gate crossings, pad centre).

    Every wire gets a window on each gate it crosses and is then pulled taut
    through its windows. A wire with r wires between it and a gate endpoint
    must stay r pitches from that endpoint, so the window on gate (u, v) is
    [rank_u * pitch, L - rank_v * pitch]. Windows depend only on the topology,
    which makes the wires independent of each other during relaxation.

    ``spacing_scale`` maps a wire id to a factor on the spacing it keeps from
    its neighbours; DRC repair (13.2) uses it to widen spacing locally.

    In the first ``slide`` passes each end of a wire also moves along its pad
    edge, pulled towards the straight line from the pad centre to the wire's
    next point; the last pass settles everything with the ends fixed
    (``terminals``). Several passes are needed because the windows that keep
    foreign wires away from a pad exit depend on where that exit is. That removes the kink between the short pad stub and the
    taut part. If ``report`` is given, ``report["clamped"]`` lists the ends
    that press against a corner of their pad edge as (wire, is its start, corner).
    """
    scale = spacing_scale or {}
    pmap = state.map
    d = board.rules.pitch
    order, vxy, edge_v, lens = state.gate_order, pmap.vxy, pmap.edge_v_list, pmap.edge_len_list
    owner, pad_net, kind = pmap.edge_owner_list, pmap.pad_net, pmap.edge_kind_list

    ends = {w: (owner[steps[0][0]], owner[steps[-1][0]]) for w, steps in state.wire_path.items()}
    wire_net = {w: pad_net.get(a, -1) for w, (a, _) in ends.items()}

    # Gate frames: origin at the u end, unit direction towards v.
    frame: dict[int, tuple[float, float, float, float, float]] = {}
    for e, row in enumerate(order):
        if row:
            (ux, uy), (wx, wy) = vxy[edge_v[e][0]], vxy[edge_v[e][1]]
            L = lens[e]
            frame[e] = (ux, uy, (wx - ux) / L, (wy - uy) / L, L)

    def point(e: int, s: float) -> tuple[float, float]:
        ux, uy, dx, dy, _ = frame[e]
        return (ux + s * dx, uy + s * dy)

    # Terminal points are fixed during a pass: wires sharing a pad edge are
    # spread along it in order, within the part a trace may leave through.
    S: dict[int, list[float]] = {}
    exits = pmap.exit_window
    for e in frame:
        if kind[e] == TERMINAL:
            k = len(order[e])
            a, b = exits.get(e, (0.0, frame[e][4]))
            S[e] = list(terminals[e]) if terminals and e in terminals else [a + (i + 0.5) / k * (b - a) for i in range(k)]

    # Half of each wire's width beyond the base width the map was inflated for.
    half = {w: board.rules.extra(net) for w, net in wire_net.items()}

    def spacing(w1: int, w2: int) -> float:
        """Centreline distance two neighbouring wires must keep. Wires of one
        net owe each other nothing: side by side they merge into one trace."""
        if wire_net[w1] == wire_net[w2] and wire_net[w1] >= 0:
            return 0.0
        gap = d + half[w1] + half[w2]
        return gap * max(scale.get(w1, 1.0), scale.get(w2, 1.0)) if scale else gap

    # Corners of each wire's own two pads. A wire owes its own pad no clearance,
    # so it is not held away from them (that would bend it right after the pad).
    ring: dict[int, set[int]] = {}
    for pad, edges in pmap.pad_edges.items():
        ring[pad] = {v for e in edges for v in edge_v[e]}
    own = {w: ring[a] | ring[b] for w, (a, b) in ends.items()}

    # Vertices of active via sites (12.2): a wire of another net keeps the
    # via's keep-off from them, whatever lies in between.
    via_keep = sites.keep_at(pmap)

    def via_need(w: int, vertex: int) -> float:
        keep, net = via_keep[vertex]
        return 0.0 if wire_net[w] == net else keep + half[w]

    def radial(e: int, vertex: int, w: int) -> float:
        """Distance wire w must keep from ``vertex``, an end of gate e: its own
        extra half-width plus a spacing for every wire in between."""
        row = order[e] if vertex == edge_v[e][0] else order[e][::-1]
        total = 0.0 if vertex in own[row[0]] else half[row[0]]
        if vertex in via_keep:
            total = max(total, via_need(row[0], vertex))
        for a, b in zip(row, row[1:]):
            if a == w:
                break
            if vertex in via_keep:
                # The wire inside is drawn round the via as a polygon whose
                # corners stand out from the circle: allow for them.
                total = max(total * ARC_BULGE + spacing(a, b), via_need(b, vertex))
            else:
                total += spacing(a, b)
        return total

    # Foreign wires must also keep a pitch away from where a wire leaves its
    # pad. Index the terminal points by the pad-boundary vertices near them.
    near_terms: dict[int, list[tuple[int, float, float]]] = {}
    reach = 2.0 * d  # generous: covers every DRC-repair spacing factor
    for te, row in S.items():
        pad_vertices = {v for pe in pmap.pad_edges[owner[te]] for v in edge_v[pe]}
        for w2, s2 in zip(order[te], row):
            px, py = point(te, s2)
            for v in pad_vertices:
                if math.hypot(vxy[v][0] - px, vxy[v][1] - py) < reach:
                    near_terms.setdefault(v, []).append((w2, px, py))

    def keep_away(w: int, e: int, vertex: int, from_u: bool) -> float:
        """Distance from ``vertex`` along gate e needed to stay one pitch away
        from foreign wires that terminate on a pad edge touching that vertex."""
        best = 0.0
        ux, uy, dx, dy, L = frame[e]
        ox, oy = (ux, uy) if from_u else (ux + L * dx, uy + L * dy)
        sx, sy = (dx, dy) if from_u else (-dx, -dy)
        for w2, px, py in near_terms[vertex]:
            if w2 == w or (wire_net[w2] >= 0 and wire_net[w2] == wire_net[w]):
                continue
            qx, qy = px - ox, py - oy
            proj = qx * sx + qy * sy
            perp2 = qx * qx + qy * qy - proj * proj
            dd = spacing(w, w2)
            # Only when the gate starts inside the keep-away disc: the wire
            # then has to leave it on the far side.
            if qx * qx + qy * qy < dd * dd:
                best = max(best, proj + math.sqrt(dd * dd - perp2))
        return best

    def wall_need(e: int, vertex: int, from_u: bool, target: float) -> float:
        """Distance from ``vertex`` along gate e at which the gate is ``target``
        away from the obstacle boundary on either side of the vertex. A gate
        that leaves a wall at a slant needs more than ``target`` along itself."""
        nbrs = pmap.v_nbr.get(vertex)
        if not nbrs or target <= 0.0:
            return target
        key = (e, from_u, round(target, 7))
        if key in wall_cache:
            return wall_cache[key]
        wall_cache[key] = found = _wall_need(e, vertex, from_u, target, nbrs)
        return found

    def _wall_need(e: int, vertex: int, from_u: bool, target: float, nbrs) -> float:
        ux, uy, dx, dy, L = frame[e]
        ox, oy = (ux, uy) if from_u else (ux + L * dx, uy + L * dy)
        sx, sy = (dx, dy) if from_u else (-dx, -dy)
        segs = [(ox, oy, *vxy[n]) for n in nbrs if n != vertex]

        def clear(s: float) -> bool:
            px, py = ox + s * sx, oy + s * sy
            return all(_dist_point_segment(px, py, *seg) >= target - 1e-9 for seg in segs)

        if clear(target):
            return target
        lo, hi = target, min(L, target * MAX_SLANT)
        if not clear(hi):
            return hi
        for _ in range(30):
            mid = (lo + hi) / 2.0
            if clear(mid):
                hi = mid
            else:
                lo = mid
        return hi

    def via_shadow(e: int, row: list[int]):
        """Limits a via opposite gate e puts on the gate's wires. In a triangle
        whose third vertex is a via, the gate's wires pass the via on one side
        or the other: those cutting the u corner must cross the gate before the
        via's keep-off disc, the others after it. Returns (lowest, highest)
        position per wire, or None if no via is opposite."""
        found = None
        ux, uy, dx, dy, L = frame[e]
        for t in pmap.edge_t_list[e]:
            if t < 0:
                continue
            verts = pmap.tri_v_list[t]
            top = next(x for x in verts if x not in edge_v[e])
            if top not in via_keep:
                continue
            if found is None:
                found = ([0.0] * len(row), [L] * len(row))
            qx, qy = vxy[top][0] - ux, vxy[top][1] - uy
            foot, height = qx * dx + qy * dy, abs(qx * dy - qy * dx)
            near_u = state.corner_cnt[t][verts.index(edge_v[e][0])]  # this many wires, from u, cut the u corner
            for k, w in enumerate(row):
                need = via_need(w, top)
                if need <= height:
                    continue
                reach = math.sqrt(need * need - height * height)
                if k < near_u:
                    found[1][k] = min(found[1][k], foot - reach)
                else:
                    found[0][k] = max(found[0][k], foot + reach)
        return found

    wall_cache = pmap.__dict__.setdefault("_wall_need_cache", {})  # geometry only: valid for the map's lifetime

    # Windows on the gates.
    window: dict[int, list[tuple[float, float]]] = {}
    for e in frame:
        if kind[e] == TERMINAL:
            continue
        row, L = order[e], frame[e][4]
        u, v = edge_v[e]
        n = len(row)
        lo, hi = [0.0] * n, [0.0] * n
        apex = via_shadow(e, row) if via_keep else None
        reach = 0.0 if u in own[row[0]] else half[row[0]]  # required distance from the obstacle at this end
        for k, w in enumerate(row):
            need = keep_away(w, e, u, True) if u in near_terms else 0.0
            if apex:
                need = max(need, apex[0][k])
            if k:
                reach += spacing(row[k - 1], w)
                need = max(need, lo[k - 1] * (ARC_BULGE if u in via_keep else 1.0) + spacing(row[k - 1], w))
            if u in via_keep:
                reach = max(reach, via_need(w, u))
            lo[k] = need if u in own[w] else max(need, wall_need(e, u, True, reach))
        reach = 0.0 if v in own[row[-1]] else half[row[-1]]
        for k in range(n - 1, -1, -1):
            w = row[k]
            need = keep_away(w, e, v, False) if v in near_terms else 0.0
            if apex:
                need = max(need, L - apex[1][k])
            if k < n - 1:
                reach += spacing(row[k + 1], w)
                need = max(need, (L - hi[k + 1]) * (ARC_BULGE if v in via_keep else 1.0) + spacing(row[k + 1], w))
            if v in via_keep:
                reach = max(reach, via_need(w, v))
            hi[k] = L - (need if v in own[w] else max(need, wall_need(e, v, False, reach)))
        win = []
        for a, b in zip(lo, hi):
            if a > b:  # over-full gate: collapse the window; DRC will report it
                a = b = min(max((a + b) / 2.0, 0.0), L)
            win.append((a, b))
        window[e] = win
        S[e] = [(a + b) / 2.0 for a, b in win]

    wires = {w: [(s[0], order[s[0]].index(w)) for s in steps] for w, steps in state.wire_path.items()}

    # Start from each wire's exact taut path through its own windows (string
    # pulling). Point-by-point sweeps alone converge far too slowly on a wire
    # that crosses many gates; from here they only have to settle the places
    # where neighbouring wires press on each other.
    mids, cen = pmap.edge_mid_list, pmap.tri_cen
    for w, pts in wires.items():
        steps = state.wire_path[w]
        portals, spans = [], []
        for i, (e, k) in enumerate(pts):
            ux, uy, dx, dy, L = frame[e]
            if e in window:
                lo, hi = window[e][k]
            elif slide:
                lo, hi = exits.get(e, (0.0, L))
            else:
                lo = hi = S[e][k]
            mx, my = mids[e]
            if i == 0:
                cx, cy = cen[steps[1][1]]
                hx, hy = cx - mx, cy - my      # heading into the first triangle
            else:
                cx, cy = cen[steps[i][1]]
                hx, hy = mx - cx, my - cy      # heading out of the triangle just crossed
            a, b = (ux + lo * dx, uy + lo * dy), (ux + hi * dx, uy + hi * dy)
            portals.append((a, b) if hx * (uy - my) - hy * (ux - mx) > 0 else (b, a))
            spans.append((lo, hi))
        corners = string_pull(pmap.pad_centre[ends[w][0]], portals, pmap.pad_centre[ends[w][1]])
        j = 0
        for i, (e, k) in enumerate(pts):
            while corners[j + 1][0] < i:
                j += 1
            (_, (ax, ay)), (ib, (bx, by)) = corners[j], corners[j + 1]
            ux, uy, dx, dy, L = frame[e]
            if ib == i:
                s = (bx - ux) * dx + (by - uy) * dy
            else:
                rx, ry = bx - ax, by - ay
                den = dx * ry - dy * rx
                s = S[e][k] if abs(den) < 1e-12 else ((ax - ux) * ry - (ay - uy) * rx) / den
            lo, hi = spans[i]
            S[e][k] = min(max(s, lo), hi)
    for e, row in S.items():  # wires were pulled independently: restore their order on each gate
        for k in range(1, len(row)):
            if row[k] < row[k - 1]:
                row[k] = row[k - 1]

    # Windows keep every wire clear of the obstacles. Neighbouring wires can
    # still be pulled against each other inside overlapping windows, so the
    # sweep also keeps each wire a pitch from its neighbours on the same gate
    # (13.1 step 2), measured perpendicular to the wires rather than along the gate.
    # Flatten to arrays for the compiled sweep: one slot per (gate, wire).
    base: dict[int, int] = {}
    total = 0
    for e, row in S.items():
        base[e] = total
        total += len(row)
    pos = np.zeros(total)
    fr = np.zeros((total, 5))
    wlo, whi = np.zeros(total), np.zeros(total)
    is_term = np.zeros(total, dtype=np.uint8)
    has_prev, has_next = np.zeros(total, dtype=np.uint8), np.zeros(total, dtype=np.uint8)
    space, share = np.zeros(total), np.zeros(total)
    for e, row in S.items():
        o, n = base[e], len(row)
        pos[o:o + n] = row
        fr[o:o + n] = frame[e]
        has_prev[o + 1:o + n] = 1
        has_next[o:o + n - 1] = 1
        if e in window:
            win = window[e]
            wlo[o:o + n] = [a for a, _ in win]
            whi[o:o + n] = [b for _, b in win]
            wires_e = order[e]
            for k in range(n - 1):
                space[o + k] = spacing(wires_e[k], wires_e[k + 1])
        else:
            is_term[o:o + n] = 1
            wlo[o:o + n], whi[o:o + n] = exits.get(e, (0.0, frame[e][4]))
            share[o:o + n] = min(d, (whi[o] - wlo[o]) / n)
    wire_ids = list(wires)
    wire_ptr = np.zeros(len(wire_ids) + 1, dtype=np.int64)
    wire_slot = np.zeros(sum(len(wires[w]) for w in wire_ids), dtype=np.int64)
    centre = np.zeros((len(wire_ids), 4))
    along_prev, along_next = np.full(total, -1, dtype=np.int64), np.full(total, -1, dtype=np.int64)
    at = 0
    for wi, w in enumerate(wire_ids):
        slots = [base[e] + k for e, k in wires[w]]
        wire_slot[at:at + len(slots)] = slots
        along_prev[slots[1:]] = slots[:-1]
        along_next[slots[:-1]] = slots[1:]
        at += len(slots)
        wire_ptr[wi + 1] = at
        centre[wi, 0:2] = pmap.pad_centre[ends[w][0]]
        centre[wi, 2:4] = pmap.pad_centre[ends[w][1]]
    if total:
        args = (pos, fr, wlo, whi, is_term, has_prev, has_next, space, share, wire_ptr, wire_slot, centre,
                along_prev, along_next, bool(slide), max_sweeps, tol, MIN_SIN)
        try:
            kernel.sweep(*args)
        except Exception as error:  # the compiled kernel failed: never crash, fall back for good
            from weaveengine import accel
            accel.failed("relaxation", error)
            for e, row in S.items():
                pos[base[e]:base[e] + len(row)] = row
            kernel.sweep(*args)
    for e, row in S.items():
        o = base[e]
        row[:] = pos[o:o + len(row)].tolist()
    clamped: dict[tuple[int, bool], int] = {}

    if slide:
        # The windows that keep foreign wires away from pad exits were built for
        # the old end positions: rebuild them and settle with the ends fixed.
        if report is not None:
            # Judge each end by where the wire is heading once it has left the
            # pad's surroundings, not by its first crossing (which may sit on
            # the very corner the end is pressed against).
            clamped.clear()
            for w, pts in wires.items():
                for first in (True, False):
                    seq = pts if first else pts[::-1]
                    e, k = seq[0]
                    ux, uy, dx, dy, L = frame[e]
                    px, py = point(e, S[e][k])
                    cx, cy = pmap.pad_centre[ends[w][0 if first else 1]]
                    far = pmap.pad_centre[ends[w][1 if first else 0]]
                    # A via site's edge is microns long: look a trace width away at least.
                    ahead = max(0.5 * L, d) if owner[e] in pmap.sites else 0.5 * L
                    for qe, qk in seq[1:]:
                        q = point(qe, S[qe][qk])
                        if math.hypot(q[0] - px, q[1] - py) > ahead:
                            far = q
                            break
                    rx, ry = far[0] - cx, far[1] - cy
                    den = dx * ry - dy * rx
                    if abs(den) > 1e-12:
                        s = ((cx - ux) * ry - (cy - uy) * rx) / den
                        if owner[e] in pmap.sites and ((ux - cx) * dy - (uy - cy) * dx) / -den < 0.0:
                            # The wire heads away from this edge of the via's hole (any
                            # of the three may have been the search's start): the line
                            # test above would name either end. Go round the nearer way.
                            s = -1.0 if (ux - cx) * rx + (uy - cy) * ry > (ux + L * dx - cx) * rx + (uy + L * dy - cy) * ry else 2.0 * L
                        if s < -0.02 * L or s > 1.02 * L:
                            clamped[(w, first)] = edge_v[e][0] if s < 0 else edge_v[e][1]
            report["clamped"] = [(w, first, v) for (w, first), v in clamped.items()]
        if report is not None and report.get("detect_only"):
            return {}
        ends_now = {e: list(row) for e, row in S.items() if kind[e] == TERMINAL}
        return relax(state, board, tol, max_sweeps, arcs, spacing_scale, ends_now, slide - 1, report)

    result: dict[int, Polyline] = {}
    pressed: list[tuple[int, bool, tuple[float, float]]] = []
    for w, pts in wires.items():
        body = [point(e, S[e][k]) for e, k in pts]
        line = [body[0]]
        steps = state.wire_path[w]
        for i in range(1, len(body)):
            if arcs:
                line.extend(_corner_arc(state, w, steps[i - 1][0], steps[i], body[i - 1], body[i], radial))
            line.append(body[i])
        src, dst = ends[w]
        line = [pmap.pad_centre[src]] + line + [pmap.pad_centre[dst]]
        result[w] = _simplify(line)
        # Ends pressed against a corner of their pad edge (they could not move
        # round the pad because another wire of the same pad is in the way).
        for first in (True, False):
            e, k = pts[0] if first else pts[-1]
            L = frame[e][4]
            if (S[e][k] < 1e-6 or S[e][k] > L - 1e-6) and len(line) > 3:
                pressed.append((w, first, point(e, S[e][k])))
    if pressed:
        _merge_pressed_ends(result, pressed, pmap, wire_net, half, board)
    return result


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
        ok = True
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


def _corner_arc(state: TopoState, w: int, prev_edge: int, step, a, b, radial) -> Polyline:
    """A wire with r wires between it and a vertex of the triangle it crosses
    must stay r pitches from that vertex. The chord between its two gate points
    can dip inside that circle; replace the dip by a circumscribed polyline arc.

    The vertex is the corner the wire cuts off, or the far end of its entry or
    exit gate; the ranks come from the gate orders.
    """
    pmap = state.map
    e, t, k, _ = step
    c = pmap.tri_v_list[t][k]

    def other(edge: int) -> int:
        u, v = pmap.edge_v_list[edge]
        return v if u == c else u

    worst, worst_depth = None, 1e-9
    for vertex, edge in ((c, e), (other(prev_edge), prev_edge), (other(e), e)):
        rho = radial(edge, vertex, w)
        if rho <= 1e-9:
            continue
        cx, cy = pmap.vxy[vertex]
        depth = rho - _dist_point_segment(cx, cy, a[0], a[1], b[0], b[1])
        if depth > worst_depth:
            worst, worst_depth = (cx, cy, rho), depth
    if worst is None:
        return []
    cx, cy, rho = worst
    ax, ay, bx, by = a[0] - cx, a[1] - cy, b[0] - cx, b[1] - cy
    da, db = math.hypot(ax, ay), math.hypot(bx, by)
    if da < 1e-9 or db < 1e-9:
        return []
    ta, tb = math.atan2(ay, ax), math.atan2(by, bx)
    delta = (tb - ta + math.pi) % (2 * math.pi) - math.pi
    sign = 1.0 if delta >= 0 else -1.0
    span = abs(delta) - math.acos(min(1.0, rho / da)) - math.acos(min(1.0, rho / db))
    if span <= 1e-9:
        return []  # the chord already clears the circle
    n = max(1, math.ceil(span / ARC_STEP))
    step_ang = span / n
    radius = rho / math.cos(step_ang / 2.0)
    start = ta + sign * math.acos(min(1.0, rho / da))
    pts = []
    for j in range(n):
        ang = start + sign * (j + 0.5) * step_ang
        pts.append((cx + radius * math.cos(ang), cy + radius * math.sin(ang)))
    tri = [pmap.vxy[v] for v in pmap.tri_v_list[t]]
    if not all(_in_triangle(p, tri) for p in pts):
        return []  # would leave the triangle; leave the chord for DRC to judge
    return pts


def _at(S, slot):
    e, k = slot
    return e, S[e][k]


def _dist_point_segment(px: float, py: float, ax: float, ay: float, bx: float, by: float) -> float:
    dx, dy = bx - ax, by - ay
    n2 = dx * dx + dy * dy
    t = 0.0 if n2 == 0.0 else min(1.0, max(0.0, ((px - ax) * dx + (py - ay) * dy) / n2))
    return math.hypot(px - ax - t * dx, py - ay - t * dy)


def _in_triangle(p, tri, eps: float = 1e-9) -> bool:
    (x1, y1), (x2, y2), (x3, y3) = tri
    d1 = (x2 - x1) * (p[1] - y1) - (y2 - y1) * (p[0] - x1)
    d2 = (x3 - x2) * (p[1] - y2) - (y3 - y2) * (p[0] - x2)
    d3 = (x1 - x3) * (p[1] - y3) - (y1 - y3) * (p[0] - x3)
    return d1 >= -eps and d2 >= -eps and d3 >= -eps


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
    if len(out) > 2:
        # Gate crossings along a straight run are collinear only to within the
        # relaxation tolerance: drop the ones within half a micron of the line.
        out = [(x, y) for x, y in LineString(out).simplify(STRAIGHTEN, preserve_topology=False).coords]
    return out


def polyline_length(line: Polyline) -> float:
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(line, line[1:]))


def realize(state: TopoState, board: Board, repair_rounds: int = 4, **kwargs):  # one layer
    """Relax, check, and widen spacing locally where DRC still complains (13.2).

    Returns (polylines, violations, wire_net).
    """
    pmap = state.map
    wire_net = {w: pmap.pad_net.get(pmap.edge_owner_list[steps[0][0]], -1) for w, steps in state.wire_path.items()}
    scale: dict[int, float] = {}
    best = None
    for _ in range(repair_rounds + 1):
        lines = relax(state, board, spacing_scale=scale, **kwargs)
        violations = drc.check(board, lines, wire_net, pmap.layer)
        key = (len(violations), sum(v.required - v.distance for v in violations))
        if best is not None and key >= best[2]:
            break  # wider spacing made it no better (gates with nothing to spare): stop
        best = (lines, violations, key)
        # Wider spacing can only mend spacing; but one violation of another
        # kind somewhere on the layer is no reason to leave those unmended.
        spaced = [v for v in violations if v.kind == "spacing"]
        if not spaced:
            break
        for v in spaced:
            factor = min(1.5, 1.05 * v.required / max(v.distance, 1e-6))
            for w in v.wires:
                scale[w] = min(2.0, scale.get(w, 1.0) * factor)
    return best[0], best[1], wire_net
