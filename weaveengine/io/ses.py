"""Specctra SES (session) writer: each wire is a polyline ``path`` with width (design section 4)."""
import math
import os

from weaveengine.io.arcs import arc_pieces, find_arcs
from weaveengine.io.dsn import UNIT_MM, Design
from weaveengine.realize.teardrop import tip_of


TEARDROP_OVERLAP = 0.75  # spacing of the fill lines at the pad, in trace widths (below 1 so they overlap)
MIN_ARC_UNITS = 4        # an arc of a smaller radius (in session units) stays as segments
SHORTEST = 0.01          # mm: a fill line shorter than this adds no copper, and KiCad calls its end unconnected


def teardrop_tracks(poly, track_width: float) -> list[tuple[float, tuple[float, float], tuple[float, float]]]:
    """A teardrop as copper a session file can carry: (width, start, end) traces.

    SES wiring is paths only, no filled shapes. The teardrop is filled with
    ordinary traces of the track's own width, fanned out from the point where
    the teardrop starts on the track to points along its edge at the pad: the
    two outer lines first, then inwards to the middle. The outer lines are set
    exactly half a trace width inside the teardrop's sides (which are tangent to
    the track's round end), so the fill stays inside the teardrop whatever the
    shape of the pad.
    """
    half = track_width / 2.0
    _, t1, left, right, t2 = poly[:5]

    def inset(a, b, towards):
        """The side a b moved half a trace width in, towards the other side."""
        nx, ny = b[1] - a[1], a[0] - b[0]
        n = math.hypot(nx, ny) or 1.0
        if nx * (towards[0] - a[0]) + ny * (towards[1] - a[1]) < 0.0:
            nx, ny = -nx, -ny
        return (a[0] + half * nx / n, a[1] + half * ny / n), (b[0] + half * nx / n, b[1] + half * ny / n)

    # Outer lines: the teardrop's sides moved in by half a trace width. At the
    # track that brings both to the track's centre (the sides are tangent to
    # its round end); where the copper is wide there too (one piece from pad to
    # pad, ``teardrops``), the lines start side by side across it.
    (first, begin), (last, end) = inset(t1, left, t2), inset(t2, right, t1)
    span = math.dist(first, last)
    if math.dist(t1, t2) <= track_width or span <= 1e-6:
        return []
    count = max(2, math.ceil(max(span, math.dist(begin, end)) / (TEARDROP_OVERLAP * track_width)) + 1)
    ends = [(first[0] + (last[0] - first[0]) * i / (count - 1), first[1] + (last[1] - first[1]) * i / (count - 1))
            for i in range(count)]
    starts = [(begin[0] + (end[0] - begin[0]) * i / (count - 1), begin[1] + (end[1] - begin[1]) * i / (count - 1))
              for i in range(count)]
    # From the edges towards the middle.
    order, lo, hi = [], 0, count - 1
    while lo <= hi:
        order.append(lo)
        if hi != lo:
            order.append(hi)
        lo, hi = lo + 1, hi - 1
    # Each line stops half a trace width short of the teardrop's pad edge, so
    # its round end reaches that edge and no further. (Ending on the edge would
    # push the round end past a pad's corner, into the clearance of whatever is
    # beyond it.) The slivers this leaves between round ends are over the pad.
    # Where the sides slant, that edge (moved in with them) can lie outside the
    # pad: such a line is carried on until its round end is half its own
    # radius across the chord t1 t2 itself, which is on the pad (KiCad calls a
    # trace that stops short of the pad unconnected). A line that would cross
    # the chord's line beyond t1 or t2 (the teardrop of a long pad met from its
    # side) is not carried on, and if its round end does not then reach a
    # quarter of its radius over the chord, it is left out: it would end beside
    # the pad, or only graze it.
    out = []
    cx, cy = t2[0] - t1[0], t2[1] - t1[1]
    for i in order:
        (sx, sy), (ex, ey) = starts[i], ends[i]
        dx, dy = ex - sx, ey - sy
        length = math.hypot(dx, dy)
        if length <= half + SHORTEST:
            continue
        reach = length - half
        across = cx * dy - cy * dx
        if abs(across) > 1e-12:
            along = (dx * (t1[1] - sy) - dy * (t1[0] - sx)) / across  # where the line crosses the chord, from t1 (0) to t2 (1)
            if 0.0 <= along <= 1.0:
                chord = (cx * (t1[1] - sy) - cy * (t1[0] - sx)) / across * length  # along the line, to the chord
                reach = max(reach, chord - half / 2.0)
        if i in (0, count - 1) and _to_segment((ex, ey), t1, t2) <= 0.75 * half:
            # An outer line goes on nearer the teardrop's corner on the pad: stopping a
            # half width short, its round end leaves a notch beside a rounded corner.
            reach = max(reach, length - half / 2.0)
        f = reach / length
        end = (sx + dx * f, sy + dy * f)
        if _to_segment(end, t1, t2) > 0.75 * half:
            continue
        out.append((track_width, (sx, sy), end))
    # A line alone starts partway along its track, where KiCad sees no end of
    # anything to join it to, and calls it unconnected; it adds little copper.
    return out if len(out) > 1 else []


def _to_segment(p, a, b) -> float:
    """Distance from ``p`` to the segment a b."""
    vx, vy = b[0] - a[0], b[1] - a[1]
    n = vx * vx + vy * vy
    t = 0.0 if n == 0.0 else max(0.0, min(1.0, ((p[0] - a[0]) * vx + (p[1] - a[1]) * vy) / n))
    return math.hypot(p[0] - a[0] - t * vx, p[1] - a[1] - t * vy)


def write_ses(design: Design, result, path: str, teardrops: bool = True, arcs: bool = True) -> None:
    """Writes the routed traces and vias of ``result``. With ``teardrops``,
    each teardrop is added as a fan of ordinary traces (see ``teardrop_tracks``).
    With ``arcs``, the curves of a trace are written as arcs (``qarc``, in
    pieces of at most 90 degrees) instead of runs of short segments.

    The file is written beside ``path`` under another name and then renamed to
    it, so ``path`` is always a whole session: the one there before, if writing
    fails or the program is stopped part way, else the new one. (The command
    line writes the best routing so far over and over while it runs.)"""
    factor = design.resolution / UNIT_MM[design.unit.lower()]  # mm -> session units
    names = design.net_names
    board = result.board

    def n(v: float) -> str:
        return str(int(round(v * factor)))

    def q(name: str) -> str:
        # Always quoted, as KiCad and Freerouting write them: references and
        # net names may contain '-', which is the pin separator when bare.
        return '"' + name.replace('"', "") + '"'

    out = [f"(session {q(design.name)}", f"  (base_design {q(design.name)})", "  (placement",
           f"    (resolution {design.unit} {design.resolution})"]
    # No components are listed: the router never moves parts, and a session
    # that names none cannot leave the CAD tool with a reference it fails to resolve.
    out += ["  )", "  (was_is)", "  (routes", f"    (resolution {design.unit} {design.resolution})",
            "    (parser (host_cad weaveengine))"]

    via_name = design.via_padstack or "via"
    out.append("    (library_out")
    if result.vias or design.via_padstack:
        out.append(f"      (padstack {q(via_name)}")
        # (on every copper layer, the routed and those left out: KiCad takes a via of fewer for a blind one)
        shapes = design.via_shapes or [["circle", layer, str(board.rules.via_diameter / UNIT_MM[design.unit.lower()])]
                                       for layer in board.layers + board.unrouted]
        for desc in shapes:
            size = int(round(float(desc[2]) * design.resolution))
            out.append(f"        (shape (circle {desc[1]} {size} 0 0))")
        out += ["        (attach off)", "      )"]
    out.append("    )")

    out.append("    (network_out")
    by_net: dict[int, list[int]] = {}
    for wire, net in result.wire_net.items():
        if wire in result.polylines:
            by_net.setdefault(net, []).append(wire)
    vias_by_net: dict[int, list] = {}
    for via in result.vias:
        vias_by_net.setdefault(via.net_id, []).append(via)
    for net in sorted(set(by_net) | set(vias_by_net)):
        width = n(board.rules.drawn(net))
        out.append(f"      (net {q(names.get(net, f'N{net}'))}")
        for wire in sorted(by_net.get(net, [])):
            layer = board.layers[result.wire_layer.get(wire, 0)]
            # (a tapered trace is several pieces, each of its own width)
            for pts, drawn in result.widths.get(wire) or [(result.polylines[wire], board.rules.drawn(net))]:
                width = n(drawn)
                runs = []  # the wire as paths and arcs, in order
                here, at = [pts[0]], 0
                for first, last, centre, radius in (find_arcs(pts) if arcs else []):
                    if radius * factor < MIN_ARC_UNITS:
                        continue  # too small to hold in whole units: left as segments
                    here += pts[at + 1:first + 1]
                    runs.append(("path", here))
                    runs.append(("arc", arc_pieces(pts, first, last, centre, radius), centre))
                    here, at = [pts[last]], last
                runs.append(("path", here + pts[at + 1:]))
                for run in runs:
                    if run[0] == "arc":
                        cx, cy = run[2]
                        for a, b in run[1]:
                            if (n(a[0]), n(a[1])) != (n(b[0]), n(b[1])):
                                out.append(f"        (wire (qarc {layer} {width} {n(a[0])} {n(a[1])} {n(b[0])} {n(b[1])} {n(cx)} {n(cy)}))")
                        continue
                    # Points that coincide after rounding would make zero-length segments.
                    pp = [f"{n(x)} {n(y)}" for x, y in run[1]]
                    pp = [p for i, p in enumerate(pp) if i == 0 or p != pp[i - 1]]
                    if len(pp) >= 2:
                        out.append(f"        (wire (path {layer} {width} {' '.join(pp)}))")
            if teardrops:
                for poly in result.teardrops.get(wire, []):
                    for w, a, b in teardrop_tracks(poly, board.rules.drawn(net)):
                        if (n(a[0]), n(a[1])) != (n(b[0]), n(b[1])):
                            out.append(f"        (wire (path {layer} {n(w)} {n(a[0])} {n(a[1])} {n(b[0])} {n(b[1])}))")
        for via in vias_by_net.get(net, []):
            x, y = via.centre
            out.append(f"        (via {q(via_name)} {n(x)} {n(y)})")
        out.append("      )")
    out += ["    )", "  )", ")"]
    part = f"{path}.{os.getpid()}.part"
    try:
        with open(part, "w", encoding="utf-8") as f:
            f.write("\n".join(out) + "\n")
        os.replace(part, path)
    except BaseException:
        try:
            os.remove(part)
        except OSError:
            pass
        raise
