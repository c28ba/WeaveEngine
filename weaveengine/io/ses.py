"""Specctra SES (session) writer: each wire is a polyline ``path`` with width (design section 4)."""
import math

from weaveengine.io.dsn import UNIT_MM, Design
from weaveengine.realize.teardrop import tip_of


TEARDROP_OVERLAP = 0.75  # spacing of the fill lines at the pad, in trace widths (below 1 so they overlap)


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
    tip = tip_of(poly)
    # Outer lines: the teardrop's sides moved in by half a trace width.
    first = (t1[0] - (left[0] - tip[0]), t1[1] - (left[1] - tip[1]))
    last = (t2[0] - (right[0] - tip[0]), t2[1] - (right[1] - tip[1]))
    span = math.dist(first, last)
    if math.dist(t1, t2) <= track_width or span <= 1e-6:
        return []
    count = max(2, math.ceil(span / (TEARDROP_OVERLAP * track_width)) + 1)
    ends = [(first[0] + (last[0] - first[0]) * i / (count - 1), first[1] + (last[1] - first[1]) * i / (count - 1))
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
    out = []
    for i in order:
        ex, ey = ends[i]
        dx, dy = ex - tip[0], ey - tip[1]
        length = math.hypot(dx, dy)
        if length <= half + 1e-6:
            continue
        f = (length - half) / length
        out.append((track_width, tip, (tip[0] + dx * f, tip[1] + dy * f)))
    return out


def write_ses(design: Design, result, path: str, teardrops: bool = True) -> None:
    """Writes the routed traces and vias of ``result``. With ``teardrops``,
    each teardrop is added as a fan of ordinary traces (see ``teardrop_tracks``)."""
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
        shapes = design.via_shapes or [["circle", layer, str(board.rules.via_diameter / UNIT_MM[design.unit.lower()])] for layer in board.layers]
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
        width = n(board.rules.width(net))
        out.append(f"      (net {q(names.get(net, f'N{net}'))}")
        for wire in sorted(by_net.get(net, [])):
            layer = board.layers[result.wire_layer.get(wire, 0)]
            # Points that coincide after rounding would make zero-length segments.
            pts = [f"{n(x)} {n(y)}" for x, y in result.polylines[wire]]
            pts = [p for i, p in enumerate(pts) if i == 0 or p != pts[i - 1]]
            if len(pts) < 2:
                continue
            out.append(f"        (wire (path {layer} {width} {' '.join(pts)}))")
            if teardrops:
                for poly in result.teardrops.get(wire, []):
                    for w, a, b in teardrop_tracks(poly, board.rules.width(net)):
                        if (n(a[0]), n(a[1])) != (n(b[0]), n(b[1])):
                            out.append(f"        (wire (path {layer} {n(w)} {n(a[0])} {n(a[1])} {n(b[0])} {n(b[1])}))")
        for via in vias_by_net.get(net, []):
            x, y = via.centre
            out.append(f"        (via {q(via_name)} {n(x)} {n(y)})")
        out.append("      )")
    out += ["    )", "  )", ")"]
    with open(path, "w") as f:
        f.write("\n".join(out) + "\n")
