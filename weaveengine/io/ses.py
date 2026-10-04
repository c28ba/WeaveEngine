"""Specctra SES (session) writer: each wire is a polyline ``path`` with width (design section 4)."""
from weaveengine.io.dsn import UNIT_MM, Design


def write_ses(design: Design, result, path: str, teardrops: bool = False) -> None:
    """Writes the routed traces and vias of ``result``.

    ``teardrops`` adds each teardrop as two extra straight traces along its
    edges. SES has no filled-shape wiring that CAD importers accept, so this is
    an outline only; the CAD tool's own teardrop fill does a better job.
    """
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
    by_image: dict[str, list] = {}
    for image, ref, x, y, side, rot in design.placements:
        by_image.setdefault(image, []).append((ref, x, y, side, rot))
    for image, places in by_image.items():
        out.append(f"    (component {q(image)}")
        for ref, x, y, side, rot in places:
            out.append(f"      (place {q(ref)} {int(round(x * design.resolution))} {int(round(y * design.resolution))} {side} {rot:g})")
        out.append("    )")
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
                    _, t1, p1, p2, t2 = poly
                    for a, b in ((t1, p1), (t2, p2)):
                        out.append(f"        (wire (path {layer} {width} {n(a[0])} {n(a[1])} {n(b[0])} {n(b[1])}))")
        for via in vias_by_net.get(net, []):
            x, y = via.centre
            out.append(f"        (via {q(via_name)} {n(x)} {n(y)})")
        out.append("      )")
    out += ["    )", "  )", ")"]
    with open(path, "w") as f:
        f.write("\n".join(out) + "\n")
