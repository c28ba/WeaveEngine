"""SVG rendering of a board, its planar map and realised traces."""
from weaveengine.board import Board
from weaveengine.realize.taper import with_wedges

NET_COLORS = ["#4f8dff", "#ff5a4a", "#34c46a", "#ffb02e", "#b06cff", "#22cfcf", "#ff7ac0", "#c8d44a"]
LAYER_COLORS = ["#e0483c", "#3c7be0", "#35b56a", "#c9a227"]  # front, back, inner...


def export_svg(board: Board, filename: str, polylines: dict[int, list[tuple[float, float]]] | None = None,
               wire_net: dict[int, int] | None = None, pmap=None,
               unrouted: list[tuple[tuple[float, float], tuple[float, float]]] | None = None, violations=None,
               scale: float | None = None, padding: float = 20.0, wire_layer: dict[int, int] | None = None,
               teardrops: dict[int, list] | None = None, widths: dict[int, list] | None = None) -> None:
    """Traces are coloured by net on a one-layer board and by layer otherwise.
    ``widths``: wire -> [(points, width)] for traces drawn in pieces (``realize.taper``)."""
    min_x, min_y, max_x, max_y = board.outline.bounds
    if scale is None:
        scale = 900.0 / max(max_x - min_x, max_y - min_y)
    width = (max_x - min_x) * scale + 2 * padding
    height = (max_y - min_y) * scale + 2 * padding
    by_layer = len(board.layers) > 1
    wire_net, wire_layer = wire_net or {}, wire_layer or {}

    def pts(coords) -> str:
        return " ".join(f"{(x - min_x) * scale + padding:.2f},{height - ((y - min_y) * scale + padding):.2f}" for x, y in coords)

    def xy(p) -> tuple[float, float]:
        return ((p[0] - min_x) * scale + padding, height - ((p[1] - min_y) * scale + padding))

    out = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width:.0f}" height="{height:.0f}" viewBox="0 0 {width:.0f} {height:.0f}">',
           f'<rect width="{width:.0f}" height="{height:.0f}" fill="#121212"/>',
           f'<path d="{" ".join("M" + pts(r.coords) + "Z" for r in (board.outline.exterior, *board.outline.interiors))}" fill="#1b2a1f" fill-rule="evenodd" stroke="#7a8a7d" stroke-width="1.5"/>']
    for obs in board.obstacles:
        out.append(f'<polygon points="{pts(obs.shape.exterior.coords)}" fill="#282b30" stroke="#9aa0a6" stroke-width="1" stroke-dasharray="4,3"/>')
    if board.no_via:  # areas only vias are kept out of: a light dotted grey, apart from the dashed grey keepouts
        out.append('<defs><pattern id="no-via" width="6" height="6" patternUnits="userSpaceOnUse" patternTransform="rotate(45)">'
                   '<line x1="0" y1="0" x2="0" y2="6" stroke="#e0e0e0" stroke-width="1.5"/></pattern></defs>')
        for area in board.no_via:
            for part in getattr(area, "geoms", [area]):
                out.append(f'<polygon points="{pts(part.exterior.coords)}" fill="url(#no-via)" stroke="#e0e0e0" stroke-width="1" '
                           f'stroke-dasharray="1,3"><title>no vias</title></polygon>')
    if pmap is not None:
        for e in range(pmap.num_edges):
            u, v = pmap.edge_v_list[e]
            kind = pmap.edge_kind_list[e]
            color, w = ("#3d4a40", 0.5) if kind == 0 else (("#66705f", 0.8) if kind == 1 else ("#8a7a30", 0.8))
            out.append(f'<polyline points="{pts((pmap.vxy[u], pmap.vxy[v]))}" stroke="{color}" stroke-width="{w}" fill="none"/>')
    # Back layers first, so the front layer is drawn on top.
    for wire, line in sorted((polylines or {}).items(), key=lambda kv: -wire_layer.get(kv[0], 0)):
        net = wire_net.get(wire, wire)
        layer = wire_layer.get(wire, 0)
        color = LAYER_COLORS[layer % len(LAYER_COLORS)] if by_layer else NET_COLORS[net % len(NET_COLORS)]
        label = f"wire {wire} (net {board.net_names.get(net, net)}, {board.layers[layer]})"
        for piece, width in (widths or {}).get(wire) or [(line, board.rules.drawn(net))]:
            stroke = max(1.0, width * scale)
            out.append(f'<polyline points="{pts(piece)}" fill="none" stroke="{color}" stroke-width="{stroke:.2f}" '
                       f'stroke-linecap="round" stroke-linejoin="round" opacity="0.85"><title>{label}</title></polyline>')
        for poly in (teardrops or {}).get(wire, []):
            out.append(f'<polygon points="{pts(poly[:5])}" fill="{color}" opacity="0.85"/>')
    for a, b in unrouted or []:
        out.append(f'<polyline points="{pts((a, b))}" stroke="#ffffff" stroke-width="1" stroke-dasharray="5,4" fill="none" opacity="0.7"/>')
    for pad in board.pads:
        color = NET_COLORS[pad.net_id % len(NET_COLORS)] if pad.net_id >= 0 else "#888888"
        fill = "#b8c2cc" if pad.is_via else "#d9b23c"
        out.append(f'<polygon points="{pts(pad.shape.exterior.coords)}" fill="{fill}" stroke="{color}" stroke-width="{1 if pad.is_via else 2}">'
                   f'<title>{"via" if pad.is_via else "pad"} {pad.name or pad.pad_id} (net {board.net_names.get(pad.net_id, pad.net_id)})</title></polygon>')
        if pad.is_via:
            x, y = xy(pad.centre)
            out.append(f'<circle cx="{x:.2f}" cy="{y:.2f}" r="{board.rules.via_drill / 2 * scale:.2f}" fill="#121212"/>')
    for v in violations or []:
        x, y = xy(v.at)
        out.append(f'<circle cx="{x:.2f}" cy="{y:.2f}" r="7" fill="none" stroke="#ff00ff" stroke-width="2"><title>{v.kind} {v.wires}</title></circle>')
    out.append("</svg>")
    with open(filename, "w", encoding="utf-8") as f:
        f.write("\n".join(out))


def export_result(result, filename: str, mesh: bool = False, **kwargs) -> None:
    """Picture of a routing result: traces, teardrops, vias, and dashed airwires for what is unrouted."""
    centre = {p.pad_id: p.centre for p in result.board.pads}
    unrouted = [(centre[result.connections[w].src], centre[result.connections[w].dst]) for w in result.unrouted]
    export_svg(result.board, filename, result.polylines, result.wire_net, pmap=result.pmap if mesh else None,
               unrouted=unrouted, violations=result.violations, wire_layer=result.wire_layer,
               teardrops=with_wedges(result.teardrops, result.wedges), widths=result.widths, **kwargs)
