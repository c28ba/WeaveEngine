"""Acute angles where copper of one net meets (ROADMAP 1.1 item 6, design 13.3b).

Routes a board as the CLI does and lists every place where copper of one net
meets itself under the least angle (``realize.acute``), by kind: a trace and
its pad, traces at a via, traces meeting away from any pad (``tee``), a
fan-out via or trace, a trace's own corner.

    python bench/acute.py BOARD.dsn [--pictures DIR] [--save FILE.pickle] [weaveengine.cli options]
    python bench/acute.py FILE.pickle [--pictures DIR]

``--pictures DIR``: a PNG of the copper round each meeting (the net's own in
colour, everything else on the layer grey, the meeting ringed), sharpest first.
"""
import math
import os
import pickle
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from shapely.geometry import LineString, Polygon, box  # noqa: E402
from shapely.ops import unary_union  # noqa: E402

from weaveengine import cli  # noqa: E402
from weaveengine.realize.acute import KINDS, meetings  # noqa: E402
from weaveengine.realize.taper import with_wedges  # noqa: E402

PICTURES = 40   # at most this many pictures
SPAN = 1.2      # mm round the meeting


def route(argv: list[str]):
    """(design, result) of routing as the CLI does."""
    got = {}
    write = cli.write_ses

    def keep(design, result, *args, **kw):
        got["design"], got["result"] = design, result
        return write(design, result, *args, **kw)

    cli.write_ses = keep
    try:
        cli.main(argv)
    finally:
        cli.write_ses = write
    return got["design"], got["result"]


def found(result) -> list:
    ends = {**{w: (c.src, c.dst) for w, c in result.connections.items() if w in result.polylines}, **result.wire_pads}
    return meetings(result.board, result.polylines, result.wire_net, result.wire_layer, ends,
                    with_wedges(result.teardrops, result.wedges), result.widths, result.stats.get("least_angle", 90.0))


def _svg(result, m, names: dict) -> str:
    x0, y0 = m.at[0] - SPAN, m.at[1] - SPAN
    view = box(x0, y0, x0 + 2 * SPAN, y0 + 2 * SPAN)
    scale = 400.0  # px per mm
    out = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{int(2 * SPAN * scale)}" height="{int(2 * SPAN * scale)}" '
           f'viewBox="{x0} {-y0 - 2 * SPAN} {2 * SPAN} {2 * SPAN}"><rect x="{x0}" y="{-y0 - 2 * SPAN}" width="{2 * SPAN}" '
           f'height="{2 * SPAN}" fill="white"/><g transform="scale(1,-1)">']

    def poly(shape, colour):
        for g in getattr(shape, "geoms", [shape]):
            if g.is_empty or not g.intersects(view):
                continue
            pts = " ".join(f"{x:.4f},{y:.4f}" for x, y in g.exterior.coords)
            out.append(f'<polygon points="{pts}" fill="{colour}" fill-opacity="0.75"/>')

    rules = result.board.rules
    for p in result.board.pads_on(m.layer):
        poly(p.shape, "#d33" if p.net_id == m.net else "#999")
    for w, line in result.polylines.items():
        if result.wire_layer.get(w) != m.layer or len(line) < 2:
            continue
        net = result.wire_net[w]
        colour = ("#36c" if w in m.wires else "#3a6") if net == m.net else "#bbb"
        poly(LineString(line).buffer(rules.drawn(net) / 2.0), colour)
        for drop in result.teardrops.get(w, []):
            poly(Polygon(drop[:5]).buffer(0), colour)
    own = [LineString(line).buffer(rules.drawn(m.net) / 2.0, quad_segs=32) for w, line in result.polylines.items()
           if result.wire_layer.get(w) == m.layer and result.wire_net[w] == m.net and len(line) > 1]
    own += [Polygon(d[:5]).buffer(0) for w in result.polylines if result.wire_layer.get(w) == m.layer
            and result.wire_net[w] == m.net for d in result.teardrops.get(w, [])]
    own += [p.shape for p in result.board.pads_on(m.layer) if p.net_id == m.net]
    for g in getattr(unary_union(own), "geoms", [unary_union(own)]):
        for ring in (g.exterior, *g.interiors):
            pts = " ".join(f"{x:.4f},{y:.4f}" for x, y in ring.coords)
            out.append(f'<polyline points="{pts}" fill="none" stroke="black" stroke-width="0.004"/>')
    r = 0.06
    out.append(f'<circle cx="{m.at[0]}" cy="{m.at[1]}" r="{r}" fill="none" stroke="black" stroke-width="0.02"/>')
    out.append("</g></svg>")
    return "\n".join(out)


def pictures(result, found_: list, folder: str, names: dict) -> None:
    from PySide6.QtCore import QByteArray
    from PySide6.QtGui import QGuiApplication, QImage, QPainter
    from PySide6.QtSvg import QSvgRenderer
    app = QGuiApplication.instance() or QGuiApplication([])  # noqa: F841
    os.makedirs(folder, exist_ok=True)
    for i, m in enumerate(sorted(found_, key=lambda m: m.angle)[:PICTURES]):
        svg = _svg(result, m, names)
        render = QSvgRenderer(QByteArray(svg.encode()))
        image = QImage(render.defaultSize(), QImage.Format_ARGB32)
        image.fill(0xFFFFFFFF)
        paint = QPainter(image)
        render.render(paint)
        paint.end()
        image.save(os.path.join(folder, f"{i:02d}-{m.kind}-{m.angle:.0f}.png"))


def main(argv: list[str]) -> int:
    folder = None
    if "--pictures" in argv:
        i = argv.index("--pictures")
        folder = argv[i + 1]
        del argv[i:i + 2]
    save = None
    if "--save" in argv:
        i = argv.index("--save")
        save = argv[i + 1]
        del argv[i:i + 2]
    if argv and argv[0].endswith(".pickle"):  # (a routing saved with --save: looked at again without routing)
        with open(argv[0], "rb") as f:
            design, result = pickle.load(f)
    else:
        design, result = route(argv)
    if save:
        with open(save, "wb") as f:
            pickle.dump((design, result.summary()), f)
    names = getattr(design, "net_names", {})
    pads = {p.pad_id: p for p in result.board.pads}
    got = found(result)
    print("acute: " + ", ".join(f"{kind} {sum(1 for m in got if m.kind == kind)}" for kind in KINDS)
          + f"; {sum(1 for p in result.board.pads if p.is_via)} vias")
    for i, m in enumerate(sorted(got, key=lambda m: m.angle)):
        pad = pads[m.pad].name if m.pad is not None and m.pad in pads else ""
        print(f"  {i:02d} {m.kind:6s} {m.angle:5.1f} deg  layer {m.layer} net {names.get(m.net, m.net)} at ({m.at[0]:.3f}, {m.at[1]:.3f})"
              f" wires {list(m.wires)} {pad}")
    if folder:
        pictures(result, got, folder, names)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
