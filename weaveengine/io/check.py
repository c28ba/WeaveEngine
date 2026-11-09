"""Independent measurement of a written SES file against the DSN it was routed from.

Nothing here uses the router's own data: the session file is parsed back and
the copper it describes is measured with shapely against the pads (round pads
as true circles), the other traces and the board edge.
"""
import json
import math
from dataclasses import dataclass, field

import shapely
from shapely.geometry import LineString, Point

from weaveengine.io.dsn import UNIT_MM, Design, _child, _children, parse_sexpr


@dataclass
class Measured:
    required_clearance: float
    required_edge: float
    track_to_pad: float = float("inf")      # smallest copper gap, trace to a pad or fixed copper of another net (less what it asks beyond the rule)
    track_to_keepout: float = float("inf")  # smallest gap, trace copper to a keepout (which it may not enter)
    track_to_track: float = float("inf")    # smallest copper gap between traces of different nets on a layer
    track_to_edge: float = float("inf")     # smallest gap, trace copper to the board edge
    widths: dict[str, set] = field(default_factory=dict)  # net name -> widths used
    wrong_width: list[str] = field(default_factory=list)
    crossings: int = 0
    vias_in_keepout: int = 0  # vias whose copper is inside a keepout or an area no via may touch
    traces: int = 0       # includes the short wide traces that make up teardrops
    vias: int = 0

    @property
    def ok(self) -> bool:
        tol = 1.1e-3  # the router works to one micron
        return (self.track_to_pad >= self.required_clearance - tol and self.track_to_track >= self.required_clearance - tol
                and self.track_to_edge >= self.required_edge - tol and self.track_to_keepout >= -tol
                and not self.wrong_width and not self.crossings and not self.vias_in_keepout)


def _sampled_arc(x1, y1, x2, y2, cx, cy) -> list[tuple[float, float]]:
    """The arc of a ``qarc`` (start, end, centre; the shorter way round) as points one degree apart."""
    r1, r2 = math.hypot(x1 - cx, y1 - cy), math.hypot(x2 - cx, y2 - cy)
    a1, a2 = math.atan2(y1 - cy, x1 - cx), math.atan2(y2 - cy, x2 - cx)
    sweep = (a2 - a1 + math.pi) % (2.0 * math.pi) - math.pi
    n = max(1, math.ceil(abs(sweep) / math.radians(1.0)))
    pts = [(cx + (r1 + (r2 - r1) * k / n) * math.cos(a1 + sweep * k / n), cy + (r1 + (r2 - r1) * k / n) * math.sin(a1 + sweep * k / n))
           for k in range(1, n)]
    return [(x1, y1)] + pts + [(x2, y2)]


def read_session(path: str) -> tuple[list[tuple[str, str, float, list[tuple[float, float]]]], list[tuple[str, float, float]]]:
    """(wires, vias) of a SES file in mm: wires are (net, layer, width, points), vias (net, x, y)."""
    root = parse_sexpr(open(path, encoding="utf-8").read())
    routes = _child(root, "routes")
    res = _child(routes, "resolution")
    scale = UNIT_MM[res[1].lower()] / float(res[2])
    wires, vias = [], []
    for net in _children(_child(routes, "network_out"), "net"):
        for wire in _children(net, "wire"):
            p = _child(wire, "path")
            if p is None:
                p = _child(wire, "qarc")
                nums = [float(v) * scale for v in p[3:]]
                wires.append((net[1], p[1], float(p[2]) * scale, _sampled_arc(*nums)))
                continue
            nums = [float(v) * scale for v in p[3:]]
            wires.append((net[1], p[1], float(p[2]) * scale, list(zip(nums[0::2], nums[1::2]))))
        for via in _children(net, "via"):
            vias.append((net[1], float(via[2]) * scale, float(via[3]) * scale))
    return wires, vias


def measure(design: Design, ses_path: str, clearance: float | None = None, edge: float | None = None) -> Measured:
    """Measures the session against ``clearance`` and ``edge`` (default: the board's rules)."""
    board, rules = design.board, design.board.rules
    if clearance is None:
        clearance = rules.clearance
    if edge is None:
        edge = rules.clearance if rules.edge_clearance is None else rules.edge_clearance
    out = Measured(clearance, edge)
    wires, vias = read_session(ses_path)
    out.traces, out.vias = len(wires), len(vias)
    outline = board.outline.boundary  # the edge and its cutouts
    layer_index = {name: i for i, name in enumerate(board.layers)}
    # Pads and fixed copper (each with its layers), then the session's vias (on every layer). A
    # keepout is held to what the CAD tool asks of it: no copper inside (KiCad writes a hole
    # without copper as a keepout that includes the hole clearance already). The router keeps
    # the clearance from keepouts as well, except where a pad of the design meets one.
    # Copper of no net (text, a logo) is held to the clearance like a pad. What an item
    # asks beyond the rule (a local clearance, a hole's) counts against the gap to it.
    pads = [(Point(p.centre).buffer(p.radius, quad_segs=32) if p.radius else p.shape, p) for p in board.pads]
    pads += [(o.shape, o) for o in board.obstacles if o.net_id >= 0 or o.copper]
    keepouts = [o for o in board.obstacles if o.net_id < 0 and not o.copper]
    pad_net = [design.net_names.get(p.net_id, "") if p.net_id >= 0 else None for _, p in pads]
    beyond = [max(0.0, p.clearance - clearance) for _, p in pads]
    pads += [(Point(x, y).buffer(rules.via_diameter / 2.0, quad_segs=32), None) for _, x, y in vias]
    pad_net += [net for net, _, _ in vias]
    beyond += [max(0.0, rules.via_clearance - clearance)] * len(vias)
    # A via goes through every layer, so a keepout on any layer, or an area no via may touch,
    # forbids its copper.
    forbidden = [o.shape for o in keepouts] + list(board.no_via)
    for _, x, y in vias:
        if any(Point(x, y).buffer(rules.via_diameter / 2.0, quad_segs=32).intersects(area) for area in forbidden):
            out.vias_in_keepout += 1
    tree = shapely.STRtree([g for g, _ in pads])
    by_layer: dict[str, list] = {}
    for net, layer, width, pts in wires:
        out.widths.setdefault(net, set()).add(round(width, 6))
        want = min(rules.width(design.net_ids.get(net, -1)), rules.min_width or math.inf)  # narrowed to reach its pads: down to the rules' least
        if width < want - 1e-6:  # wider is fine: teardrops are written as short, wide traces
            out.wrong_width.append(net)
        line = LineString(pts)
        by_layer.setdefault(layer, []).append((net, width, line))
        out.track_to_edge = min(out.track_to_edge, line.distance(outline) - width / 2.0)
        for o in keepouts:
            if o.on(layer_index.get(layer, 0)):
                out.track_to_keepout = min(out.track_to_keepout, line.distance(o.shape) - width / 2.0)
        for i in tree.query(line, predicate="dwithin", distance=rules.clearance + width / 2.0 + 1.0).tolist():
            pad = pads[i][1]
            if pad_net[i] != net and (pad is None or pad.on(layer_index.get(layer, 0))):
                out.track_to_pad = min(out.track_to_pad, line.distance(pads[i][0]) - width / 2.0 - beyond[i])
    for items in by_layer.values():
        lines = [l for _, _, l in items]
        t = shapely.STRtree(lines)
        a, b = t.query(lines, predicate="dwithin", distance=rules.clearance + 2.0)
        for i, j in zip(a.tolist(), b.tolist()):
            if i < j and items[i][0] != items[j][0]:
                if lines[i].crosses(lines[j]):
                    out.crossings += 1
                out.track_to_track = min(out.track_to_track, lines[i].distance(lines[j]) - (items[i][1] + items[j][1]) / 2.0)
    return out


def kicad_project_rules(path: str) -> dict[str, float]:
    """Board-level minimums from a .kicad_pro file that a DSN does not carry."""
    rules = json.load(open(path, encoding="utf-8"))["board"]["design_settings"]["rules"]
    return {"edge_clearance": float(rules.get("min_copper_edge_clearance", 0.0)),
            "hole_clearance": float(rules.get("min_hole_clearance", 0.0)),
            "min_clearance": float(rules.get("min_clearance", 0.0)),
            "min_track_width": float(rules.get("min_track_width", 0.0))}
