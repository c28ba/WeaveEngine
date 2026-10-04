"""Independent measurement of a written SES file against the DSN it was routed from.

Nothing here uses the router's own data: the session file is parsed back and
the copper it describes is measured with shapely against the pads (round pads
as true circles), the other traces and the board edge.
"""
import json
from dataclasses import dataclass, field

import shapely
from shapely.geometry import LineString, Point

from weaveengine.io.dsn import UNIT_MM, Design, _child, _children, parse_sexpr


@dataclass
class Measured:
    required_clearance: float
    required_edge: float
    track_to_pad: float = float("inf")      # smallest copper gap, trace to a pad of another net
    track_to_track: float = float("inf")    # smallest copper gap between traces of different nets on a layer
    track_to_edge: float = float("inf")     # smallest gap, trace copper to the board edge
    widths: dict[str, set] = field(default_factory=dict)  # net name -> widths used
    wrong_width: list[str] = field(default_factory=list)
    crossings: int = 0
    traces: int = 0       # includes the short wide traces that make up teardrops
    vias: int = 0

    @property
    def ok(self) -> bool:
        tol = 1.1e-3  # the router works to one micron
        return (self.track_to_pad >= self.required_clearance - tol and self.track_to_track >= self.required_clearance - tol
                and self.track_to_edge >= self.required_edge - tol and not self.wrong_width and not self.crossings)


def read_session(path: str) -> tuple[list[tuple[str, str, float, list[tuple[float, float]]]], list[tuple[str, float, float]]]:
    """(wires, vias) of a SES file in mm: wires are (net, layer, width, points), vias (net, x, y)."""
    root = parse_sexpr(open(path).read())
    routes = _child(root, "routes")
    res = _child(routes, "resolution")
    scale = UNIT_MM[res[1].lower()] / float(res[2])
    wires, vias = [], []
    for net in _children(_child(routes, "network_out"), "net"):
        for wire in _children(net, "wire"):
            p = _child(wire, "path")
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
    outline = board.outline.exterior
    layer_index = {name: i for i, name in enumerate(board.layers)}
    pads = [(Point(p.centre).buffer(p.radius, quad_segs=32) if p.radius else p.shape, p) for p in board.pads]
    pads += [(Point(x, y).buffer(rules.via_diameter / 2.0, quad_segs=32), None) for _, x, y in vias]
    pad_net = [design.net_names.get(p.net_id, "") if p is not None else vias[i - len(board.pads)][0] for i, (_, p) in enumerate(pads)]
    tree = shapely.STRtree([g for g, _ in pads])
    by_layer: dict[str, list] = {}
    for net, layer, width, pts in wires:
        out.widths.setdefault(net, set()).add(round(width, 6))
        want = rules.width(design.net_ids.get(net, -1))
        if width < want - 1e-6:  # wider is fine: teardrops are written as short, wide traces
            out.wrong_width.append(net)
        line = LineString(pts)
        by_layer.setdefault(layer, []).append((net, width, line))
        out.track_to_edge = min(out.track_to_edge, line.distance(outline) - width / 2.0)
        for i in tree.query(line, predicate="dwithin", distance=rules.clearance + width / 2.0 + 1.0).tolist():
            pad = pads[i][1]
            if pad_net[i] != net and (pad is None or pad.on(layer_index.get(layer, 0))):
                out.track_to_pad = min(out.track_to_pad, line.distance(pads[i][0]) - width / 2.0)
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
    rules = json.load(open(path))["board"]["design_settings"]["rules"]
    return {"edge_clearance": float(rules.get("min_copper_edge_clearance", 0.0)),
            "hole_clearance": float(rules.get("min_hole_clearance", 0.0)),
            "min_clearance": float(rules.get("min_clearance", 0.0)),
            "min_track_width": float(rules.get("min_track_width", 0.0))}
