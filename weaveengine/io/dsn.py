"""Specctra DSN reader and a minimal writer (design section 4).

Reads the sections the router needs: boundary, rules, keepouts, placement,
library (images and padstacks), network and fixed wiring. v1 routes a single
layer, so pads are taken from one signal layer.
"""
import re
from dataclasses import dataclass, field

from shapely import affinity
from shapely.geometry import LineString, Point, Polygon, box
from shapely.ops import unary_union

from weaveengine.board import Board, Obstacle, Pad, Rules, circle_polygon

UNIT_MM = {"mm": 1.0, "um": 1e-3, "mil": 0.0254, "inch": 25.4, "cm": 10.0}


def parse_sexpr(text: str) -> list:
    """Tiny s-expression reader. Returns nested lists of strings."""
    text = re.sub(r'\(string_quote\s+"\)', "", text)  # KiCad writes a bare quote here
    # A token may join quoted and bare parts, as in the pin reference "Out-I"-1.
    tokens = re.findall(r'[()]|(?:"(?:[^"\\]|\\.)*"|[^\s()"]+)+', text)
    stack: list[list] = [[]]
    for tok in tokens:
        if tok == "(":
            stack.append([])
        elif tok == ")":
            done = stack.pop()
            if not stack:
                raise ValueError("unbalanced ')' in DSN")
            stack[-1].append(done)
        else:
            stack[-1].append(re.sub(r'"((?:[^"\\]|\\.)*)"', r"\1", tok))
    if len(stack) != 1 or not stack[0]:
        raise ValueError("unbalanced parentheses in DSN")
    return stack[0][0]


def _children(node: list, name: str) -> list[list]:
    return [c for c in node if isinstance(c, list) and c and c[0] == name]


def _child(node: list, name: str):
    found = _children(node, name)
    return found[0] if found else None


@dataclass
class Design:
    """A parsed DSN: the routable board plus what the SES writer needs."""
    name: str
    board: Board
    unit: str = "um"
    resolution: int = 10
    net_ids: dict[str, int] = field(default_factory=dict)
    placements: list[tuple[str, str, float, float, str, float]] = field(default_factory=list)  # image, ref, x, y, side, rot (file units)
    via_padstack: str = ""
    via_shapes: list[list] = field(default_factory=list)  # raw shape descriptors of the via padstack

    @property
    def net_names(self) -> dict[int, str]:
        return {i: n for n, i in self.net_ids.items()}


def _shape(desc: list, scale: float):
    """(geometry, layer, circle radius or None) for a DSN shape descriptor, in mm, or None."""
    kind, layer = desc[0], desc[1]
    nums = [float(x) * scale for x in desc[2:] if not isinstance(x, list)]
    if kind == "circle":
        cx, cy = (nums[1], nums[2]) if len(nums) >= 3 else (0.0, 0.0)
        return circle_polygon(cx, cy, nums[0] / 2.0), layer, nums[0] / 2.0
    if kind == "rect":
        return box(min(nums[0], nums[2]), min(nums[1], nums[3]), max(nums[0], nums[2]), max(nums[1], nums[3])), layer, None
    if kind == "polygon":
        pts = list(zip(nums[1::2], nums[2::2]))
        return (Polygon(pts).buffer(0), layer, None) if len(pts) >= 3 else None
    if kind == "path":
        pts = list(zip(nums[1::2], nums[2::2]))
        if len(pts) == 1 or (len(pts) == 2 and pts[0] == pts[1]):
            return Point(pts[0]).buffer(max(nums[0], 1e-6) / 2.0, quad_segs=4), layer, None
        return LineString(pts).buffer(max(nums[0], 1e-6) / 2.0, quad_segs=4), layer, None
    return None


def read_dsn(path: str, layers: list[str] | None = None) -> Design:
    """Reads a DSN. ``layers`` restricts routing to the named signal layers (default: all)."""
    with open(path) as f:
        root = parse_sexpr(f.read())
    if root[0].lower() != "pcb":
        raise ValueError("not a Specctra DSN file")
    unit, resolution = "um", 10
    for key in ("resolution", "unit"):
        node = _child(root, key)
        if node:
            unit = node[1]
            if key == "resolution":
                resolution = int(float(node[2]))
    scale = UNIT_MM[unit.lower()]

    structure = _child(root, "structure")
    all_layers = [n[1] for n in _children(structure, "layer")]
    use = layers or all_layers
    unknown = [name for name in use if name not in all_layers]
    if unknown:
        raise ValueError(f"layers {unknown} not in design (has {all_layers})")
    index = {name: i for i, name in enumerate(use)}
    flip = {all_layers[0]: all_layers[-1], all_layers[-1]: all_layers[0]}

    def layer_set(names, back: bool = False):
        """Layer indices for DSN layer names; None means all routed layers."""
        found = set()
        for name in names:
            if name in ("signal", "pcb"):
                return None
            name = flip.get(name, name) if back else name
            if name in index:
                found.add(index[name])
        return None if len(found) == len(use) else frozenset(found)

    # Boundary: the largest outline wins.
    outlines = []
    for b in _children(structure, "boundary"):
        for desc in b[1:]:
            if isinstance(desc, list):
                nums = [float(x) * scale for x in desc[2:]]
                if desc[0] == "rect":
                    outlines.append(box(min(nums[0], nums[2]), min(nums[1], nums[3]), max(nums[0], nums[2]), max(nums[1], nums[3])))
                elif desc[0] in ("path", "polygon"):
                    pts = list(zip(nums[1::2], nums[2::2]))
                    if len(pts) >= 3:
                        outlines.append(Polygon(pts).buffer(0))
    if not outlines:
        raise ValueError("DSN has no usable boundary")
    outline = max(outlines, key=lambda p: p.area)

    def rule_of(node):
        """(width, clearance) of a rule list; typed clearances (smd_smd, ...) are ignored."""
        width = clearance = None
        for rule in _children(node, "rule"):
            for w in _children(rule, "width"):
                width = float(w[1]) * scale
            for c in _children(rule, "clearance"):
                if not _children(c, "type"):
                    clearance = float(c[1]) * scale
        return width, clearance

    width, clearance = rule_of(structure)
    network = _child(root, "network") or []
    classes = [(cls, *rule_of(cls)) for cls in _children(network, "class")]
    # The class named kicad_default (or the first one) overrides the structure rule as the default.
    for cls, w, c in classes:
        if cls[1] in ("kicad_default", "default") and w:
            width = w
    if not width:
        width = min((w for _, w, _ in classes if w), default=None)
    clearances = [c for c in [clearance] + [c for _, _, c in classes] if c]
    if not width or not clearances:
        raise ValueError("DSN has no width/clearance rule")
    board = Board(outline=outline, rules=Rules(width, max(clearances)), layers=list(use))

    for k in _children(structure, "keepout"):
        for desc in k[1:]:
            if isinstance(desc, list):
                got = _shape(desc, scale)
                if got:
                    on = layer_set([got[1]])
                    if on is None or on:
                        board.obstacles.append(Obstacle(got[0], layers=on))

    library = _child(root, "library") or []
    padstacks = {p[1]: [s[1] for s in _children(p, "shape")] for p in _children(library, "padstack")}
    images = {img[1]: img for img in _children(library, "image")}

    design = Design(root[1], board, unit, resolution)
    via = _child(structure, "via")
    if via and via[1] in padstacks:
        design.via_padstack = via[1]
        design.via_shapes = padstacks[via[1]]
        circles = [float(d[2]) * scale for d in design.via_shapes if d[0] == "circle"]
        if circles:
            board.rules.via_diameter = max(circles)
            m = re.search(r"_(\d+(?:\.\d+)?):(\d+(?:\.\d+)?)_um", via[1])
            board.rules.via_drill = float(m.group(2)) / 1000.0 if m else board.rules.via_diameter / 2.0

    # Nets and their classes.
    pin_net: dict[str, int] = {}
    for net in _children(network, "net"):
        net_id = design.net_ids.setdefault(net[1], len(design.net_ids))
        pins = _child(net, "pins")
        for pin in (pins[1:] if pins else []):
            pin_net[pin] = net_id
    board.net_names = design.net_names
    for cls, w, _ in classes:
        if w and abs(w - width) > 1e-9:
            for name in cls[2:]:
                if isinstance(name, str) and name in design.net_ids:
                    board.rules.net_width[design.net_ids[name]] = w

    # Pads from placed components.
    placement = _child(root, "placement") or []
    for comp in _children(placement, "component"):
        image = images.get(comp[1])
        for place in _children(comp, "place"):
            ref, px, py, side, rot = place[1], float(place[2]), float(place[3]), place[4], float(place[5])
            design.placements.append((comp[1], ref, px, py, side, rot))
            if image is None:
                continue
            back = side == "back"
            for pin in _children(image, "pin"):
                fields = [x for x in pin[1:] if not isinstance(x, list)]
                stack, pin_id, x, y = fields[0], fields[1], float(fields[2]) * scale, float(fields[3]) * scale
                pin_rot = float(_child(pin, "rotate")[1]) if _child(pin, "rotate") else 0.0
                shapes = [g for g in (_shape(d, scale) for d in padstacks.get(stack, [])) if g]
                on = layer_set([g[1] for g in shapes], back)
                if not shapes or (on is not None and not on):
                    continue
                shape = unary_union([g[0] for g in shapes])
                radius = shapes[0][2] if all(g[2] is not None for g in shapes) else None
                shape = affinity.rotate(shape, pin_rot, origin=(0, 0))
                shape = affinity.translate(shape, x, y)
                if back:
                    shape = affinity.scale(shape, xfact=-1.0, origin=(0, 0))
                shape = affinity.rotate(shape, rot, origin=(0, 0))
                shape = affinity.translate(shape, px * scale, py * scale)
                if shape.geom_type != "Polygon":
                    shape = shape.convex_hull
                name = f"{ref}-{pin_id}"
                board.pads.append(Pad(len(board.pads), pin_net.get(name, -1), shape, name, on, radius))

    # Pre-existing copper is a fixed obstacle in v1.
    wiring = _child(root, "wiring") or []
    for wire in _children(wiring, "wire"):
        net = _child(wire, "net")
        net_id = design.net_ids.get(net[1], -1) if net else -1
        for desc in wire[1:]:
            if isinstance(desc, list) and desc[0] in ("path", "polygon", "rect", "circle"):
                got = _shape(desc, scale)
                if got:
                    on = layer_set([got[1]])
                    if on is None or on:
                        board.obstacles.append(Obstacle(got[0], net_id, on))
    for via in _children(wiring, "via"):
        net = _child(via, "net")
        net_id = design.net_ids.get(net[1], -1) if net else -1
        parts = [g[0] for g in (_shape(d, scale) for d in padstacks.get(via[1], [])) if g]
        if parts:
            shape = affinity.translate(unary_union(parts), float(via[2]) * scale, float(via[3]) * scale)
            board.obstacles.append(Obstacle(shape if shape.geom_type == "Polygon" else shape.convex_hull, net_id))
    return design


def write_dsn(board: Board, path: str, name: str = "board", layer: str = "F.Cu") -> None:
    """Writes a board as a single-layer DSN (every pad is its own component).
    Used by the benchmarks and the round-trip test."""
    def u(v: float) -> str:
        return f"{v * 1000:.4f}".rstrip("0").rstrip(".")

    rules = board.rules
    out = [f"(pcb {name}", "  (parser (host_cad weaveengine))", "  (resolution um 10)", "  (unit um)", "  (structure",
           f"    (layer {layer} (type signal))",
           "    (boundary (path pcb 0 " + " ".join(f"{u(x)} {u(y)}" for x, y in board.outline.exterior.coords) + "))"]
    for obs in board.obstacles:
        out.append(f'    (keepout "" (polygon {layer} 0 ' + " ".join(f"{u(x)} {u(y)}" for x, y in obs.shape.exterior.coords) + "))")
    out += [f"    (rule (width {u(rules.trace_width)}) (clearance {u(rules.clearance)}))", "  )", "  (placement"]
    for pad in board.pads:
        x, y = pad.centre
        out.append(f"    (component IMG{pad.pad_id} (place P{pad.pad_id} {u(x)} {u(y)} front 0))")
    out += ["  )", "  (library"]
    for pad in board.pads:
        out.append(f"    (image IMG{pad.pad_id} (pin PS{pad.pad_id} 1 0 0))")
    for pad in board.pads:
        cx, cy = pad.centre
        pts = " ".join(f"{u(x - cx)} {u(y - cy)}" for x, y in pad.shape.exterior.coords)
        out.append(f"    (padstack PS{pad.pad_id} (shape (polygon {layer} 0 {pts})) (attach off))")
    out += ["  )", "  (network"]
    for net_id, pads in sorted(board.nets().items()):
        net_name = board.net_names.get(net_id, f"N{net_id}")
        out.append(f'    (net "{net_name}" (pins ' + " ".join(f"P{p}-1" for p in pads) + "))")
    out += ["  )", "  (wiring)", ")"]
    with open(path, "w") as f:
        f.write("\n".join(out) + "\n")
