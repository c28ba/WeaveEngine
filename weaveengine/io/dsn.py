"""Specctra DSN reader and a minimal writer (design section 4).

Reads the sections the router needs: boundary, rules, keepouts, placement,
library (images and padstacks), network and fixed wiring. v1 routes a single
layer, so pads are taken from one signal layer.
"""
import math
import os
import re
from dataclasses import dataclass, field

from shapely import affinity
from shapely.geometry import LineString, Polygon, box
from shapely.ops import unary_union

from weaveengine.board import Board, Component, Obstacle, Pad, Rules, circle_polygon
from weaveengine.io.kicad import apply as apply_kicad, read_kicad

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
    defaulted: dict[str, str] = field(default_factory=dict)  # rule the DSN did not give -> what is used instead
    dropped_wiring: str = ""                                 # what was done with routing already in the file
    kicad: str = ""                                          # what was taken from the KiCad board and project beside the DSN
    walled: list[str] = field(default_factory=list)          # layers a keepout covers whole, not routed (a single-sided board)

    @property
    def net_names(self) -> dict[int, str]:
        return {i: n for n, i in self.net_ids.items()}


def walled_note(walled: list[str]) -> str:
    return (f"{', '.join(walled)} {'is' if len(walled) == 1 else 'are'} covered by a keepout that forbids tracks: "
            "not routed (a single-sided board)")


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
        pts = [p for i, p in enumerate(pts) if i == 0 or p != pts[i - 1]]
        half = max(nums[0], 1e-6) / 2.0
        if len(pts) == 1:  # a round pad (KiCad writes an oval of equal sides so)
            return circle_polygon(*pts[0], half), layer, half
        # Its round ends and corners circumscribed, as circles are (circle_polygon):
        # a polygon inside the true copper would let a trace come too close.
        return unary_union([LineString(pts).buffer(half, cap_style="flat", join_style="bevel"),
                            *(circle_polygon(x, y, half) for x, y in pts)]), layer, None
    return None


def _outline(desc: list, scale: float) -> list[tuple[float, float]] | None:
    """A footprint outline element as a polyline in mm (footprint coordinates)."""
    kind = desc[0]
    nums = [float(x) * scale for x in desc[2:] if not isinstance(x, list)]
    if kind in ("path", "polygon"):
        pts = list(zip(nums[1::2], nums[2::2]))
        if kind == "polygon" and pts and pts[0] != pts[-1]:
            pts.append(pts[0])
        return pts if len(pts) >= 2 else None
    if kind == "rect":
        x0, y0, x1, y1 = nums[:4]
        return [(x0, y0), (x1, y0), (x1, y1), (x0, y1), (x0, y0)]
    if kind == "circle":
        r = nums[0] / 2.0
        cx, cy = (nums[1], nums[2]) if len(nums) >= 3 else (0.0, 0.0)
        return [(cx + r * math.cos(2 * math.pi * i / 32), cy + r * math.sin(2 * math.pi * i / 32)) for i in range(33)]
    return None


def read_dsn(path: str, layers: list[str] | None = None) -> Design:
    """Reads a DSN. ``layers`` restricts routing to the named signal layers (default: all).
    Raises ValueError, with a readable message, for anything that is not a usable DSN."""
    try:
        return _read_dsn(path, layers)
    except ValueError:
        raise
    except (IndexError, KeyError, TypeError, AttributeError) as error:
        raise ValueError(f"not a readable Specctra DSN file: a required section is missing or malformed "
                         f"({type(error).__name__}: {error})") from error


def _read_dsn(path: str, layers: list[str] | None = None) -> Design:
    with open(path, encoding="utf-8", errors="replace") as f:
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
    if structure is None:
        raise ValueError("DSN has no structure section (layers and boundary)")
    all_layers = [n[1] for n in _children(structure, "layer")]
    if not all_layers:
        raise ValueError("DSN has no signal layer")
    use = layers or all_layers
    unknown = [name for name in use if name not in all_layers]
    if unknown:
        raise ValueError(f"layers {unknown} not in design (has {all_layers})")

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
    # A keepout on every layer at once ("signal") is how KiCad writes a cutout
    # in the board (Edge.Cuts inside the outline); a keepout area it writes
    # per layer. A cutout is board edge, and keeps the edge clearance.
    keepouts, cutouts = [], []
    for k in _children(structure, "keepout"):
        for desc in k[1:]:
            got = _shape(desc, scale) if isinstance(desc, list) else None
            if got:
                (cutouts if got[1] in ("signal", "pcb") else keepouts).append(got)
    if cutouts:
        outline = outline.difference(unary_union([g[0] for g in cutouts]))
        if outline.geom_type != "Polygon":
            outline = max(outline.geoms, key=lambda p: p.area)

    # A KiCad board beside the DSN says more than its export does (io/kicad.py); first, which rule areas let traces through.
    kicad = read_kicad(path)
    open_areas = kicad.open_areas if kicad else []

    def forbids(shape) -> bool:
        return not any(all(abs(a - b) < 0.01 for a, b in zip(shape.bounds, area)) for area in open_areas)

    # A layer one keepout covers whole is not routed: how a single-sided board
    # is drawn in KiCad (two copper layers, a rule area forbidding tracks on one).
    walled = []
    if layers is None:
        for name in use:
            shut = [shape for shape, layer, _ in keepouts if layer == name and forbids(shape)]
            if shut and outline.difference(unary_union(shut)).area < 1e-3 * outline.area:
                walled.append(name)
        if len(walled) < len(use):
            use = [name for name in use if name not in walled]
        else:
            walled = []
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

    def rule_of(node):
        """(width, clearance) of a rule list; typed clearances (smd_smd, ...) are ignored."""
        width = clearance = None
        for rule in _children(node, "rule"):
            for w in _children(rule, "width"):
                width = float(w[1]) * scale
            for c in _children(rule, "clearance") + _children(rule, "clear"):  # some tools write "clear"
                if not _children(c, "type"):
                    clearance = float(c[1]) * scale
        return width, clearance

    width, clearance = rule_of(structure)
    network = _child(root, "network") or []
    classes = [(cls, *rule_of(cls)) for cls in _children(network, "class")]
    # The class named kicad_default (or the first one) overrides the structure rule as the default.
    for cls, w, c in classes:
        if len(cls) > 1 and cls[1] in ("kicad_default", "default") and w:
            width = w
    if not width:
        width = min((w for _, w, _ in classes if w), default=None)
    clearances = [c for c in [clearance] + [c for _, _, c in classes] if c]
    # A DSN need not carry rules (section 4): whatever is missing takes the built-in default.
    rules, defaulted = Rules(), {}
    if width:
        rules.trace_width = width
    else:
        defaulted["trace_width"] = f"trace width {rules.trace_width:g} mm"
    if clearances:
        rules.clearance = max(clearances)
    else:
        defaulted["clearance"] = f"clearance {rules.clearance:g} mm"
    board = Board(outline=outline, rules=rules, layers=list(use))

    ignored = 0
    for shape, layer, _ in keepouts:
        if not forbids(shape):
            ignored += 1
            continue
        on = layer_set([layer])
        if on is None or on:
            board.obstacles.append(Obstacle(shape, layers=on))

    library = _child(root, "library") or []
    padstacks = {p[1]: [s[1] for s in _children(p, "shape")] for p in _children(library, "padstack")}
    images = {img[1]: img for img in _children(library, "image")}

    design = Design(root[1] if len(root) > 1 and isinstance(root[1], str) else "board", board, unit, resolution,
                    defaulted=defaulted, walled=walled)
    via = _child(structure, "via")
    circles = []
    if via and len(via) > 1 and via[1] in padstacks:
        design.via_padstack = via[1]
        design.via_shapes = padstacks[via[1]]
        circles = [float(d[2]) * scale for d in design.via_shapes if d[0] == "circle" and len(d) > 2]
        if circles:
            board.rules.via_diameter = max(circles)
            m = re.search(r"_(\d+(?:\.\d+)?):(\d+(?:\.\d+)?)_um", via[1])
            board.rules.via_drill = float(m.group(2)) / 1000.0 if m else board.rules.via_diameter / 2.0
    if not circles:
        design.via_padstack, design.via_shapes = "", []  # the writer makes a round one of the default size
        defaulted["via_diameter"] = f"via diameter {board.rules.via_diameter:g} mm"
        defaulted["via_drill"] = f"via drill {board.rules.via_drill:g} mm"

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

    # Copper planes (zones in the CAD tool): which nets they connect is decided once the pads are known.
    planes = [(design.net_ids[plane[1]], got) for plane in _children(structure, "plane") if plane[1] in design.net_ids
              for got in (_shape(d, scale) for d in plane[2:] if isinstance(d, list)) if got]

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
            part_number = _child(place, "PN")
            component = Component(ref, part_number[1] if part_number and len(part_number) > 1 else "", comp[1], side,
                                  px * scale, py * scale, rot)
            board.components.append(component)

            def on_board(shape):
                """A shape in the image's coordinates, on the board."""
                if back:
                    shape = affinity.scale(shape, xfact=-1.0, origin=(0, 0))
                return affinity.translate(affinity.rotate(shape, rot, origin=(0, 0)), px * scale, py * scale)
            for outline in _children(image, "outline"):
                for desc in outline[1:]:
                    line = _outline(desc, scale) if isinstance(desc, list) else None
                    if line:
                        placed = []
                        for x, y in line:
                            if back:
                                x = -x
                            a = math.radians(rot)
                            placed.append((px * scale + x * math.cos(a) - y * math.sin(a),
                                           py * scale + x * math.sin(a) + y * math.cos(a)))
                        component.outlines.append(placed)
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
                shape = on_board(affinity.translate(affinity.rotate(shape, pin_rot, origin=(0, 0)), x, y))
                if shape.geom_type != "Polygon":
                    shape = shape.convex_hull
                name = f"{ref}-{pin_id}"
                component.pads.append(len(board.pads))
                board.pads.append(Pad(len(board.pads), pin_net.get(name, -1), shape, name, on, radius))
            # The part's own keepouts: KiCad writes a hole without copper (NPTH) as one per layer.
            for k in _children(image, "keepout"):
                for shape, layer, _ in filter(None, (_shape(d, scale) for d in k[1:] if isinstance(d, list))):
                    on = layer_set([layer], back)
                    if on is None or on:
                        board.obstacles.append(Obstacle(on_board(shape), layers=on))

    # Nets with a copper plane are connected by that plane, not by traces. Pours
    # are not modelled, so those nets are left alone. KiCad writes every zone as
    # a plane, teardrops too (zones since KiCad 7), so only a plane that spreads
    # over two separate pads of its net (on any layer: one inside the board is
    # reached through vias) and is larger than they are is taken for a pour. A
    # teardrop touches one pad, or two that overlap (KiCad writes two pads of one
    # number at one place as "U1-K9" and "U1-K9@1"), and is smaller than they are.
    by_net: dict[int, list[Pad]] = {}
    for pad in board.pads:
        by_net.setdefault(pad.net_id, []).append(pad)
    for net_id, (shape, layer, _) in planes:
        reached = unary_union([pad.shape for pad in by_net.get(net_id, ()) if pad.shape.intersects(shape)])
        if len(getattr(reached, "geoms", [reached] if not reached.is_empty else [])) >= 2 and shape.area > reached.area:
            board.plane_nets.add(net_id)
            if layer in board.layers:
                board.pours.append((net_id, board.layers.index(layer), shape))

    if kicad is not None:
        taken = apply_kicad(design, kicad)
        if ignored:
            taken += (", " if taken else "") + f"{ignored} keepouts that do not forbid traces (pour keepouts, placement areas) left out"
        if taken:
            design.kicad = f"from {os.path.basename(os.path.splitext(path)[0])}.kicad_pcb/.kicad_pro: {taken}"

    # Pre-existing copper that is marked fixed is an obstacle. The rest is
    # routing some earlier run left behind (a saved, already-routed design):
    # it is discarded and those connections are routed afresh. Keeping its
    # vias while its traces are redone would only litter the board with walls.
    wiring = _child(root, "wiring") or []

    def fixed(item) -> bool:
        kind = _child(item, "type")
        return kind is not None and len(kind) > 1 and kind[1] in ("protect", "fix", "shove_fixed")

    loose = [item for item in wiring[1:] if isinstance(item, list) and item[0] in ("wire", "via") and not fixed(item)]
    unread = [w for w in _children(wiring, "wire") if fixed(w) and not any(
        isinstance(x, list) and x[0] in ("path", "polygon", "rect", "circle") for x in w[1:])]
    if loose:
        design.dropped_wiring = f"{len(loose)} existing traces and vias that are not marked fixed are discarded and routed afresh"
    if unread:
        design.dropped_wiring += ("; " if design.dropped_wiring else "") + \
            f"{len(unread)} fixed traces are in a form that is not read (polyline_path) and are ignored"
    for wire in _children(wiring, "wire"):
        if not fixed(wire):
            continue
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
        if not fixed(via):
            continue
        net = _child(via, "net")
        net_id = design.net_ids.get(net[1], -1) if net else -1
        parts = [g[0] for g in (_shape(d, scale) for d in padstacks.get(via[1], [])) if g]
        if parts:
            shape = affinity.translate(unary_union(parts), float(via[2]) * scale, float(via[3]) * scale)
            board.obstacles.append(Obstacle(shape if shape.geom_type == "Polygon" else shape.convex_hull, net_id, via=True))
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
    for hole in board.outline.interiors:
        out.append('    (keepout "" (polygon signal 0 ' + " ".join(f"{u(x)} {u(y)}" for x, y in hole.coords) + "))")
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
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(out) + "\n")
