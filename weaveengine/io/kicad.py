"""What a KiCad board says that its Specctra export does not (design 4.1).

KiCad's DSN export carries the pads, the outline, the zones and the net
classes, and leaves out much the board's own design rules check holds traces
to: text and drawings on copper layers, the local clearances of parts and pads,
the drill of every hole (so the hole clearance), what a rule area forbids
(every one is written as a plain keepout), and the board's own minimums.
(Not taken: drawings on the Margin layer, which KiCad's check holds to the
edge clearance. Boards draw there what is not an edge, closer to their own
pads than that; taken as edge, those pads could not be reached.) When the board
(``.kicad_pcb``) and project (``.kicad_pro``) of the same name lie beside the
DSN, they are read for these.

Coordinates here are those of KiCad's DSN export: millimetres, y turned over.
"""
import json
import math
import os
import re
from dataclasses import dataclass, field

from shapely import affinity
from shapely.geometry import LineString, Point, Polygon, box
from shapely.ops import unary_union

ARC_STEPS = 16       # segments of a drawn arc or circle
AROUND = 1.0 / math.cos(math.pi / ARC_STEPS)  # a circle's polygon is drawn round it, never inside (a 3.4 mm hole inside: 0.03 mm short)
CHAR = 1.3           # width of a character of KiCad's stroke font, of the font width (its widest, '>', and room about it)
LINE = 1.62          # line spacing, of the font height
NARROWEST = 0.127    # mm: no trace is narrowed below this, whatever the rules allow (5 mil: what board makers make at their standard price)
TALL = 1.8           # height of a line of text as KiCad's own check sees it, of the font height


@dataclass
class KicadPad:
    ref: str
    number: str
    x: float
    y: float
    layers: set[str]                       # copper layers it has copper on (canonical names; "*" = all)
    drill: float = 0.0                     # the smaller side of its hole (0: none)
    plated: bool = True
    size: float = 0.0                      # the smaller side of the pad
    clearance: float = 0.0                 # its local clearance, else its part's (0: none)
    hole: object = None                    # the hole as a shape, if it has one
    outlines: tuple = ()                   # a custom pad's copper, as placed and as placed mirrored (see ``apply``)


@dataclass
class KicadBoard:
    layer_names: dict[str, str] = field(default_factory=dict)        # canonical copper layer name -> the name the DSN uses
    open_areas: list[tuple] = field(default_factory=list)             # bounds of rule areas that let traces through
    pads: list[KicadPad] = field(default_factory=list)
    copper: list[tuple] = field(default_factory=list)                 # (shape, canonical layer, what it is) on copper layers
    rules: dict[str, float] = field(default_factory=dict)             # the project's minimums (``project_rules``)


def project_rules(path: str) -> dict[str, float]:
    """Board-level minimums from a .kicad_pro file that a DSN does not carry (0: not set)."""
    with open(path, encoding="utf-8") as f:
        rules = json.load(f)["board"]["design_settings"]["rules"]
    get = lambda key: float(rules.get(key) or 0.0)
    return {"edge_clearance": get("min_copper_edge_clearance"), "hole_clearance": get("min_hole_clearance"),
            "min_clearance": get("min_clearance"), "min_track_width": get("min_track_width"),
            "via_annulus": get("min_via_annular_width"), "via_diameter": get("min_via_diameter"),
            "min_drill": get("min_through_hole_diameter")}


def read_kicad(dsn_path: str) -> KicadBoard | None:
    """The board and project beside ``dsn_path`` (same name), or None if there is no board."""
    from weaveengine.io.dsn import parse_sexpr
    stem = os.path.splitext(dsn_path)[0]
    if not os.path.exists(stem + ".kicad_pcb"):
        return None
    try:
        with open(stem + ".kicad_pcb", encoding="utf-8", errors="replace") as f:
            root = parse_sexpr(f.read())
    except (OSError, ValueError, IndexError):
        return None
    if not root or root[0] != "kicad_pcb":
        return None
    out = KicadBoard()
    if os.path.exists(stem + ".kicad_pro"):
        try:
            out.rules = project_rules(stem + ".kicad_pro")
        except (OSError, ValueError, KeyError, TypeError):
            pass
    for layer in _children(_child(root, "layers"), None):
        if len(layer) >= 3 and isinstance(layer[1], str) and layer[1].endswith(".Cu"):
            out.layer_names[layer[1]] = layer[3] if len(layer) > 3 and isinstance(layer[3], str) else layer[1]
    for item in root[1:]:
        if not isinstance(item, list):
            continue
        kind = item[0]
        if kind == "zone":
            _rule_area(item, out)
        elif kind == "footprint" or kind == "module":
            _footprint(item, out)
        elif kind in ("gr_text", "gr_text_box"):
            _text(item, _place(0.0, 0.0, 0.0), out, kind)
        elif kind.startswith("gr_"):
            _drawing(item, _place(0.0, 0.0, 0.0), out, kind)
    return out


# -- the s-expression ----------------------------------------------------------

def _child(node, name):
    for x in node[1:] if isinstance(node, list) else ():
        if isinstance(x, list) and x and x[0] == name:
            return x
    return None


def _children(node, name):
    return [x for x in (node[1:] if isinstance(node, list) else ()) if isinstance(x, list) and x and (name is None or x[0] == name)]


def _numbers(node, n: int):
    return [float(v) for v in node[1:1 + n]] if node else []


def _layers(item) -> list[str]:
    one = _child(item, "layer")
    if one:
        return [one[1]]
    many = _child(item, "layers")
    return [x for x in many[1:] if isinstance(x, str)] if many else []


# -- placing a part's own coordinates on the board -------------------------------

def _place(x: float, y: float, angle: float):
    """A function from a footprint's own coordinates (KiCad's, y down) to the
    DSN's (y up): turned by ``angle`` degrees and moved to x y. A part on the
    back is stored in the file already turned over."""
    a = math.radians(angle)
    c, s = math.cos(a), math.sin(a)
    return lambda u, v: (x + u * c + v * s, -(y - u * s + v * c))


def _footprint(item, out: KicadBoard) -> None:
    at = _numbers(_child(item, "at"), 3)
    fx, fy, angle = (at + [0.0, 0.0, 0.0])[:3]
    place = _place(fx, fy, angle)
    ref = ""
    for prop in _children(item, "property"):
        if len(prop) > 2 and prop[1] == "Reference":
            ref = prop[2]
    for text in _children(item, "fp_text"):
        if len(text) > 2 and text[1] == "reference":
            ref = text[2]
    own = _local_clearance(item)
    for pad in _children(item, "pad"):
        _pad(pad, place, angle, ref, own, out)
    for x in item[1:]:
        if not isinstance(x, list):
            continue
        if x[0] in ("fp_text", "fp_text_box") or (x[0] == "property" and _child(x, "layer")):
            _text(x, place, out, x[0], angle, ref)
        elif x[0].startswith("fp_"):
            _drawing(x, place, out, x[0], ref)


def _local_clearance(item) -> float:
    for name in ("clearance", "local_clearance"):
        node = _child(item, name)
        if node and len(node) > 1:
            return float(node[1])
    return 0.0


def _pad(item, place, fp_angle: float, ref: str, own: float, out: KicadBoard) -> None:
    number = item[1] if len(item) > 1 and isinstance(item[1], str) else ""
    kind = item[2] if len(item) > 2 else ""
    at = _numbers(_child(item, "at"), 3)
    px, py = (at + [0.0, 0.0])[:2]
    x, y = place(px, py)
    size = _numbers(_child(item, "size"), 2)
    layers = set()
    for name in _layers(item):
        if name in ("*.Cu", "F&B.Cu"):
            layers.add("*")
        elif name.endswith(".Cu"):
            layers.add(name)
    pad = KicadPad(ref, number, x, y, layers, plated=kind != "np_thru_hole", size=min(size) if size else 0.0,
                   clearance=_local_clearance(item) or own)
    if len(item) > 3 and item[3] == "custom":
        pad.outlines = tuple(o for o in (_custom(item, x, y, at[2] if len(at) > 2 else 0.0, size, mirrored) for mirrored in (False, True))
                             if o is not None)
    drill = _child(item, "drill")
    if drill and kind in ("thru_hole", "np_thru_hole"):
        values = [float(v) for v in drill[1:] if isinstance(v, str) and _is_number(v)]
        oval = "oval" in drill[1:]
        if values:
            w, h = (values[0], values[1]) if oval and len(values) > 1 else (values[0], values[0])
            pad.drill = min(w, h)
            offset = _numbers(_child(drill, "offset"), 2)
            hx, hy = place(px + (offset[0] if offset else 0.0), py + (offset[1] if offset else 0.0))
            # An oval hole is a slot (a stadium: a line with round ends), not an ellipse, which falls inside it.
            run = (abs(w - h) / 2.0, 0.0) if w >= h else (0.0, abs(w - h) / 2.0)
            hole = LineString([(-run[0], -run[1]), run]).buffer(min(w, h) / 2.0 * AROUND, quad_segs=ARC_STEPS // 4) \
                if oval and abs(w - h) > 1e-9 else Point(0, 0).buffer(w / 2.0 * AROUND, quad_segs=ARC_STEPS // 4)
            turn = (at[2] if len(at) > 2 else 0.0)  # the pad's angle in the file includes its part's
            pad.hole = affinity.translate(affinity.rotate(hole, turn), hx, hy)
    out.pads.append(pad)


def _custom(item, x: float, y: float, turn: float, size: list, mirrored: bool):
    """A custom pad's copper: its anchor and its primitives, turned by the pad's
    angle (which includes its part's) and put at (x, y) in the board's
    coordinates (y up); ``mirrored``, the same flipped about the pad's axis.
    None if it is not one piece."""
    c, s = math.cos(math.radians(turn)), math.sin(math.radians(turn))
    flip = 1.0 if mirrored else -1.0  # the file's y is down
    place = lambda u, v: (x + u * c - flip * v * s, y + u * s + flip * v * c)
    options = _child(item, "options")
    anchor = _child(options, "anchor") if options else None
    w, h = (size + [0.0, 0.0])[:2]
    if anchor and len(anchor) > 1 and anchor[1] == "rect":
        parts = [Polygon([place(-w / 2, -h / 2), place(w / 2, -h / 2), place(w / 2, h / 2), place(-w / 2, h / 2)])]
    else:
        parts = [Point(x, y).buffer(w / 2.0 * AROUND, quad_segs=ARC_STEPS // 4)] if w > 0 else []
    for prim in (_child(item, "primitives") or [])[1:]:
        if isinstance(prim, list) and prim[0].startswith("gr_"):
            shape = _shape(prim, place, prim[0])
            if shape is not None:
                parts.append(shape)
    shape = unary_union(parts) if parts else None
    return shape if isinstance(shape, Polygon) and not shape.is_empty else None


def _is_number(text: str) -> bool:
    try:
        float(text)
        return True
    except ValueError:
        return False


# -- what is drawn and written on copper ---------------------------------------

def _width(item) -> float:
    stroke = _child(item, "stroke")
    node = _child(stroke, "width") if stroke else _child(item, "width")
    return float(node[1]) if node and len(node) > 1 else 0.0


def _filled(item) -> bool:
    fill = _child(item, "fill")
    return bool(fill) and len(fill) > 1 and fill[1] in ("solid", "yes", "true")


def _arc(start, mid, end) -> list[tuple[float, float]]:
    """Points along the circle through three points, from start to end through mid."""
    (ax, ay), (bx, by), (cx, cy) = start, mid, end
    d = 2.0 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
    if abs(d) < 1e-12:
        return [start, end]
    ux = ((ax * ax + ay * ay) * (by - cy) + (bx * bx + by * by) * (cy - ay) + (cx * cx + cy * cy) * (ay - by)) / d
    uy = ((ax * ax + ay * ay) * (cx - bx) + (bx * bx + by * by) * (ax - cx) + (cx * cx + cy * cy) * (bx - ax)) / d
    r = math.hypot(ax - ux, ay - uy)
    t0, t1, tm = (math.atan2(p[1] - uy, p[0] - ux) for p in (start, end, mid))
    span = (t1 - t0) % (2 * math.pi)
    if (tm - t0) % (2 * math.pi) > span:  # mid is the other way round
        span -= 2 * math.pi
    return [(ux + r * math.cos(t0 + span * i / ARC_STEPS), uy + r * math.sin(t0 + span * i / ARC_STEPS)) for i in range(ARC_STEPS + 1)]


def _drawing(item, place, out: KicadBoard, kind: str, ref: str = "") -> None:
    layers = [l for l in _layers(item) if l.endswith(".Cu")]
    if not layers:
        return
    shape = _shape(item, place, kind)
    if shape is None:
        return
    what = f"a drawing of {ref}" if ref else "a drawing"
    for layer in layers:
        out.copper.append((shape, layer, what))


def _shape(item, place, kind: str):
    """The copper of a drawn line, rectangle, circle, arc or polygon (``fp_``,
    ``gr_``, or a custom pad's primitive), placed by ``place``; None if empty."""
    pt = lambda node: place(*_numbers(node, 2)) if node else None
    width = _width(item)
    shape, solid = None, _filled(item)
    name = kind[3:]
    if name == "line":
        shape = LineString([pt(_child(item, "start")), pt(_child(item, "end"))])
    elif name == "rect":  # (its corners placed one by one: a turned part turns it)
        (x0, y0), (x1, y1) = _numbers(_child(item, "start"), 2), _numbers(_child(item, "end"), 2)
        shape = Polygon([place(x0, y0), place(x1, y0), place(x1, y1), place(x0, y1)]).buffer(0)
    elif name == "circle":
        c, e = pt(_child(item, "center")), pt(_child(item, "end"))
        shape = Point(c).buffer(math.dist(c, e) * AROUND, quad_segs=ARC_STEPS // 4)
    elif name == "arc":
        start, mid, end = (pt(_child(item, k)) for k in ("start", "mid", "end"))
        if start and mid and end:
            shape = LineString(_arc(start, mid, end))
    elif name in ("poly", "curve"):
        pts = []
        for node in (_child(item, "pts") or [])[1:]:
            if isinstance(node, list) and node[0] == "xy":
                pts.append(pt(node))
            elif isinstance(node, list) and node[0] == "arc":
                pts += _arc(*(pt(_child(node, k)) for k in ("start", "mid", "end")))
        if len(pts) >= 3 and name == "poly":
            shape = Polygon(pts).buffer(0)
            solid = solid or not _child(item, "fill")  # a polygon on copper is filled unless it says not
        elif len(pts) >= 2:
            shape = LineString(pts)
    if shape is None or shape.is_empty:
        return None
    if isinstance(shape, Polygon) and not solid:
        shape = shape.exterior
    return shape.buffer(max(width, 1e-3) / 2.0, quad_segs=4) if not (isinstance(shape, Polygon) and width <= 0) else shape


def _text(item, place, out: KicadBoard, kind: str, fp_angle: float = 0.0, ref: str = "") -> None:
    """Text on a copper layer, as the box it fills (KiCad's stroke font is not
    modelled: a box a little larger than the letters)."""
    layers = [l for l in _layers(item) if l.endswith(".Cu")]
    if not layers or _child(item, "hide") or "hide" in item[1:]:
        return
    effects = _child(item, "effects")
    if effects and ("hide" in effects[1:] or _child(effects, "hide")):
        return
    text = next((x for x in item[1:] if isinstance(x, str) and x not in ("reference", "value", "user", "Reference", "Value")
                 and not _is_number(x)), "")
    if kind in ("property", "fp_text"):  # (fp_text reference "R1"): its kind first
        text = item[2] if len(item) > 2 and isinstance(item[2], str) else ""
    elif kind in ("gr_text", "gr_text_box", "fp_text_box"):  # its text first, which may be a number ("4")
        text = item[1] if len(item) > 1 and isinstance(item[1], str) else ""
    if not text.strip():
        return
    font = _child(effects, "font") if effects else None
    size = _numbers(_child(font, "size"), 2) if font else []
    height, width = (size + [1.0, 1.0])[:2]
    thick = float(_child(font, "thickness")[1]) if font and _child(font, "thickness") else 0.15 * height
    lines = text.split("\\n") if "\\n" in text else text.split("\n")
    w = max(len(line) for line in lines) * CHAR * width + 2.0 * thick
    h = (len(lines) - 1) * LINE * height + TALL * height + 2.0 * thick
    justify = _child(effects, "justify") if effects else None
    flags = set(justify[1:]) if justify else set()
    left = -w / 2.0 if not flags & {"left", "right"} else (0.0 if "left" in flags else -w)
    if "mirror" in flags:
        left = -w - left
    bottom = -h / 2.0 if not flags & {"top", "bottom"} else (-h if "top" in flags else 0.0)
    at = _numbers(_child(item, "at"), 3)
    tx, ty = (at + [0.0, 0.0])[:2]
    angle = at[2] if len(at) > 2 else 0.0
    shape = box(left, bottom, left + w, bottom + h)               # around the anchor, y up
    shape = affinity.rotate(shape, angle, origin=(0, 0))          # (the angle in the file includes a part's)
    x, y = place(tx, ty)
    shape = affinity.translate(shape, x, y)
    what = f"text '{text[:20]}'" + (f" of {ref}" if ref else "")
    for layer in layers:
        out.copper.append((shape, layer, what))


def _rule_area(item, out: KicadBoard) -> None:
    keepout = _child(item, "keepout")
    if keepout is None:
        return  # a copper zone, not a rule area
    if any(len(rule) > 1 and rule[1] == "not_allowed" for rule in _children(keepout, "tracks")):
        return  # it does keep traces out
    for polygon in _children(item, "polygon"):
        pts = [(float(xy[1]), -float(xy[2])) for xy in _children(_child(polygon, "pts"), "xy")]
        if pts:
            xs, ys = [x for x, _ in pts], [y for _, y in pts]
            out.open_areas.append((min(xs), min(ys), max(xs), max(ys)))


def apply(design, kicad: KicadBoard) -> str:
    """Puts into ``design`` (as ``read_dsn`` made it) what ``kicad`` adds:
    the board's minimum clearance, vias as large as its minimum ring and drill,
    the hole clearance (from vias, from pads whose ring is narrow, and from holes
    on layers their pad has no copper on), local clearances, text and drawings
    on copper. Returns a line saying what
    was taken, or "" if nothing was."""
    import shapely
    from weaveengine.board import Obstacle
    board, rules, r = design.board, design.board.rules, kicad.rules
    said = []
    index = {name: i for i, name in enumerate(board.layers)}
    on = lambda canonical: index.get(kicad.layer_names.get(canonical, canonical))
    rules.min_width = max(r.get("min_track_width", 0.0), NARROWEST)
    if r.get("min_clearance", 0.0) > rules.clearance + 1e-9:
        rules.clearance = r["min_clearance"]
        said.append(f"the board's minimum clearance, {rules.clearance:g} mm")
    drill = max(rules.via_drill, r.get("min_drill", 0.0))
    diameter = max(rules.via_diameter + drill - rules.via_drill, drill + 2.0 * r.get("via_annulus", 0.0), r.get("via_diameter", 0.0))
    if diameter > rules.via_diameter + 1e-9 or drill > rules.via_drill + 1e-9:
        rules.via_diameter, rules.via_drill = diameter, drill
        design.via_shapes = []
        design.via_padstack = f"Via[0-{len(board.layers) - 1}]_{round(diameter * 1000)}:{round(drill * 1000)}_um"
        said.append(f"vias of {diameter:g}/{drill:g} mm, the board's minimum ring and drill")
    hole = r.get("hole_clearance", 0.0)
    if hole > (rules.via_diameter - rules.via_drill) / 2.0 + 1e-9:
        rules.via_clearance = hole - (rules.via_diameter - rules.via_drill) / 2.0
    # Custom pads: their own copper. A DSN gives only its convex hull, which can
    # cover another pad (a reversible part's jumpers, bent round each other's pins).
    customs: dict[tuple[str, str], list[KicadPad]] = {}
    for kp in kicad.pads:
        if kp.outlines:
            customs.setdefault((kp.ref, kp.number), []).append(kp)
    shaped = 0
    for pad in board.pads if customs else ():
        ref, _, number = pad.name.split("@")[0].rpartition("-")
        best = None
        for kp in customs.get((ref, number), ()):
            for outline in kp.outlines:  # (as placed and mirrored: the one whose hull is the DSN's)
                off = outline.convex_hull.symmetric_difference(pad.shape).area / max(pad.shape.area, 1e-9)
                if off < 0.05 and (best is None or off < best[0]):
                    best = (off, outline, kp)
        if best and best[1].area < pad.shape.area * 0.98:
            pad.shape, pad.anchor = best[1], (best[2].x, best[2].y)
            shaped += 1
    if shaped:
        said.append(f"the shapes of {shaped} custom pads (a DSN has only their outline's hull)")
    # Pads: their local clearance, and what the hole clearance asks where the ring is narrow.
    found: dict[tuple[str, str], list[KicadPad]] = {}
    for pad in kicad.pads:
        found.setdefault((pad.ref, pad.number), []).append(pad)
    local = narrow = 0
    for pad in board.pads:
        ref, _, number = pad.name.split("@")[0].rpartition("-")
        mine = found.get((ref, number))
        if not mine:
            continue
        cx, cy = pad.centre
        kp = min(mine, key=lambda p: (p.x - cx) ** 2 + (p.y - cy) ** 2)
        if (kp.x - cx) ** 2 + (kp.y - cy) ** 2 > 4.0:
            continue
        need = kp.clearance
        local += kp.clearance > rules.clearance + 1e-9
        if kp.drill and kp.plated and hole > (kp.size - kp.drill) / 2.0 + 1e-9:
            narrow += hole - (kp.size - kp.drill) / 2.0 > rules.clearance + 1e-9
            need = max(need, hole - (kp.size - kp.drill) / 2.0)
        pad.clearance = max(pad.clearance, need)
    if local:
        said.append(f"local clearances on {local} pads")
    # Holes where no copper of their pad is: on every layer for one without plating, else on the layers it lacks.
    holes = 0
    every = frozenset(range(len(board.layers)))
    for kp in kicad.pads:
        if kp.hole is None:
            continue
        has = every if "*" in kp.layers else frozenset(i for i in (on(l) for l in kp.layers) if i is not None)
        bare = every if not kp.plated else every - has
        if bare:
            # (a local clearance of the pad holds its hole too: a mounting hole's 1.55 mm, against the board's 0.15)
            board.obstacles.append(Obstacle(kp.hole, layers=None if bare == every else bare, clearance=max(hole, kp.clearance), copper=True))
            holes += 1
    kept = [x for x in ("vias" if rules.via_clearance else "", f"{narrow} pads with a narrow ring" if narrow else "",
                        f"{holes} holes" if holes else "") if x]
    if kept:
        said.append(f"the hole clearance, {hole:g} mm, kept from " + ", ".join(kept))
    # Copper that is not a pad: its net is that of the pads it touches; one that joins two nets is a net tie, and left alone.
    pads = [p for p in board.pads]
    tree = shapely.STRtree([p.shape for p in pads]) if pads else None
    drawn = 0
    for shape, layer, _ in kicad.copper:
        i = on(layer)
        if i is None:
            continue
        nets = {pads[j].net_id for j in tree.query(shape, predicate="intersects").tolist() if pads[j].layers is None or i in pads[j].layers} if tree else set()
        if len(nets) > 1:
            continue
        net = next(iter(nets)) if nets else -1
        board.obstacles.append(Obstacle(shape, net, frozenset({i}), copper=True))
        drawn += 1
    if drawn:
        said.append(f"{drawn} texts and drawings on copper")
    return ", ".join(said)
