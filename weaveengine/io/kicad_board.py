"""A routed copy of the KiCad board: the routing written straight into a copy of
the ``.kicad_pcb`` the DSN was exported from, beside the session file.

The session file (``io.ses``) can only carry traces, arcs and vias: KiCad's
import of it drops a ``wire`` of any other shape (a polygon, a rectangle)
without a word. A teardrop or a taper's wedge has to go into a session as a fan
of tracks. A board file can hold a teardrop as what KiCad itself makes for
one: a small zone marked as one (``(attr (teardrop (type padvia)))``), filled
when the zones are filled, and one shape to select or delete.

The copy is the original board, text for text, with:

- the tracks, arcs and vias that are not locked taken out, and the teardrops
  that were there (as KiCad's import of a session does), and this routing put
  in instead;
- each zone's fill taken out, since it was made round the old tracks (KiCad
  fills the zones again with B, and its check does with ``--refill-zones``);
- the teardrops, as one of ``TEARDROP_STYLES``: KiCad teardrop zones of the
  router's shapes ("zones"), fans of tracks as in the session ("tracks"), or
  none ("none": KiCad's own Edit > Edit Teardrops can add them; it cannot be
  asked to from outside its PCB editor, and no pad or via is marked for them,
  so that KiCad does not add them on its own). A taper's wedge stays a fan of
  tracks: as a KiCad teardrop zone between two widths (``track_end``) KiCad's
  check found the narrow piece unconnected;
- the thermal reliefs the routing changed to join a pad to its pour
  (``plan.fanout``, with ``--change-reliefs`` or ``--narrow-reliefs``), as
  that pad's own zone connection, spoke angle, relief gap and spoke width. A
  session cannot carry them.

Beside it go copies of the project (``.kicad_pro``) and custom rules
(``.kicad_dru``), so that KiCad checks it by the board's own rules and
severities. The original files are only read. Coordinates: the DSN's,
millimetres with y turned over, as KiCad's export writes them.
"""
import math
import os
import re
import uuid

from weaveengine.io.kicad import CONNECT, _place
from weaveengine.io.ses import routed_copper

TEARDROP_STYLES = {
    "tracks": "fans of tracks, in the session and the routed board",
    "zones": "KiCad teardrop zones, in the routed board (the session gets none)",
    "none": "none (KiCad's own Edit > Edit Teardrops can add them)",
}
ZONE_PRIORITY = 30000  # (KiCad's own teardrops sit above the board's pours)


def routed_board_path(path: str) -> str:
    """Where the routed copy of the board of ``path`` (a DSN or SES) goes by default: NAME-routed.kicad_pcb beside it."""
    return os.path.splitext(path)[0] + "-routed.kicad_pcb"


def source_board(dsn_path: str) -> str | None:
    """The ``.kicad_pcb`` the DSN was exported from (same name, beside it), if there is one."""
    pcb = os.path.splitext(dsn_path)[0] + ".kicad_pcb"
    return pcb if os.path.exists(pcb) else None


def _lists(text: str, start: int, end: int) -> list[tuple[str, int, int]]:
    """The lists directly inside ``text[start:end]`` (which is inside one list):
    (first word, start, end) of each, ``end`` past its closing bracket."""
    out, depth, i, opened, n = [], 0, start, 0, end
    while i < n:
        c = text[i]
        if c == '"':
            i += 1
            while i < n and text[i] != '"':
                i += 2 if text[i] == "\\" else 1
        elif c == "(":
            if depth == 0:
                opened = i
            depth += 1
        elif c == ")":
            depth -= 1
            if depth == 0:
                m = re.match(r"\(\s*([^\s()\"]+)", text[opened:opened + 64])
                out.append((m[1] if m else "", opened, i + 1))
            elif depth < 0:
                break
        i += 1
    return out


def _quote(s: str) -> str:
    return '"' + s.replace("\\", "\\\\").replace('"', '\\"') + '"'


def write_routed_board(design, result, source: str, path: str, teardrops: str = "zones", arcs: bool = True) -> dict:
    """Writes the routed copy of ``source`` to ``path`` (see the module); returns what it holds, counted
    (tracks, arcs, vias, teardrop zones, and the tracks, arcs, vias and teardrops of the original taken out)."""
    if os.path.abspath(path) == os.path.abspath(source):
        raise ValueError(f"the routed copy would overwrite the board itself ({source}): give it another name")
    if teardrops not in TEARDROP_STYLES:
        raise ValueError(f"teardrop style {teardrops!r}: one of {', '.join(TEARDROP_STYLES)}")
    text = open(source, encoding="utf-8").read()
    root = _lists(text, 0, len(text))
    if not root or root[0][0] != "kicad_pcb":
        raise ValueError(f"{source} is not a KiCad board")
    _, top_start, top_end = root[0]
    top = _lists(text, top_start + 1, top_end - 1)
    # Layer names: the DSN's (a layer's user name, if it has one) -> KiCad's own.
    layer_name = {}
    for kind, s, e in top:
        if kind == "layers":
            for _, ls, le in _lists(text, s + 1, e - 1):
                words = re.findall(r'"((?:[^"\\]|\\.)*)"|(\S+)', text[ls + 1:le - 1])
                words = [a or b for a, b in words]
                if len(words) >= 3:
                    layer_name[words[1]] = words[1]
                    if len(words) >= 4:
                        layer_name[words[3]] = words[1]
    # Nets: by name (KiCad 10), or by number where the board lists them (KiCad 9 and before).
    numbers = {}
    for kind, s, e in top:
        if kind == "net":
            m = re.match(r'\(net\s+(\d+)\s+"((?:[^"\\]|\\.)*)"', text[s:e])
            if m:
                numbers[m[2].replace('\\"', '"')] = int(m[1])

    def net_ref(name: str) -> str:
        if numbers:
            return f"(net {numbers.get(name, 0)})"
        return f"(net {_quote(name)})"

    board = result.board
    names = design.net_names
    counts = dict(tracks=0, arcs=0, vias=0, zones=0, removed=0, reliefs=0)
    cut = []  # (start, end, what goes there instead)
    changed = _reliefs(board, result.stats.get("reliefs") or {})
    for kind, s, e in top:
        body = text[s:e]
        if kind == "footprint" and changed:
            for start, end, new in _relieved(text, s, e, changed):
                cut.append((start, end, new))
                counts["reliefs"] += 1
        if kind in ("segment", "arc", "via") and not re.search(r"\(locked\s+yes\)|^\(\w+\s+locked\b", body):
            cut.append((s, e, ""))
            counts["removed"] += 1
        elif kind == "zone":
            if re.search(r"\(attr\s*\(teardrop", body):
                cut.append((s, e, ""))
                counts["removed"] += 1
                continue
            for k2, s2, e2 in _lists(text, s + 1, e - 1):
                if k2 in ("filled_polygon", "fill_segments"):
                    cut.append((s2, e2, ""))

    def mm(p):
        return f"{p[0]:.6f} {-p[1]:.6f}"

    def layer_of(index):
        dsn = board.layers[index]
        return _quote(layer_name.get(dsn, dsn))

    copper = [layer_name.get(n, n) for n in board.layers + board.unrouted]
    outer = [n for n in ("F.Cu", "B.Cu") if n in copper] or copper[:1] + copper[-1:]
    via_layers = f'{_quote(outer[0])} {_quote(outer[-1])}'
    added = []
    for net, items in routed_copper(design, result, teardrops == "tracks", arcs, kept=teardrops != "none"):
        name = names.get(net, f"N{net}")
        for item in items:
            if item[0] == "via":
                added.append(f"\t(via\n\t\t(at {mm(item[1:])})\n\t\t(size {board.rules.via_diameter:g})\n\t\t(drill {board.rules.via_drill:g})\n"
                             f"\t\t(layers {via_layers})\n"
                             + f"\t\t{net_ref(name)}\n\t\t(uuid {_quote(str(uuid.uuid4()))})\n\t)")
                counts["vias"] += 1
                continue
            _, layer, width = item[:3]
            if item[0] == "arc":
                a, b, c = item[3:]
                r = math.dist(a, c)
                a1, a2 = math.atan2(a[1] - c[1], a[0] - c[0]), math.atan2(b[1] - c[1], b[0] - c[0])
                mid = a1 + ((a2 - a1 + math.pi) % (2.0 * math.pi) - math.pi) / 2.0
                if math.dist(a, b) < 1e-6:
                    continue
                added.append(f"\t(arc\n\t\t(start {mm(a)})\n\t\t(mid {mm((c[0] + r * math.cos(mid), c[1] + r * math.sin(mid)))})\n"
                             f"\t\t(end {mm(b)})\n\t\t(width {width:g})\n\t\t(layer {layer_of(layer)})\n\t\t{net_ref(name)}\n"
                             f"\t\t(uuid {_quote(str(uuid.uuid4()))})\n\t)")
                counts["arcs"] += 1
                continue
            pts = [p for i, p in enumerate(item[3]) if i == 0 or math.dist(p, item[3][i - 1]) > 1e-6]
            for a, b in zip(pts, pts[1:]):
                added.append(f"\t(segment\n\t\t(start {mm(a)})\n\t\t(end {mm(b)})\n\t\t(width {width:g})\n\t\t(layer {layer_of(layer)})\n"
                             f"\t\t{net_ref(name)}\n\t\t(uuid {_quote(str(uuid.uuid4()))})\n\t)")
                counts["tracks"] += 1
    if teardrops == "zones":
        for wire, polys in sorted(result.teardrops.items()):
            name, layer = names.get(result.wire_net[wire], ""), layer_of(result.wire_layer.get(wire, 0))
            for poly in polys:
                added.append(_zone(net_ref(name), layer, poly[:5], "padvia"))
                counts["zones"] += 1
    cut.append((top_end - 1, top_end - 1, "".join(item + "\n" for item in added)))
    out, at = [], 0
    for s, e, new in sorted(cut):
        out.append(text[at:s])
        out.append(new)
        at = e
    out.append(text[at:])
    result_text = "".join(out)
    result_text = re.sub(r"\n[ \t]*\n(?=[ \t]*\n)", "\n", result_text)  # (no runs of blank lines where items were taken out)
    part = f"{path}.{os.getpid()}.part"
    try:
        with open(part, "w", encoding="utf-8") as f:
            f.write(result_text)
        os.replace(part, path)
    except BaseException:
        try:
            os.remove(part)
        except OSError:
            pass
        raise
    for ext in (".kicad_pro", ".kicad_dru"):
        given = os.path.splitext(source)[0] + ext
        if os.path.exists(given):
            with open(given, "rb") as f:
                data = f.read()
            with open(os.path.splitext(path)[0] + ext, "wb") as f:
                f.write(data)
    return counts


# A pad's thermal relief, as ``Pad`` holds it -> as KiCad writes it in the pad.
RELIEF_TOKENS = {"zone_connect": "zone_connect", "spoke_angle": "thermal_bridge_angle", "thermal_gap": "thermal_gap",
                 "spoke": "thermal_bridge_width"}


def _reliefs(board, reliefs: dict) -> dict:
    """(reference, pad number) -> [(centre, {KiCad token: value})] of the pads whose thermal relief was changed."""
    out: dict = {}
    pads = {p.pad_id: p for p in board.pads}
    numbers = {v: k for k, v in CONNECT.items()}
    for pad_id, change in sorted(reliefs.items()):
        pad = pads.get(pad_id)
        if pad is None or not pad.name:
            continue
        ref, _, number = pad.name.split("@")[0].rpartition("-")
        tokens = {RELIEF_TOKENS[k]: (numbers[v] if k == "zone_connect" else f"{v:g}") for k, v in change.items() if k in RELIEF_TOKENS}
        out.setdefault((ref, number), []).append((pad.centre, tokens))
    return out


def _relieved(text: str, start: int, end: int, changed: dict) -> list[tuple[int, int, str]]:
    """The pads of the footprint ``text[start:end]`` whose relief was changed (``_reliefs``), each written again with
    its own zone connection, spoke angle, gap and spoke width: (start, end, the pad's new text)."""
    body = text[start:end]
    m = re.search(r'\(property\s+"Reference"\s+"((?:[^"\\]|\\.)*)"', body) or re.search(r'\(fp_text\s+reference\s+"?((?:[^"\\\s)]|\\.)*)"?', body)
    ref = m[1] if m else ""
    wanted = {number: got for (r, number), got in changed.items() if r == ref}
    if not wanted:
        return []
    inner = _lists(text, start + 1, end - 1)
    at = next((text[s:e] for k, s, e in inner if k == "at"), "(at 0 0)")
    fx, fy, angle = ([float(v) for v in re.findall(r"-?[\d.]+(?:e-?\d+)?", at)] + [0.0, 0.0, 0.0])[:3]
    place = _place(fx, fy, angle)
    pads = []
    for k, s, e in inner:
        m = re.match(r'\(pad\s+"?((?:[^"\\\s]|\\.)*)"?', text[s:e]) if k == "pad" else None
        if m and m[1] in wanted:
            pad_at = next((text[s2:e2] for k2, s2, e2 in _lists(text, s + 1, e - 1) if k2 == "at"), "(at 0 0)")
            u, v = ([float(x) for x in re.findall(r"-?[\d.]+(?:e-?\d+)?", pad_at)] + [0.0, 0.0])[:2]
            pads.append((m[1], place(u, v), s, e))
    out = []
    for number, got in wanted.items():
        for centre, tokens in got:
            mine = [p for p in pads if p[0] == number]
            if not mine:
                continue
            _, _, s, e = min(mine, key=lambda p: math.dist(p[1], centre))
            old = text[s:e]
            for token in tokens:  # (the pad's own setting, if it had one, goes: KiCad reads one of each)
                old = re.sub(r"\s*\(" + token + r"\s+[^()]*\)", "", old)
            if "thermal_bridge_width" in tokens:
                old = re.sub(r"\s*\(thermal_width\s+[^()]*\)", "", old)
            indent = re.search(r"\n([ \t]*)\(", old)
            pad_indent = indent[1] if indent else "\t\t\t"
            new = old[:-1].rstrip() + "".join(f"\n{pad_indent}({token} {value})" for token, value in tokens.items()) + "\n" + pad_indent[:-1] + ")"
            out.append((s, e, new))
    return out


def _zone(net: str, layer: str, poly, kind: str) -> str:
    """A KiCad teardrop zone of ``poly`` (DSN frame)."""
    pts = [p for i, p in enumerate(poly) if i == 0 or math.dist(p, poly[i - 1]) > 1e-6]
    xy = " ".join(f"(xy {x:.6f} {-y:.6f})" for x, y in pts)
    return (f"\t(zone\n\t\t{net}\n\t\t(layer {layer})\n\t\t(uuid {_quote(str(uuid.uuid4()))})\n\t\t(name \"$teardrop_{'padvia' if kind == 'padvia' else 'track'}$\")\n"
            f"\t\t(hatch full 0.1)\n\t\t(priority {ZONE_PRIORITY})\n\t\t(attr\n\t\t\t(teardrop\n\t\t\t\t(type {kind})\n\t\t\t)\n\t\t)\n"
            f"\t\t(connect_pads yes\n\t\t\t(clearance 0)\n\t\t)\n\t\t(min_thickness 0.0254)\n\t\t(filled_areas_thickness no)\n"
            f"\t\t(fill yes\n\t\t\t(thermal_gap 0.5)\n\t\t\t(thermal_bridge_width 0.5)\n\t\t\t(island_removal_mode 1)\n\t\t\t(island_area_min 10)\n\t\t)\n"
            f"\t\t(polygon\n\t\t\t(pts\n\t\t\t\t{xy}\n\t\t\t)\n\t\t)\n\t)")
