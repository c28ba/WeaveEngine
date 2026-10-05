import pytest

from bench.generators import channel
from weaveengine.cli import main
from weaveengine.io.dsn import parse_sexpr, read_dsn, write_dsn

KICAD_STYLE = """(pcb demo.dsn
  (parser (string_quote ") (space_in_quoted_tokens on) (host_cad "KiCad's Pcbnew"))
  (resolution um 10)
  (unit um)
  (structure
    (layer F.Cu (type signal) (property (index 0)))
    (layer B.Cu (type signal) (property (index 1)))
    (boundary (path pcb 0  0 0  20000 0  20000 -10000  0 -10000  0 0))
    (keepout "" (rect F.Cu 9000 -6000 11000 -4000))
    (via "Via[0-1]_800:400_um")
    (rule (width 250) (clearance 200.1) (clearance 50 (type smd_smd)))
  )
  (placement
    (component R_0805 (place R1 5000 -5000 front 0 (PN 10k)) (place R2 15000 -5000 front 90))
    (component TP (place TP1 10000 -8000 back 0))
  )
  (library
    (image R_0805 (pin Rect[T]Pad_1000x1300_um 1 -900 0) (pin Rect[T]Pad_1000x1300_um 2 900 0))
    (image TP (pin Round[A]Pad_1500_um 1 0 0))
    (padstack Round[A]Pad_1500_um (shape (circle F.Cu 1500)) (shape (circle B.Cu 1500)) (attach off))
    (padstack Rect[T]Pad_1000x1300_um (shape (rect F.Cu -500 -650 500 650)) (attach off))
  )
  (network
    (net "Net-(R1-Pad2)" (pins R1-2 R2-1))
    (net GND (pins R1-1 TP1-1 R2-2))
    (class kicad_default "" GND "Net-(R1-Pad2)" (circuit (use_via Via[0-1]_800:400_um)) (rule (width 250) (clearance 200.1)))
  )
  (wiring)
)
"""


def test_sexpr_reader():
    assert parse_sexpr('(a (b "c d" 1) e)') == ["a", ["b", "c d", "1"], "e"]


def test_read_kicad_style_dsn(tmp_path):
    path = tmp_path / "demo.dsn"
    path.write_text(KICAD_STYLE)
    design = read_dsn(str(path))
    board = design.board
    assert board.layers == ["F.Cu", "B.Cu"]
    assert design.via_padstack == "Via[0-1]_800:400_um" or design.via_padstack == ""
    assert abs(board.rules.trace_width - 0.25) < 1e-9 and abs(board.rules.clearance - 0.2001) < 1e-9
    assert board.outline.bounds == (0.0, -10.0, 20.0, 0.0)
    pads = {p.name: p for p in board.pads}
    assert set(pads) == {"R1-1", "R1-2", "R2-1", "R2-2", "TP1-1"}
    assert [round(c, 3) for c in pads["R1-1"].centre] == [4.1, -5.0]
    assert [round(c, 3) for c in pads["R2-2"].centre] == [15.0, -4.1]  # rotated 90 degrees
    xmin, ymin, xmax, ymax = pads["R2-2"].shape.bounds
    assert round(xmax - xmin, 3) == 1.3 and round(ymax - ymin, 3) == 1.0
    assert pads["R1-2"].net_id == pads["R2-1"].net_id == design.net_ids["Net-(R1-Pad2)"]
    assert len(board.nets()[design.net_ids["GND"]]) == 3
    assert len(board.obstacles) == 1
    # SMD pads are on the front only; the through-hole test point (placed on the back) is on both.
    assert pads["R1-1"].layers == frozenset({0}) and pads["TP1-1"].layers is None
    assert pads["TP1-1"].radius == 0.75 and pads["R1-1"].radius is None
    back = read_dsn(str(path), layers=["B.Cu"])
    assert {p.name for p in back.board.pads} == {"TP1-1"}


def test_dsn_round_trip(tmp_path):
    board = channel(6, seed=8)
    board.add_keepout([(1, 1), (2, 1), (2, 2), (1, 2)])
    path = str(tmp_path / "board.dsn")
    write_dsn(board, path)
    again = read_dsn(path).board
    assert abs(again.rules.trace_width - board.rules.trace_width) < 1e-9
    assert abs(again.outline.symmetric_difference(board.outline).area) < 1e-6
    assert len(again.obstacles) == 1
    by_name = {p.name: p for p in again.pads}
    for pad in board.pads:
        twin = by_name[f"P{pad.pad_id}-1"]
        assert pad.shape.symmetric_difference(twin.shape).area < 1e-6
    partner = lambda b, key: {frozenset(key(b, p) for p in pads) for pads in b.nets().values()}
    original = partner(board, lambda b, p: f"P{p}-1")
    reread = partner(again, lambda b, p: next(q.name for q in b.pads if q.pad_id == p))
    assert original == reread


def test_cli_writes_ses_and_svg(tmp_path, capsys):
    dsn, ses, svg = (str(tmp_path / n) for n in ("b.dsn", "b.ses", "b.svg"))
    write_dsn(channel(4, seed=1), dsn)
    assert main([dsn, "-o", ses, "--svg", svg]) == 0
    assert "4/4 connections routed" in capsys.readouterr().out
    session = parse_sexpr(open(ses).read())
    assert session[0] == "session"
    routes = next(c for c in session if isinstance(c, list) and c[0] == "routes")
    nets = [c for c in next(c for c in routes if isinstance(c, list) and c[0] == "network_out") if isinstance(c, list)]
    assert len(nets) == 4
    for net in nets:
        path = net[2][1]
        assert path[0] == "path" and path[1] == "F.Cu" and int(path[2]) == 2000  # 0.2 mm at 0.1 um
        assert len(path[3:]) % 2 == 0 and all(float(v) == int(v) for v in path[3:])
    assert "<svg" in open(svg).read()


def test_joined_quoted_pin_references():
    """KiCad writes a pin of a reference containing '-' as "Out-I"-1."""
    assert parse_sexpr('(pins "Out-I"-1 "D6-11"-2 D7-1 "/In 4")') == ["pins", "Out-I-1", "D6-11-2", "D7-1", "/In 4"]


def test_net_class_widths_and_typed_clearance(tmp_path):
    text = KICAD_STYLE.replace('(wiring)', '(wiring)').replace(
        '(class kicad_default "" GND "Net-(R1-Pad2)"', '(class kicad_default "" "Net-(R1-Pad2)"').replace(
        '  (wiring)', '    (class Power GND (rule (width 500) (clearance 200.1)))\n  (wiring)')
    # the extra class has to sit inside (network ...): move the closing parenthesis
    text = text.replace('(rule (width 250) (clearance 200.1)))\n  )\n    (class Power', '(rule (width 250) (clearance 200.1)))\n    (class Power').replace(
        '(clearance 200.1)))\n  (wiring)', '(clearance 200.1)))\n  )\n  (wiring)')
    path = tmp_path / "classes.dsn"
    path.write_text(text)
    design = read_dsn(str(path))
    rules = design.board.rules
    gnd = design.net_ids["GND"]
    assert rules.net_width == {gnd: 0.5}
    assert rules.width(gnd) == 0.5 and rules.width(design.net_ids["Net-(R1-Pad2)"]) == 0.25
    assert abs(rules.clearance - 0.2001) < 1e-9  # the 50 um smd_smd clearance is ignored
    assert rules.base_width == 0.25 and abs(rules.extra(gnd) - 0.125) < 1e-9


def test_session_measured_independently(tmp_path, capsys):
    """The written SES is read back and its copper measured against the rules."""
    from weaveengine.io.check import measure, read_session
    dsn, ses = str(tmp_path / "b.dsn"), str(tmp_path / "b.ses")
    write_dsn(channel(6, seed=8), dsn)
    assert main([dsn, "-o", ses, "--margin", "0", "--no-ses-teardrops"]) == 0
    assert "-> OK" in capsys.readouterr().out
    design = read_dsn(dsn)
    wires, vias = read_session(ses)
    assert len(wires) == 6 and vias == []
    m = measure(design, ses)
    assert m.ok and m.crossings == 0 and m.traces == 6
    assert 0.2 - 1.1e-3 <= min(m.track_to_pad, m.track_to_track) < 0.25  # tight, but never under the rule
    # A stricter rule than the board was routed for is reported as a violation.
    assert not measure(design, ses, clearance=0.3).ok
    assert not measure(design, ses, edge=5.0).ok
    # The default safety margin leaves every gap strictly above the rule.
    assert main([dsn, "-o", ses, "--edge-clearance", "0.5"]) == 0
    out = capsys.readouterr().out
    assert "-> OK" in out and "rule 0.5" in out
    m = measure(design, ses, edge=0.5)
    assert m.ok and min(m.track_to_pad, m.track_to_track) >= 0.2 + 0.009 and m.track_to_edge >= 0.5 + 0.009


def test_session_names_are_quoted(tmp_path):
    """References and nets may contain '-', the pin separator: write them quoted, like KiCad does."""
    dsn, ses = str(tmp_path / "b.dsn"), str(tmp_path / "b.ses")
    write_dsn(channel(4, seed=1), dsn)
    main([dsn, "-o", ses])
    text = open(ses).read()
    assert '(net "N0"' in text
    # No components are listed (nothing is ever moved), so the CAD tool has no reference to resolve.
    assert "(placement" in text and "(component" not in text and "(place " not in text
    for line in text.splitlines():
        if "(path " in line:
            pts = line.split("(path ")[1].rstrip(")").split()[2:]
            pairs = list(zip(pts[0::2], pts[1::2]))
            assert all(a != b for a, b in zip(pairs, pairs[1:]))  # no zero-length segments


def test_kicad_project_rules(tmp_path):
    from weaveengine.io.check import kicad_project_rules
    pro = tmp_path / "b.kicad_pro"
    pro.write_text('{"board": {"design_settings": {"rules": {"min_copper_edge_clearance": 0.5, "min_hole_clearance": 0.25}}}}')
    assert kicad_project_rules(str(pro)) == {"edge_clearance": 0.5, "hole_clearance": 0.25, "min_clearance": 0.0, "min_track_width": 0.0}
    dsn, ses = str(tmp_path / "b.dsn"), str(tmp_path / "b.ses")
    write_dsn(channel(4, seed=1), dsn)  # the project file next to the DSN is picked up
    main([dsn, "-o", ses])
    from weaveengine.io.check import measure
    assert measure(read_dsn(dsn), ses).track_to_edge >= 0.5


def test_components_outlines_and_values_are_read(tmp_path):
    """Part outlines, references and values come from the DSN, for display and selection."""
    text = KICAD_STYLE.replace(
        "(image R_0805 (pin", "(image R_0805 (outline (path signal 100 -1000 -500 1000 -500)) (outline (rect signal -1000 -500 1000 500)) (pin")
    path = tmp_path / "parts.dsn"
    path.write_text(text)
    board = read_dsn(str(path)).board
    parts = {c.reference: c for c in board.components}
    assert set(parts) == {"R1", "R2", "TP1"}
    r1, r2 = parts["R1"], parts["R2"]
    assert r1.value == "10k" and r2.value == "" and r1.footprint == "R_0805" and r1.side == "front"
    assert [board.pads[i].name for i in r1.pads] == ["R1-1", "R1-2"]
    assert [round(v, 3) for v in r1.bounds()] == [4.0, -5.5, 6.0, -4.5]          # 2 x 1 mm about (5, -5)
    assert [round(v, 3) for v in r2.bounds()] == [14.5, -6.0, 15.5, -4.0]        # the same, turned 90 degrees
    assert len(r1.outlines) == 2 and parts["TP1"].outlines == []


def _without(text: str, name: str) -> str:
    """``text`` with every (name ...) list removed."""
    import re
    out, i = [], 0
    while (found := re.compile(r"\(" + name + r"\s").search(text, i)) is not None:
        depth, j = 0, (start := found.start())
        while True:
            depth += {"(": 1, ")": -1}.get(text[j], 0)
            j += 1
            if depth == 0:
                break
        out.append(text[i:start])
        i = j
    return "".join(out) + text[i:]


def test_missing_rules_take_defaults(tmp_path):
    """A DSN need not carry rules (design section 4); each missing value is defaulted on its own and reported."""
    def load(text):
        path = tmp_path / "board.dsn"
        path.write_text(text)
        return read_dsn(str(path))

    # KICAD_STYLE names a via padstack its library does not define: the via size is defaulted.
    assert set(load(KICAD_STYLE).defaulted) == {"via_diameter", "via_drill"}
    complete = KICAD_STYLE.replace("    (padstack Round", '    (padstack "Via[0-1]_800:400_um" (shape (circle F.Cu 800)) '
                                   "(shape (circle B.Cu 800)) (attach off))\n    (padstack Round", 1)
    full = load(complete)
    assert not full.defaulted and (full.board.rules.via_diameter, full.board.rules.via_drill) == (0.8, 0.4)
    bare = load(_without(_without(KICAD_STYLE, "rule"), "via"))
    rules = bare.board.rules
    assert (rules.trace_width, rules.clearance, rules.via_diameter, rules.via_drill) == (0.2, 0.2, 0.6, 0.3)
    assert set(bare.defaulted) == {"trace_width", "clearance", "via_diameter", "via_drill"}
    assert len(bare.board.pads) == len(full.board.pads)
    width_only = load(_without(complete, "clearance"))
    assert set(width_only.defaulted) == {"clearance"} and width_only.board.rules.trace_width == full.board.rules.trace_width
    clearance_only = load(_without(complete, "width"))
    assert set(clearance_only.defaulted) == {"trace_width"}
    assert clearance_only.board.rules.clearance == full.board.rules.clearance
    # A setting the user made is not reported as defaulted.
    from weaveengine.settings import Settings
    notes = Settings(trace_width=0.3).apply_rules(bare)[2]
    said = next(n for n in notes if n.startswith("not in the DSN"))
    assert "clearance 0.2 mm" in said and "trace width" not in said and bare.board.rules.trace_width == 0.3
    # What cannot be assumed is still refused, readably.
    for name in ("boundary", "layer", "structure"):
        with pytest.raises(ValueError, match="DSN has no"):
            load(_without(KICAD_STYLE, name))


def test_cli_routes_a_dsn_without_rules(tmp_path, capsys):
    from tests.conftest import demo_board
    dsn = tmp_path / "bare.dsn"
    write_dsn(demo_board(), str(dsn))
    dsn.write_text(_without(dsn.read_text(), "rule"))
    assert main([str(dsn), "--quiet", "--workers", "1"]) == 0
    out = capsys.readouterr().out
    assert "not in the DSN, defaults used: trace width 0.2 mm, clearance 0.2 mm" in out and "-> OK" in out
