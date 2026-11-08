import math

import pytest
from shapely.geometry import Point

from bench.generators import channel
from weaveengine.cli import main
from weaveengine.io.dsn import parse_sexpr, read_dsn, write_dsn
from weaveengine.plan.context import Options

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


def test_a_layer_a_keepout_covers_whole_is_not_routed(tmp_path):
    """A single-sided board as KiCad draws one: two copper layers, a rule area
    forbidding tracks over all of the front. The front is left out (it has no
    free space at all), and the board is routed on the back."""
    path = tmp_path / "single.dsn"
    path.write_text(KICAD_STYLE.replace('(keepout "" (rect F.Cu 9000 -6000 11000 -4000))',
                                        '(keepout "" (rect F.Cu -1000 1000 21000 -11000))'), encoding="utf-8")
    design = read_dsn(str(path))
    assert design.board.layers == ["B.Cu"] and design.walled == ["F.Cu"]
    assert {p.name for p in design.board.pads} == {"TP1-1"}
    assert read_dsn(str(path), layers=["F.Cu", "B.Cu"]).walled == []   # layers named: as asked
    # A keepout over part of a layer leaves it routed.
    path.write_text(KICAD_STYLE, encoding="utf-8")
    assert read_dsn(str(path)).walled == []


def test_sexpr_reader():
    assert parse_sexpr('(a (b "c d" 1) e)') == ["a", ["b", "c d", "1"], "e"]


def test_read_kicad_style_dsn(tmp_path):
    path = tmp_path / "demo.dsn"
    path.write_text(KICAD_STYLE, encoding="utf-8")
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
    session = parse_sexpr(open(ses, encoding="utf-8").read())
    assert session[0] == "session"
    routes = next(c for c in session if isinstance(c, list) and c[0] == "routes")
    nets = [c for c in next(c for c in routes if isinstance(c, list) and c[0] == "network_out") if isinstance(c, list)]
    assert len(nets) == 4
    for net in nets:
        path = net[2][1]
        assert path[0] == "path" and path[1] == "F.Cu" and int(path[2]) == 2000  # 0.2 mm at 0.1 um
        assert len(path[3:]) % 2 == 0 and all(float(v) == int(v) for v in path[3:])
    assert "<svg" in open(svg, encoding="utf-8").read()


def test_cli_caps_the_rip_up_rounds(tmp_path, monkeypatch):
    """--max-rounds sets the rip-up round limit as the app's setting does: the
    parameters scaled to the board, the limit the only change. Without it the
    router makes its own, and goes on for more rounds where a board needs them."""
    from bench.generators import random_grid
    from weaveengine import cli
    from weaveengine.topo import planar_map
    from weaveengine.topo.costs import CostParams
    given, rounds, real = [], [], cli.route_board

    def route_board(board, params=None, **how):
        given.append(params)
        result = real(board, params, **how)
        rounds.append(result.stats["ripup_rounds"])
        return result
    monkeypatch.setattr(cli, "route_board", route_board)
    dsn = str(tmp_path / "b.dsn")
    write_dsn(random_grid(8, 8, nets=14, seed=4), dsn)
    main([dsn, "--quiet", "--workers", "1", "--portfolio", "1", "--max-rounds", "2"])
    main([dsn, "--quiet", "--workers", "1", "--portfolio", "1"])
    assert given == [CostParams.for_map(planar_map.build(read_dsn(dsn).board), max_rounds=2), None]
    assert rounds[0] == 2 < rounds[1]
    with pytest.raises(SystemExit):
        main([dsn, "--max-rounds", "-1"])


def test_a_net_name_beyond_ascii_comes_back_as_written(tmp_path):
    """KiCad writes UTF-8 (net names like "I_5µA" or "Ω_SENSE" are common). Read
    and written as UTF-8 whatever the platform's own encoding (cp1252 on
    Windows), the name reaches the SES as it was."""
    from weaveengine.io.ses import write_ses
    from weaveengine.router import route_board
    path, ses = tmp_path / "b.dsn", tmp_path / "b.ses"
    path.write_text(KICAD_STYLE.replace('"Net-(R1-Pad2)"', '"I_5µA→Ω"'), encoding="utf-8")
    design = read_dsn(str(path))
    assert "I_5µA→Ω" in design.net_ids
    write_ses(design, route_board(design.board, options=Options(portfolio=1), workers=1), str(ses))
    assert "I_5µA→Ω" in ses.read_text(encoding="utf-8")


def test_a_pour_is_left_to_itself_and_a_teardrop_is_not(tmp_path):
    """KiCad writes every zone as a plane, teardrops too (zones since KiCad 7).
    A net is left to its pour only if the plane spreads over two of its pads and
    is larger than they are; a teardrop at one pad leaves the net to be routed."""
    teardrop = '(plane GND (polygon F.Cu 0  4500 -5600  5200 -5000  4500 -4400))'
    pour = '(plane GND (polygon B.Cu 0  0 0  20000 0  20000 -10000  0 -10000))'
    found = {}
    for name, planes in (("teardrop", teardrop), ("pour", pour), ("both", teardrop + " " + pour)):
        path = tmp_path / f"{name}.dsn"
        path.write_text(KICAD_STYLE.replace('    (via "Via[0-1]_800:400_um")', f'    {planes}\n    (via "Via[0-1]_800:400_um")'), encoding="utf-8")
        design = read_dsn(str(path))
        found[name] = design.net_ids["GND"] in design.board.plane_nets
    assert found == {"teardrop": False, "pour": True, "both": True}


def test_fixed_wiring_is_copper_of_its_net(tmp_path):
    """Wiring marked fixed (locked in KiCad) is kept as copper of its net, a via
    as a via (``Obstacle.via``: ``plan.fanout`` keeps its drill clear, and takes
    it as a way to the plane); the rest is discarded and routed afresh."""
    path = tmp_path / "fixed.dsn"
    path.write_text(KICAD_STYLE.replace("  )\n  (network", '    (padstack "Via[0-1]_800:400_um" (shape (circle F.Cu 800)) '
                                        "(shape (circle B.Cu 800)) (attach off))\n  )\n  (network").replace(
        "  (wiring)", '  (wiring\n    (wire (path F.Cu 250  5900 -5000  8000 -5000)(net GND)(type fix))\n'
                      '    (via "Via[0-1]_800:400_um" 8000 -5000 (net GND)(type fix))\n'
                      '    (via "Via[0-1]_800:400_um" 12000 -8000 (net GND))\n  )'), encoding="utf-8")
    design = read_dsn(str(path))
    fixed = [o for o in design.board.obstacles if o.net_id == design.net_ids["GND"]]
    assert sorted((o.via, o.layers) for o in fixed) == [(False, frozenset({0})), (True, None)]
    assert [round(c, 3) for c in next(o for o in fixed if o.via).shape.centroid.coords[0]] == [8.0, -5.0]
    assert design.dropped_wiring.startswith("1 existing traces and vias that are not marked fixed")


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
    path.write_text(text, encoding="utf-8")
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
    text = open(ses, encoding="utf-8").read()
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
    pro.write_text('{"board": {"design_settings": {"rules": {"min_copper_edge_clearance": 0.5, "min_hole_clearance": 0.25}}}}', encoding="utf-8")
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
    path.write_text(text, encoding="utf-8")
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
        path.write_text(text, encoding="utf-8")
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
    said = [n for n in notes if " not detected: " in n]
    assert "WARNING: clearance not detected: the built-in 0.2 mm is used" in said
    assert not any("trace width" in n for n in said) and bare.board.rules.trace_width == 0.3
    # What cannot be assumed is still refused, readably.
    for name in ("boundary", "layer", "structure"):
        with pytest.raises(ValueError, match="DSN has no"):
            load(_without(KICAD_STYLE, name))


def test_cli_routes_a_dsn_without_rules(tmp_path, capsys):
    from tests.conftest import demo_board
    dsn = tmp_path / "bare.dsn"
    write_dsn(demo_board(), str(dsn))
    dsn.write_text(_without(dsn.read_text(encoding="utf-8"), "rule"))
    assert main([str(dsn), "--quiet", "--workers", "1"]) == 0
    out = capsys.readouterr().out
    assert "not in the DSN, defaults used: trace width 0.2 mm, clearance 0.2 mm" in out and "-> OK" in out


KICAD_CUTOUT = """(pcb cut.dsn
  (resolution um 10)
  (unit um)
  (structure
    (layer F.Cu (type signal)) (layer B.Cu (type signal))
    (boundary (path pcb 0  0 0  30000 0  30000 -20000  0 -20000  0 0))
    (keepout "" (polygon signal 0  13000 -8000  17000 -8000  17000 -12000  13000 -12000  13000 -8000))
    (rule (width 200) (clearance 200))
  )
  (placement
    (component J (place J1 5000 -10000 front 0) (place J2 25000 -10000 back 90))
  )
  (library
    (image J (pin Oval[A]Pad_1700x1700_um 1 0 0) (pin Oval[A]Pad_1700x1700_um 2 0 -2540)
      (keepout "" (circle F.Cu 1600 2540 0)) (keepout "" (circle B.Cu 1600 2540 0)))
    (padstack Oval[A]Pad_1700x1700_um (shape (path F.Cu 1700 0 0 0 0)) (shape (path B.Cu 1700 0 0 0 0)) (attach off))
  )
  (network (net A (pins J1-1 J2-1)) (net B (pins J1-2 J2-2)))
  (wiring)
)
"""


def test_cutouts_holes_and_round_paths(tmp_path):
    """KiCad writes a cutout in the board as a keepout on every layer
    ("signal"), a hole without copper as a keepout of the part on each
    layer, and a round pad as an oval of equal sides (a path of no length)."""
    from shapely.geometry import Point
    path = tmp_path / "cut.dsn"
    path.write_text(KICAD_CUTOUT, encoding="utf-8")
    board = read_dsn(str(path)).board
    assert len(board.outline.interiors) == 1 and abs(board.outline.area - (600 - 16)) < 1e-6   # board edge, not a keepout
    holes = sorted((round(o.shape.centroid.x, 3), round(o.shape.centroid.y, 3), sorted(o.layers)) for o in board.obstacles)
    assert holes == [(7.54, -10.0, [0]), (7.54, -10.0, [1]), (25.0, -12.54, [0]), (25.0, -12.54, [1])]   # placed, rotated, flipped
    pad = next(p for p in board.pads if p.name == "J1-1")
    assert pad.radius == 0.85 and pad.shape.buffer(1e-9).contains(Point(pad.centre).buffer(0.85, quad_segs=64))
    assert write_dsn(board, str(tmp_path / "again.dsn")) is None
    assert len(read_dsn(str(tmp_path / "again.dsn")).board.outline.interiors) == 1


# Rule areas as KiCad 9.0.8 writes them: forbidding tracks and vias, vias only, tracks only; and a part's, vias only.
KICAD_RULE_AREAS = """(pcb areas.dsn
  (resolution um 10)
  (unit um)
  (structure
    (layer F.Cu (type signal)) (layer B.Cu (type signal))
    (boundary (path pcb 0  0 0  40000 0  40000 -20000  0 -20000  0 0))
    (keepout "" (polygon F.Cu 0  2000 -5000  8000 -5000  8000 -15000  2000 -15000  2000 -5000))
    (via_keepout "" (polygon F.Cu 0  12000 -5000  18000 -5000  18000 -15000  12000 -15000  12000 -5000))
    (via_keepout "" (polygon B.Cu 0  12000 -5000  18000 -5000  18000 -15000  12000 -15000  12000 -5000))
    (wire_keepout "" (polygon F.Cu 0  22000 -5000  28000 -5000  28000 -15000  22000 -15000  22000 -5000))
    (rule (width 200) (clearance 200))
  )
  (placement
    (component J (place J1 34000 -10000 back 90))
  )
  (library
    (image J (pin Round[A]Pad_1500_um 1 0 0) (pin Round[A]Pad_1500_um 2 0 -2540)
      (via_keepout "" (polygon F.Cu 0  -1000 1000  3000 1000  3000 -3540  -1000 -3540  -1000 1000)))
    (padstack Round[A]Pad_1500_um (shape (circle F.Cu 1500)) (shape (circle B.Cu 1500)) (attach off))
  )
  (network (net A (pins J1-1 J1-2)))
  (wiring)
)
"""


def test_rule_areas_that_forbid_tracks_or_vias_only(tmp_path):
    """KiCad writes a rule area that forbids tracks and vias as a keepout, one
    that forbids tracks only as a wire_keepout, and one that forbids vias only
    as a via_keepout, on the board or in a part's image. The first two keep
    traces out; the third keeps vias out, whatever its layer (``Board.no_via``)."""
    path = tmp_path / "areas.dsn"
    path.write_text(KICAD_RULE_AREAS, encoding="utf-8")
    board = read_dsn(str(path)).board
    assert sorted((o.shape.bounds, o.layers) for o in board.obstacles) == [((2.0, -15.0, 8.0, -5.0), frozenset({0})),
                                                                           ((22.0, -15.0, 28.0, -5.0), frozenset({0}))]
    assert len(board.no_via) == 3 and [area.bounds for area in board.no_via[:2]] == [(12.0, -15.0, 18.0, -5.0)] * 2
    assert [round(v, 6) for v in board.no_via[2].bounds] == [33.0, -13.0, 37.54, -9.0]   # placed with its part: flipped, turned
    assert len(read_dsn(str(path), layers=["B.Cu"]).board.no_via) == 3   # a via goes through layers that are not routed too
    write_dsn(board, str(tmp_path / "again.dsn"))
    assert [a.bounds for a in read_dsn(str(tmp_path / "again.dsn")).board.no_via] == [a.bounds for a in board.no_via]
    # A rule area beside it in the KiCad board that lets tracks through (it forbids vias only) does not open a wire_keepout of its bounds.
    (tmp_path / "areas.kicad_pcb").write_text("""(kicad_pcb (version 20241229)
  (zone (net 0) (net_name "") (layer "F.Cu") (name "no vias") (hatch edge 0.5)
    (keepout (tracks allowed) (vias not_allowed) (pads allowed) (copperpour allowed) (footprints allowed))
    (polygon (pts (xy 22 5) (xy 28 5) (xy 28 15) (xy 22 15)))))
""", encoding="utf-8")
    assert len(read_dsn(str(path)).board.obstacles) == 2
    # A wire_keepout over all of a layer is a single-sided board's, as a keepout is.
    path.write_text(KICAD_STYLE.replace('(keepout "" (rect F.Cu 9000 -6000 11000 -4000))',
                                        '(wire_keepout "" (rect F.Cu -1000 1000 21000 -11000))'), encoding="utf-8")
    assert read_dsn(str(path)).walled == ["F.Cu"]


def test_rules_the_board_gives_and_does_not(tmp_path):
    """The settings editor shows what the board gives for each rule, and warns
    about what it does not: a rule missing from the DSN, and the edge
    clearance, which no DSN carries, unless a KiCad project is beside it."""
    import json
    from weaveengine.settings import Settings, detected_rules, undetected
    path = tmp_path / "demo.dsn"
    path.write_text(KICAD_STYLE.replace("(rule (width 250) (clearance 200.1) (clearance 50 (type smd_smd)))", ""), encoding="utf-8")
    design = read_dsn(str(path))
    found = detected_rules(design, str(path))
    assert found["trace_width"] == (0.25, "from the DSN")   # from the net class
    assert found["edge_clearance"][0] is None and "no demo.kicad_pro" in found["edge_clearance"][1]
    # (its via padstack is not in the library, so the via is not given either)
    assert [line.split(" not detected")[0] for line in undetected(found)] == ["via diameter", "via drill", "copper to board edge clearance"]
    assert undetected(found, Settings(edge_clearance=0.4, via_diameter=0.6, via_drill=0.3)) == []   # set by hand: nothing to warn about
    (tmp_path / "demo.kicad_pro").write_text(json.dumps({"board": {"design_settings": {"rules": {"min_copper_edge_clearance": 0.5}}}}), encoding="utf-8")
    found = detected_rules(design, str(path))
    assert found["edge_clearance"] == (0.5, "from demo.kicad_pro")
    assert Settings().apply_rules(read_dsn(str(path)), str(path))[1] == 0.5


def test_a_custom_pad_is_its_own_copper_not_the_hull_the_dsn_gives(tmp_path):
    """KiCad's DSN export writes a custom pad as the convex hull of its copper.
    A bent one (a reversible part's jumper strip, round another pin) then covers
    copper it does not: the board beside the DSN gives its real shape, and its
    anchor is where traces join it (its centroid is off its copper)."""
    from shapely.geometry import LineString
    from shapely.ops import unary_union
    strip = unary_union([Point(0, 0).buffer(0.125), LineString([(0, 0), (2, 0), (2, -2)]).buffer(0.125)])  # y up, mm
    hull = " ".join(f"{round(x * 1000)} {round(y * 1000)}" for x, y in strip.convex_hull.exterior.coords)
    dsn = f"""(pcb strip.dsn
  (parser (string_quote ") (space_in_quoted_tokens on) (host_cad "KiCad's Pcbnew"))
  (resolution um 10)
  (unit um)
  (structure
    (layer F.Cu (type signal) (property (index 0)))
    (layer B.Cu (type signal) (property (index 1)))
    (boundary (path pcb 0  0 0  20000 0  20000 -10000  0 -10000  0 0))
    (via "Via[0-1]_600:300_um")
    (rule (width 200) (clearance 200))
  )
  (placement
    (component J (place J1 8000 -3000 front 0))
    (component TP (place TP1 16000 -3000 front 0) (place TP2 4000 -8000 front 0))
  )
  (library
    (image J (pin Cust[T]Strip 1 0 0) (pin Round[T]Pad_600_um 2 1000 -1000))
    (image TP (pin Round[T]Pad_600_um 1 0 0))
    (padstack Cust[T]Strip (shape (polygon F.Cu 0  {hull})) (attach off))
    (padstack Round[T]Pad_600_um (shape (circle F.Cu 600)) (attach off))
  )
  (network
    (net A (pins J1-1 TP1-1))
    (net B (pins J1-2 TP2-1))
  )
  (wiring)
)
"""
    pcb = """(kicad_pcb (version 20240108) (generator "pcbnew")
  (layers (0 "F.Cu" signal) (31 "B.Cu" signal))
  (footprint "J" (layer "F.Cu") (at 8 3)
    (property "Reference" "J1" (at 0 0 0) (layer "F.SilkS"))
    (pad "1" smd custom (at 0 0) (size 0.25 0.25) (layers "F.Cu") (net 1 "A")
      (options (clearance outline) (anchor circle))
      (primitives (gr_line (start 0 0) (end 2 0) (width 0.25)) (gr_line (start 2 0) (end 2 2) (width 0.25))))
    (pad "2" smd circle (at 1 1) (size 0.6 0.6) (layers "F.Cu") (net 2 "B")))
)
"""
    found = {}
    for name, beside in (("alone", False), ("beside", True)):
        folder = tmp_path / name
        folder.mkdir()
        (folder / "strip.dsn").write_text(dsn, encoding="utf-8")
        if beside:
            (folder / "strip.kicad_pcb").write_text(pcb, encoding="utf-8")
        design = read_dsn(str(folder / "strip.dsn"))
        found[name] = {p.name: p for p in design.board.pads}
    hulled, real = found["alone"]["J1-1"], found["beside"]["J1-1"]
    assert hulled.shape.intersects(found["alone"]["J1-2"].shape)       # the hull covers the other pin
    assert abs(real.shape.area - strip.area) < 0.01 * strip.area       # the strip itself
    assert real.shape.distance(found["beside"]["J1-2"].shape) > 0.5    # well clear of it
    assert real.centre == pytest.approx((8.0, -3.0))                   # joined at its anchor, on its copper
    assert real.shape.contains(Point(real.centre))


def test_the_version_is_the_package_s():
    import re
    from pathlib import Path
    from weaveengine import __version__
    project = (Path(__file__).parent.parent / "pyproject.toml").read_text(encoding="utf-8")
    assert re.search(r'^version = "([^"]+)"', project, re.M)[1] == __version__


def test_an_unreachable_pad_is_told_why():
    from weaveengine.board import Board, Obstacle, Pad, Rules
    from weaveengine.cli import unreachable_note, why_buried
    from shapely.geometry import box
    board = Board.rectangle(20, 10, Rules(0.2, 0.2, edge_clearance=0.5))
    board.pads += [Pad.rect(0, 25, 5, 1, 1, 0, name="C1-1"),     # off the board
                   Pad.rect(1, 0.3, 5, 0.4, 1, 0, name="J1-1"),   # over the edge clearance
                   Pad.rect(2, 10, 5, 1, 1, 0, name="U1-1")]      # in a keepout
    board.obstacles.append(Obstacle(box(9, 4, 11, 6), -1))
    pads = {p.name: p for p in board.pads}
    assert [why_buried(board, pads[n]) for n in ("C1-1", "J1-1", "U1-1")] == ["outside", "edge", "keepout"]
    note = unreachable_note([0, 1, 2], board)
    assert "outside the board edge (a part not yet placed on the board?): C1-1" in note and "keepout: U1-1" in note


def test_a_hole_is_kept_clear_of_all_of_its_circle(tmp_path):
    """A hole is drawn as a polygon round its circle, not inside it: inside, a
    3.4 mm switch hole came 0.03 mm short, and traces passed it at 0.21 mm
    against a hole clearance of 0.25."""
    import math
    dsn = KICAD_STYLE.replace("(pcb demo.dsn", "(pcb hole.dsn")
    pcb = """(kicad_pcb (version 20240108) (generator "pcbnew")
  (layers (0 "F.Cu" signal) (31 "B.Cu" signal))
  (footprint "SW" (layer "F.Cu") (at 10 3)
    (property "Reference" "SW1" (at 0 0 0) (layer "F.SilkS"))
    (pad "" np_thru_hole circle (at 0 0) (size 3.4 3.4) (drill 3.4) (layers "*.Cu" "*.Mask"))
    (pad "" np_thru_hole oval (at 5 0) (size 1.5 2) (drill oval 1.5 2) (layers "*.Cu" "*.Mask")))
)
"""
    (tmp_path / "hole.dsn").write_text(dsn, encoding="utf-8")
    (tmp_path / "hole.kicad_pcb").write_text(pcb, encoding="utf-8")
    design = read_dsn(str(tmp_path / "hole.dsn"))
    holes = [o for o in design.board.obstacles if o.copper and o.shape.distance(Point(10, -3)) < 1e-9]
    assert holes
    circle = [Point(10 + 1.7 * math.cos(a / 100), -3 + 1.7 * math.sin(a / 100)) for a in range(629)]
    assert all(holes[0].shape.covers(p) for p in circle)
    # An oval hole is a slot, a line with round ends, wider at its corners than an ellipse would be.
    slots = [o for o in design.board.obstacles if o.copper and o.shape.distance(Point(15, -3)) < 1e-9]
    ends = [Point(15 + 0.75 * math.cos(a / 100), -3 + s * (0.25 + 0.75 * abs(math.sin(a / 100))))
            for a in range(315) for s in (1, -1)]
    assert slots and all(slots[0].shape.covers(p) for p in ends)


def test_copper_text_that_is_a_number_and_a_mounting_hole_s_own_clearance(tmp_path):
    """A pin number written on copper ("4") is copper all the same, and a
    mounting hole's local clearance holds its hole, not the board's."""
    dsn = KICAD_STYLE.replace("(pcb demo.dsn", "(pcb text.dsn")
    pcb = """(kicad_pcb (version 20240108) (generator "pcbnew")
  (layers (0 "F.Cu" signal) (31 "B.Cu" signal))
  (gr_text "4" (at 3 2 0) (layer "F.Cu") (effects (font (size 0.8 0.8) (thickness 0.2))))
  (footprint "MH" (layer "F.Cu") (at 16 8)
    (property "Reference" "MH1" (at 0 0 0) (layer "F.SilkS"))
    (pad "" np_thru_hole circle (at 0 0) (size 3 3) (drill 3) (layers "*.Cu" "*.Mask") (clearance 1.55)))
)
"""
    (tmp_path / "text.dsn").write_text(dsn, encoding="utf-8")
    (tmp_path / "text.kicad_pcb").write_text(pcb, encoding="utf-8")
    design = read_dsn(str(tmp_path / "text.dsn"))
    texts = [o for o in design.board.obstacles if o.copper and o.shape.contains(Point(3, -2))]
    holes = [o for o in design.board.obstacles if o.copper and o.shape.contains(Point(16, -8))]
    assert texts and holes and holes[0].clearance == pytest.approx(1.55)


def test_solder_mask_openings_are_kept_clear_of_other_nets(tmp_path):
    """A trace inside another net's solder-mask opening is bared beside it
    (KiCad's solder mask bridge): a pad whose opening reaches past the
    clearance asks that of foreign copper, and mask left open on purpose (drawn
    on a mask layer) keeps traces out on the copper below."""
    dsn = KICAD_STYLE.replace("(pcb demo.dsn", "(pcb mask.dsn")
    pcb = """(kicad_pcb (version 20240108) (generator "pcbnew")
  (layers (0 "F.Cu" signal) (31 "B.Cu" signal))
  (setup (pad_to_mask_clearance 0.3))
  (gr_rect (start 1 1) (end 3 2) (layer "F.Mask") (fill yes))
  (footprint "R_0805" (layer "F.Cu") (at 5 5)
    (property "Reference" "R1" (at 0 0 0) (layer "F.SilkS"))
    (pad "1" smd rect (at -0.9 0) (size 1 1.3) (layers "F.Cu" "F.Mask"))
    (pad "2" smd rect (at 0.9 0) (size 1 1.3) (layers "F.Cu" "F.Mask") (solder_mask_margin 0.05)))
)
"""
    (tmp_path / "mask.dsn").write_text(dsn, encoding="utf-8")
    (tmp_path / "mask.kicad_pcb").write_text(pcb, encoding="utf-8")
    design = read_dsn(str(tmp_path / "mask.dsn"))
    pads = {p.name: p for p in design.board.pads}
    assert pads["R1-1"].clearance == pytest.approx(0.3)   # the board's opening, wider than the 0.2 clearance
    assert pads["R1-2"].clearance == 0.0                  # its own, narrower: the clearance holds
    opened = [o for o in design.board.obstacles if o.shape.contains(Point(2, -1.5))]
    assert opened and opened[0].net_id == -1 and not opened[0].copper and opened[0].layers == frozenset({0})


def test_a_net_tie_is_copper_of_no_net_between_its_pads(tmp_path):
    """Copper joining pads of two nets (a net tie) is of no net to KiCad: every
    trace keeps clear of it, its pads apart."""
    dsn = KICAD_STYLE.replace("(pcb demo.dsn", "(pcb tie.dsn")
    pcb = """(kicad_pcb (version 20240108) (generator "pcbnew")
  (layers (0 "F.Cu" signal) (31 "B.Cu" signal))
  (footprint "R_0805" (layer "F.Cu") (at 5 5)
    (property "Reference" "R1" (at 0 0 0) (layer "F.SilkS"))
    (net_tie_pad_groups "1, 2")
    (fp_poly (pts (xy -0.9 -0.1) (xy 0.9 -0.1) (xy 0.9 0.1) (xy -0.9 0.1)) (stroke (width 0)) (fill yes) (layer "F.Cu")))
)
"""
    (tmp_path / "tie.dsn").write_text(dsn, encoding="utf-8")
    (tmp_path / "tie.kicad_pcb").write_text(pcb, encoding="utf-8")
    design = read_dsn(str(tmp_path / "tie.dsn"))
    ties = [o for o in design.board.obstacles if o.copper and o.shape.contains(Point(5, -5))]
    assert ties and ties[0].net_id == -1
    assert all(ties[0].shape.intersection(p.shape).area < 1e-9 for p in design.board.pads)


def test_a_zone_whose_fill_has_rounded_corners(tmp_path):
    """The fill of a zone smoothed with a fillet does not reach the corners of
    its outline, which is all a DSN gives."""
    from weaveengine.io.kicad import read_kicad
    pcb = """(kicad_pcb (version 20240108) (generator "pcbnew")
  (layers (0 "F.Cu" signal) (31 "B.Cu" signal))
  (zone (net 1) (net_name "GND") (layer "F.Cu") (min_thickness 0.25)
    (fill yes (smoothing fillet) (radius 2))
    (polygon (pts (xy 0 0) (xy 10 0) (xy 10 10) (xy 0 10))))
  (zone (net 1) (net_name "GND") (layer "B.Cu") (fill yes)
    (polygon (pts (xy 0 0) (xy 10 0) (xy 10 10) (xy 0 10))))
)
"""
    (tmp_path / "zone.kicad_pcb").write_text(pcb, encoding="utf-8")
    rounded = read_kicad(str(tmp_path / "zone.kicad_pcb")).rounded
    assert [(name, layer, radius, chamfer) for name, layer, _, radius, chamfer in rounded] == [("GND", "F.Cu", 2.0, False)]
    assert rounded[0][2].contains(Point(0.5, -0.5))


def test_a_zone_is_smoothed_as_kicad_smooths_it():
    """Each corner rounded to the radius, or cut back by it; never by more than
    half of a side, so a zone narrower than its radius (lumen-mobo: 2.5 mm, radius
    2) is rounded less instead of lost."""
    from shapely.geometry import box
    from weaveengine.io.kicad import _smoothed
    square = _smoothed(box(0, 0, 10, 10), 2.0, False)
    assert abs(square.area - (100 - (4 - math.pi) * 4)) < 0.1  # (arcs drawn as chords)
    assert not square.contains(Point(0.3, 0.3)) and square.contains(Point(2, 0.1))
    cut = _smoothed(box(0, 0, 10, 10), 2.0, True)
    assert abs(cut.area - (100 - 4 * 2)) < 1e-6
    narrow = _smoothed(box(0, 0, 2.5, 4), 2.0, False)
    assert abs(narrow.area - (10 - (4 - math.pi) * 1.25 ** 2)) < 0.1  # each corner at half the short side
    assert not narrow.contains(Point(0.1, 3.9)) and narrow.contains(Point(1.25, 3.9))


def test_layers_left_out_of_the_routing_keep_their_copper_and_planes(tmp_path, capsys):
    """Four layers, routed on the outer two only (``layers``, ``--layers``),
    as a board with its planes inside is: the inner layers carry no traces,
    but every via goes through them, so their keepouts and fixed copper are
    kept and their planes reached (``Board.unrouted``, at the indices after
    the routed layers'). A via is written on all four, else KiCad would take
    it for a blind one."""
    path, ses = tmp_path / "inner.dsn", tmp_path / "inner.ses"
    path.write_text(KICAD_STYLE.replace(
        "    (layer B.Cu", "    (layer In1.Cu (type signal) (property (index 1)))\n"
                         "    (layer In2.Cu (type power) (property (index 2)))\n    (layer B.Cu").replace(
        '    (via "Via[0-1]_800:400_um")', '    (keepout "" (rect In1.Cu 1000 -1000 3000 -3000))\n'
                                         '    (plane GND (polygon In2.Cu 0  0 0  20000 0  20000 -10000  0 -10000  0 0))\n'
                                         '    (via "Via[0-1]_800:400_um")').replace(
        "  (wiring)", '  (wiring (wire (path In1.Cu 250  12000 -2000  18000 -2000)(net "Net-(R1-Pad2)")(type fix)))'),
        encoding="utf-8")
    design = read_dsn(str(path), layers=["F.Cu", "B.Cu"])
    board = design.board
    assert board.layers == ["F.Cu", "B.Cu"] and board.unrouted == ["In1.Cu", "In2.Cu"]
    assert [(net, at) for net, at, _ in board.pours] == [(design.net_ids["GND"], 3)]
    assert sorted((tuple(round(v, 1) for v in o.shape.bounds), o.layers) for o in board.obstacles) == [
        ((1.0, -3.0, 3.0, -1.0), frozenset({2})), ((9.0, -6.0, 11.0, -4.0), frozenset({0})),   # the keepouts on In1 and on F.Cu,
        ((11.9, -2.1, 18.1, -1.9), frozenset({2}))]                                              # the fixed trace on In1
    assert read_dsn(str(path)).board.unrouted == []   # (all routed: as before)
    # The front pads of ground get vias down to its plane inside; each is drilled through all four layers.
    assert main([str(path), "--layers", "F.Cu", "B.Cu", "-o", str(ses), "--quiet", "--workers", "1"]) == 0
    assert "not routed: In1.Cu, In2.Cu" in capsys.readouterr().out
    session = parse_sexpr(ses.read_text(encoding="utf-8"))
    library = next(c for c in next(c for c in session if isinstance(c, list) and c[0] == "routes") if isinstance(c, list) and c[0] == "library_out")
    padstack = next(c for c in library if isinstance(c, list) and c[0] == "padstack")
    assert sorted(shape[1][1] for shape in padstack if isinstance(shape, list) and shape[0] == "shape") == ["B.Cu", "F.Cu", "In1.Cu", "In2.Cu"]


def test_one_layer_routed_and_its_plane_on_the_other(tmp_path):
    """Two layers, ground poured on the back, routed on the front only: the
    front pads of ground are on the front alone (not on every layer, as they
    are on every layer routed), so each gets a via down to the pour; a pad of
    a part on the back is no pad of the routing, but copper its vias keep
    clear of."""
    from weaveengine.router import route_board
    path = tmp_path / "front.dsn"
    path.write_text(KICAD_STYLE.replace(
        '    (via "Via[0-1]_800:400_um")', '    (plane GND (polygon B.Cu 0  0 0  20000 0  20000 -10000  0 -10000  0 0))\n'
                                         '    (via "Via[0-1]_800:400_um")').replace(
        "    (component TP (place TP1 10000 -8000 back 0))", "    (component TP (place TP1 10000 -8000 back 0))\n"
                                                             "    (component C_B (place C1 3000 -5000 back 0))").replace(
        "    (image TP (pin Round[A]Pad_1500_um 1 0 0))", "    (image TP (pin Round[A]Pad_1500_um 1 0 0))\n"
                                                         "    (image C_B (pin Rect[T]Pad_1000x1300_um 1 0 0))").replace(
        '(pins R1-2 R2-1)', '(pins R1-2 R2-1 C1-1)'), encoding="utf-8")
    design = read_dsn(str(path), layers=["F.Cu"])
    board = design.board
    pads = {p.name: p for p in board.pads}
    assert board.unrouted == ["B.Cu"] and pads["R1-1"].layers == frozenset({0}) and pads["TP1-1"].layers is None
    assert "C1-1" not in pads
    under = [o for o in board.obstacles if o.layers == frozenset({1})]
    assert len(under) == 1 and under[0].copper and under[0].net_id == design.net_ids["Net-(R1-Pad2)"]
    assert abs(under[0].shape.centroid.x - 3.0) < 1e-6
    result = route_board(board, options=Options(portfolio=1), workers=1)
    assert result.stats["fanout"] == 2 and result.stats["plane_left"] == []   # R1-1 and R2-2; TP1-1 is in the pour already
    assert all(v.shape.distance(under[0].shape) >= board.rules.clearance - 1e-6 for v in result.vias)
