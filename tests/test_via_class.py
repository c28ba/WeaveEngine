"""A via size per net class (ROADMAP 1.1 item 8): each net's vias are its class's size, read from the DSN, routed
with, checked and written so (the SES names each via's padstack as KiCad's DSN export does); a setting makes them all
the default class's."""
import math

from shapely.geometry import Point

from tests.test_io import KICAD_STYLE
from tests.test_router import assert_clean, crossing_board
from weaveengine.cli import main
from weaveengine.io.check import read_session
from weaveengine.io.dsn import Design, read_dsn
from weaveengine.io.ses import via_padstack, write_ses
from weaveengine.plan.context import Options
from weaveengine.router import route_board
from weaveengine.settings import Settings
from weaveengine.topo import sites

STACKS = """    (padstack "Via[0-1]_800:400_um" (shape (circle F.Cu 800)) (shape (circle B.Cu 800)) (attach off))
    (padstack "Via[0-1]_1200:600_um" (shape (circle F.Cu 1200)) (shape (circle B.Cu 1200)) (attach off))
  )
  (network"""
CLASSES = KICAD_STYLE.replace("  )\n  (network", STACKS, 1).replace(
    '(via "Via[0-1]_800:400_um")', '(via "Via[0-1]_800:400_um" "Via[0-1]_1200:600_um")').replace(
    '(class kicad_default "" GND "Net-(R1-Pad2)" (circuit (use_via Via[0-1]_800:400_um)) (rule (width 250) (clearance 200.1)))',
    '(class kicad_default "" "Net-(R1-Pad2)" (circuit (use_via Via[0-1]_800:400_um)) (rule (width 250) (clearance 200.1)))\n'
    '    (class Power GND (circuit (use_via "Via[0-1]_1200:600_um")) (rule (width 250) (clearance 200.1)))')


def test_a_class_with_its_own_via_gives_its_nets_that_size(tmp_path):
    path = tmp_path / "classes.dsn"
    path.write_text(CLASSES, encoding="utf-8")
    design = read_dsn(str(path))
    rules, gnd, other = design.board.rules, design.net_ids["GND"], design.net_ids["Net-(R1-Pad2)"]
    assert rules.via(other) == (0.8, 0.4) and rules.via(-1) == (0.8, 0.4)  # (the first via of the structure is the default)
    assert rules.via(gnd) == (1.2, 0.6) and rules.net_via == {gnd: (1.2, 0.6)} and rules.widest_via == 1.2
    assert "1.2/0.6 mm in other net classes" in rules.via_text
    assert via_padstack(design, (1.2, 0.6))[0] == "Via[0-1]_1200:600_um"   # the DSN's own, by its size
    assert via_padstack(design, (0.5, 0.25))[0] == "Via[0-1]_500:250_um"   # else named as KiCad names them (its drill)
    # A hole clearance asks more of foreign copper where a via's ring is narrower: per size.
    rules.hole_clearance = 0.25
    assert math.isclose(rules.via_clearance(other), 0.05) and rules.via_clearance(gnd) == 0.0
    # Bigger vias keep foreign traces further off, and further from each other.
    assert sites.keep_off(rules, gnd) > sites.keep_off(rules, other)
    assert math.isclose(rules.via_spacing(gnd, other) - rules.via_spacing(other, other), 0.2)
    # The setting off: every via the default class's.
    Settings(class_vias=False).apply_rules(design)
    assert rules.net_via == {} and rules.via(gnd) == (0.8, 0.4)


def test_the_flag_gives_every_via_one_size(tmp_path, capsys):
    path = tmp_path / "classes.dsn"
    path.write_text(CLASSES, encoding="utf-8")
    main([str(path), "-o", str(tmp_path / "a.ses"), "--quiet", "--workers", "1"])
    assert "1.2/0.6 mm in other net classes" in capsys.readouterr().out
    main([str(path), "-o", str(tmp_path / "b.ses"), "--quiet", "--workers", "1", "--one-via-size"])
    assert "other net classes" not in capsys.readouterr().out


def test_vias_are_routed_checked_and_written_at_their_class_size(tmp_path):
    """Nets 0 and 1 of the crossing board in a class with larger vias: their vias are routed at that size, clean,
    and the session writes them with their own padstack."""
    board = crossing_board()
    board.rules.net_via = {0: (0.9, 0.45), 1: (0.9, 0.45)}
    result = route_board(board, options=Options(portfolio=1), workers=1)
    assert_clean(board, result)
    assert result.stats["routed"] == 4 and {v.net_id in (0, 1) for v in result.vias} == {True, False}  # (vias of both sizes)
    assert all(math.isclose(v.radius, board.rules.via_radius(v.net_id)) for v in result.vias)
    # No two vias closer than their copper and the clearance allow.
    for a in result.vias:
        for b in result.vias:
            if a is not b and a.net_id != b.net_id:
                assert Point(a.centre).distance(Point(b.centre)) >= a.radius + b.radius + board.rules.clearance - 1e-6
    ses = str(tmp_path / "out.ses")
    write_ses(Design("crossing", result.board), result, ses)
    _, vias = read_session(ses)
    assert sorted({(net, d) for net, _, _, d in vias}) == sorted({(f"N{v.net_id}", 2 * v.radius) for v in result.vias})
    text = open(ses, encoding="utf-8").read()
    assert '(padstack "Via[0-1]_900:450_um"' in text and '(padstack "Via[0-1]_600:300_um"' in text
