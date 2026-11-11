"""Teardrop styles, the routed copy of the KiCad board, and teardrops made again after routing."""
import math
import os
import types

import pytest

from tests.conftest import demo_board
from weaveengine.io.dsn import Design, parse_sexpr
from weaveengine.io.kicad_board import write_routed_board
from weaveengine.router import redo_teardrops, route_board

# A board as KiCad 10 writes it, cut down: a layer with a user name, a track (unlocked) and a locked one,
# a teardrop zone of an earlier routing, and a pour with an old fill.
KICAD10 = """(kicad_pcb
\t(version 20260206)
\t(generator "pcbnew")
\t(layers
\t\t(0 "F.Cu" signal "Top")
\t\t(2 "B.Cu" signal)
\t)
\t(segment
\t\t(start 1 1)
\t\t(end 2 2)
\t\t(width 0.2)
\t\t(layer "F.Cu")
\t\t(net "N1")
\t\t(uuid "a")
\t)
\t(segment
\t\t(start 3 3)
\t\t(end 4 4)
\t\t(width 0.2)
\t\t(locked yes)
\t\t(layer "F.Cu")
\t\t(net "N1")
\t\t(uuid "b")
\t)
\t(zone
\t\t(net "N1")
\t\t(layer "F.Cu")
\t\t(name "$teardrop_padvia$")
\t\t(attr
\t\t\t(teardrop
\t\t\t\t(type padvia)
\t\t\t)
\t\t)
\t\t(polygon (pts (xy 0 0) (xy 1 0) (xy 1 1)))
\t)
\t(zone
\t\t(net "N2")
\t\t(layer "B.Cu")
\t\t(polygon (pts (xy 0 0) (xy 9 0) (xy 9 9)))
\t\t(filled_polygon
\t\t\t(layer "B.Cu")
\t\t\t(pts (xy 0 0) (xy 9 0) (xy 9 9))
\t\t)
\t)
)
"""


@pytest.fixture(scope="module")
def routed():
    board = demo_board()
    board.layers = ["Top", "B.Cu"]
    result = route_board(board)
    design = Design("demo", board, net_ids={"N1": 1, "N2": 2, "N3": 3})
    return design, result


def _items(path, kind):
    root = parse_sexpr(open(path, encoding="utf-8").read())
    return [x for x in root[1:] if isinstance(x, list) and x and x[0] == kind]


def test_the_routed_copy_holds_the_routing_and_teardrop_zones(routed, tmp_path):
    design, result = routed
    assert result.teardrops, "the demo board should get teardrops"
    source, out = tmp_path / "b.kicad_pcb", tmp_path / "b-routed.kicad_pcb"
    source.write_text(KICAD10, encoding="utf-8")
    (tmp_path / "b.kicad_pro").write_text("{}", encoding="utf-8")
    counts = write_routed_board(design, result, str(source), str(out), "zones")
    assert source.read_text(encoding="utf-8") == KICAD10      # the original is only read
    assert (tmp_path / "b-routed.kicad_pro").exists()           # its project beside it: KiCad checks the copy by the same rules
    segments = _items(out, "segment")
    assert ["uuid", "b"] in segments[0] and all(["uuid", "a"] not in s for s in segments)  # the locked track stays, the other goes
    assert len(segments) - 1 == counts["tracks"] > 0
    zones = _items(out, "zone")
    drops = [z for z in zones if any(isinstance(x, list) and x[0] == "attr" for x in z)]
    assert len(drops) == counts["zones"] == sum(len(d) for d in result.teardrops.values())  # the old teardrop is gone
    layers = {x[1] for item in segments + drops for x in item if isinstance(x, list) and x[0] == "layer"}
    assert layers <= {"F.Cu", "B.Cu"} and "F.Cu" in layers      # the DSN's layer name "Top" is KiCad's F.Cu
    pour = [z for z in zones if z not in drops]
    assert len(pour) == 1 and not any(isinstance(x, list) and x[0] == "filled_polygon" for x in pour[0])  # its old fill is gone
    for s in segments[1:]:
        assert [x for x in s if x[0] == "net"][0][1] in ("N1", "N2", "N3")
    with pytest.raises(ValueError):
        write_routed_board(design, result, str(source), str(source), "zones")  # never over the board itself


def test_no_teardrops_means_none_and_no_pad_marked_for_kicads(routed, tmp_path):
    design, result = routed
    source, out = tmp_path / "b.kicad_pcb", tmp_path / "c.kicad_pcb"
    source.write_text(KICAD10, encoding="utf-8")
    counts = write_routed_board(design, result, str(source), str(out), "none")
    text = out.read_text(encoding="utf-8")
    assert counts["zones"] == 0 and "(teardrops" not in text and "teardrop" not in text.split("(segment", 1)[1]
    tracks = write_routed_board(design, result, str(source), str(out), "tracks")["tracks"]
    assert tracks > counts["tracks"]  # the fans of tracks


def test_a_kicad9_board_gets_net_numbers(routed, tmp_path):
    design, result = routed
    source, out = tmp_path / "b.kicad_pcb", tmp_path / "c.kicad_pcb"
    source.write_text(KICAD10.replace("\t(layers", '\t(net 0 "")\n\t(net 1 "N1")\n\t(net 2 "N2")\n\t(net 3 "N3")\n\t(layers'),
                      encoding="utf-8")
    write_routed_board(design, result, str(source), str(out), "zones")
    added = [s for s in _items(out, "segment") if ["uuid", "b"] not in s] + [z for z in _items(out, "zone") if ["layer", "B.Cu"] not in z]
    nets = {x[1] for s in added for x in s if isinstance(x, list) and x[0] == "net"}
    assert nets <= {"1", "2", "3", 1, 2, 3}


def test_teardrops_are_made_again_after_routing_without_rerouting(routed):
    design, result = routed
    again = redo_teardrops(result)
    assert again.teardrops == result.teardrops                   # the same limits: the same teardrops
    assert again.polylines is result.polylines and again.vias is result.vias
    off = redo_teardrops(result, on=False)
    assert off.teardrops == {} and off.polylines is result.polylines
    short = redo_teardrops(result, max_length=0.2)
    assert short.teardrops and short.teardrops != result.teardrops

    def reach(drops):  # how far the teardrops reach out from their pads' centres
        return max(math.dist(poly[0], poly[5]) for polys in drops.values() for poly in polys)
    assert reach(short.teardrops) < reach(result.teardrops)


def test_the_app_makes_only_the_teardrops_again_when_their_settings_change(routed):
    pytest.importorskip("PySide6")
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    from weaveengine.app import MainWindow
    from weaveengine.settings import Settings
    design, result = routed
    shown, said = [], []
    window = types.SimpleNamespace(result=result, job=None, settings=Settings(teardrop_style="none"),
                                   _show=shown.append, say=said.append)
    MainWindow._teardrops_again(window, Settings())
    assert window.result.teardrops == {} and window.result.polylines is result.polylines and shown
    window.settings = Settings(teardrop_style="zones", teardrop_max_length=0.2)
    MainWindow._teardrops_again(window, Settings(teardrop_style="none"))
    assert window.result.teardrops == redo_teardrops(result, max_length=0.2).teardrops
    count = len(shown)
    window.settings = Settings(teardrop_style="tracks", teardrop_max_length=0.2)  # only the style: nothing to make again
    MainWindow._teardrops_again(window, Settings(teardrop_style="zones", teardrop_max_length=0.2))
    assert len(shown) == count


def test_settings_carry_the_teardrop_style(tmp_path):
    import json
    from weaveengine.settings import Settings
    path = tmp_path / "s.json"
    path.write_text(json.dumps({"teardrops_in_ses": False}), encoding="utf-8")  # (a settings file from before 1.1)
    assert Settings.load(str(path)).teardrop_style == "none"
    path.write_text(json.dumps({"teardrop_style": "blobs"}), encoding="utf-8")
    assert Settings.load(str(path)).teardrop_style == "tracks"
    assert Settings(teardrop_style="none").options().teardrops is False
    assert Settings(teardrop_style="zones").options().teardrops is True


def test_the_cli_takes_a_teardrop_style(tmp_path, monkeypatch):
    from weaveengine import cli
    from weaveengine.io.dsn import write_dsn
    dsn = str(tmp_path / "b.dsn")
    write_dsn(demo_board(), dsn)
    seen = []

    def fake(board, params=None, options=None, **kwargs):
        seen.append(options)
        raise KeyboardInterrupt
    monkeypatch.setattr(cli, "route_board", fake)
    for args, on in ((["--teardrops", "none"], False), (["--no-teardrops"], False), (["--no-ses-teardrops"], False),
                     (["--teardrops", "zones"], True), ([], True)):
        cli.main([dsn, "--quiet", "--workers", "1", *args])
        assert seen[-1].teardrops is on, args
