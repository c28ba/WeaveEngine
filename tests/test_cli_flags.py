"""Every routing option and cost parameter the settings carry has a command-line flag."""
import dataclasses

import pytest

from weaveengine import cli
from weaveengine.plan.context import Options
from weaveengine.settings import DESCRIPTIONS, Settings


def help_text(capsys):
    with pytest.raises(SystemExit):
        cli.main(["--help"])
    return capsys.readouterr().out


def test_every_option_has_a_flag(capsys):
    text = help_text(capsys)
    assert set(cli.OPTION_FLAGS) == {f.name for f in dataclasses.fields(Options)}
    for flag in cli.OPTION_FLAGS.values():
        assert flag in text, flag


def test_every_routing_setting_has_a_flag_in_a_group_of_the_same_name(capsys):
    """Each setting of Routing (and the workers of App) has its flag, and --help lists the flag under the setting's topic."""
    from weaveengine.settings import section_of
    text = help_text(capsys)
    assert set(cli.SETTING_FLAGS) == {n for n, (_, _, topic) in DESCRIPTIONS.items() if section_of(topic) == "Routing" or n == "workers"}
    sections = {}
    current = None
    for line in text.splitlines():
        if line and not line.startswith(" ") and line.endswith(":"):
            current = line[:-1]
        elif current:
            sections.setdefault(current, []).append(line)
    for name, flag in cli.SETTING_FLAGS.items():
        label, _, topic = DESCRIPTIONS[name]
        title = ("App: " if name == "workers" else "Routing: ") + topic
        assert any(line.strip().startswith(flag) or f", {flag}" in line for line in sections[title]), (name, flag, title)
    assert not any(t.startswith("App: ") and t != "App: Computer" for t in sections)  # (display settings have no flags)


def test_every_cost_parameter_the_settings_override_has_a_flag(capsys):
    text = help_text(capsys)
    assert set(Settings().cost_overrides()) | {"layer_cost"} == set(cli.COST_FLAGS)
    for flag in cli.COST_FLAGS.values():
        assert flag in text, flag


def test_every_option_is_a_setting_too():
    """The app's settings carry each option (``Settings.options``), under a label."""
    s = Settings()
    assert s.options() == Options(portfolio=s.portfolio) and "taper_to_pins" in DESCRIPTIONS
    flipped = {"taper_to_pins": "taper", "smooth_corners": "smooth", "slide_vias": "slide", "untangle": "untangle",
               "narrow_to_reach": "narrow", "fanout_to_planes": "fanout", "use_vias": "vias",
               "global_selection": "global_selection", "regret_order": "regret_order", "lookahead": "lookahead",
               "demand": "demand", "ripup": "ripup", "refine": "refine", "pour_spokes": "spokes"}
    for setting, option in flipped.items():
        t = Settings(**{setting: False})
        assert getattr(t.options(), option) is False, setting
    assert Settings(teardrop_style="none").options().teardrops is False  # (teardrops: a choice of three, one of them none)
    for setting, option in {"pour_reliefs": "reliefs", "pour_narrow_reliefs": "narrow_reliefs", "join_bare_islands": "bare_islands"}.items():  # (off unless asked for)
        assert getattr(s.options(), option) is False and getattr(Settings(**{setting: True}).options(), option) is True, setting
    assert set(flipped.values()) | {"teardrops", "skeleton", "portfolio", "teardrop_max_length", "teardrop_max_width", "teardrop_breathing",
                                    "least_angle", "reliefs", "narrow_reliefs", "bare_islands"} \
        == {f.name for f in dataclasses.fields(Options)}


def test_the_switches_turn_options_off(tmp_path, monkeypatch):
    from tests.conftest import demo_board
    from weaveengine.io.dsn import write_dsn
    dsn = str(tmp_path / "b.dsn")
    write_dsn(demo_board(), dsn)
    seen = {}

    def fake(board, params=None, options=None, **kwargs):
        seen.update(options=options, params=params, kwargs=kwargs)
        raise KeyboardInterrupt
    monkeypatch.setattr(cli, "route_board", fake)
    cli.main([dsn, "--quiet", "--workers", "1", "--no-taper", "--no-smooth", "--no-ripup", "--teardrop-max-length", "0.5",
              "--candidates", "3", "--heuristic-weight", "1.5", "--drc-rounds", "2"])
    o = seen["options"]
    assert not o.taper and not o.smooth and not o.ripup and o.teardrop_max_length == 0.5 and o.refine and o.untangle
    assert seen["params"].K == 3 and seen["params"].h_weight == 1.5 and seen["kwargs"]["drc_rounds"] == 2


def test_how_margin_drawings_are_held_is_a_flag_and_a_setting(tmp_path, monkeypatch):
    """--margin-drawings, and the setting margin_drawings, say how the Margin drawings are held as board edge (``geom.margin``)."""
    from tests.conftest import demo_board
    from weaveengine.io.dsn import read_dsn, write_dsn
    dsn = str(tmp_path / "b.dsn")
    write_dsn(demo_board(), dsn)
    seen = []

    def fake(board, params=None, options=None, **kwargs):
        seen.append(board.margin_edge)
        raise KeyboardInterrupt
    monkeypatch.setattr(cli, "route_board", fake)
    cli.main([dsn, "--quiet", "--workers", "1"])
    cli.main([dsn, "--quiet", "--workers", "1", "--margin-drawings", "edge"])
    cli.main([dsn, "--quiet", "--workers", "1", "--margin-drawings", "off"])
    assert seen == ["added", "edge", "off"]
    for mode in ("added", "edge", "off"):
        design = read_dsn(dsn)
        Settings(margin_drawings=mode).apply_rules(design, dsn)
        assert design.board.margin_edge == mode


def test_layers_and_layer_cost_are_settings(tmp_path):
    from tests.conftest import demo_board
    from weaveengine.io.dsn import read_dsn, write_dsn
    from weaveengine.settings import layer_costs, layer_list
    dsn = str(tmp_path / "b.dsn")
    write_dsn(demo_board(), dsn)
    design = read_dsn(dsn)
    first, last = design.board.layers[0], design.board.layers[-1]
    assert layer_list("") is None and layer_list(f" {first} , {last},") == [first, last]
    assert layer_costs(design, "".split(",")) == {} and layer_costs(design, [f" {last}:5 "]) == {len(design.board.layers) - 1: 5.0}
    for bad in (f"{first}:0.5", "nowhere:3", first):
        with pytest.raises(ValueError):
            layer_costs(design, [bad])
    assert Settings().layers == "" and Settings().layer_cost == ""
