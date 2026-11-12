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
               "demand": "demand", "ripup": "ripup", "refine": "refine"}
    for setting, option in flipped.items():
        t = Settings(**{setting: False})
        assert getattr(t.options(), option) is False, setting
    assert Settings(teardrop_style="none").options().teardrops is False  # (teardrops: a choice of three, one of them none)
    for setting, option in {"pour_reliefs": "reliefs", "pour_narrow_reliefs": "narrow_reliefs"}.items():  # (off unless asked for)
        assert getattr(s.options(), option) is False and getattr(Settings(**{setting: True}).options(), option) is True, setting
    assert set(flipped.values()) | {"teardrops", "skeleton", "portfolio", "teardrop_max_length", "teardrop_max_width", "teardrop_breathing",
                                    "reliefs", "narrow_reliefs"} \
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
