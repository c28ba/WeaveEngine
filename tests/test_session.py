import json
import time

from bench.generators import random_grid
from tests.conftest import demo_board
from weaveengine.io.dsn import write_dsn
from weaveengine.session import Job
from weaveengine.settings import DESCRIPTIONS, Settings, default_path


def test_settings_round_trip_and_bad_files(tmp_path):
    path = str(tmp_path / "settings.json")
    assert Settings.load(path) == Settings()                     # no file: the defaults
    custom = Settings(workers=3, margin=0.02, teardrops=False, heuristic_weight=1.5, ignore_keepouts=True)
    custom.save(path)
    assert Settings.load(path) == custom
    data = json.load(open(path))
    assert data["workers"] == 3 and data["teardrops"] is False
    # hand-edited: unknown keys ignored, bad values keep their defaults, the rest is read
    data.update({"no_such_setting": 1, "margin": "not a number", "seed": 7})
    json.dump(data, open(path, "w"))
    loaded = Settings.load(path)
    assert loaded.seed == 7 and loaded.margin == Settings().margin
    open(path, "w").write("{ this is not json")
    assert Settings.load(path) == Settings()
    assert default_path().endswith("settings.json")
    # every setting has a label and a description for the editor
    assert set(DESCRIPTIONS) == set(vars(Settings()))
    # numbers with an "automatic" state are stored as 0 and offered as a switch, never as a magic number
    from weaveengine.settings import AUTOMATIC
    assert set(AUTOMATIC) <= set(DESCRIPTIONS)
    assert all(getattr(Settings(), name) == 0 for name in AUTOMATIC)
    assert not any("0 =" in text for _, text, _ in DESCRIPTIONS.values())


def test_settings_become_router_arguments():
    s = Settings(portfolio=2, refine=False, teardrop_max_width=1.2, candidates=3, heuristic_weight=1.5, use_vias=False)
    options = s.options()
    assert options.portfolio == 2 and options.refine is False and options.teardrop_max_width == 1.2 and options.vias is False
    assert s.cost_overrides() == {"K": 3, "K_reroute": 2, "max_rounds": 100, "h_weight": 1.5}


def test_job_streams_events_and_returns_a_result(tmp_path):
    dsn = str(tmp_path / "board.dsn")
    write_dsn(demo_board(), dsn)
    job = Job(dsn, Settings(portfolio=2, workers=2))
    seen = []
    job.start()
    last = job.wait(120, seen.append)
    assert last is not None and last["type"] == "done", last
    kinds = [e["type"] for e in seen]
    assert kinds[0] == "status" and "board" in kinds and "progress" in kinds and "pass" in kinds
    status = seen[0]
    assert isinstance(status["compiled"], bool) and status["message"]
    assert (status["warning"] == "") == status["compiled"]
    board_event = next(e for e in seen if e["type"] == "board")
    assert len(board_event["design"].board.pads) == 6
    snaps = [e for e in seen if e["type"] == "snapshot"]
    assert {e["variant"] for e in snaps} >= {0, -1}              # live from the variants, then the outcome
    final = [e for e in snaps if e["final"]][-1]
    assert final["routed"] == final["total"] == 3 and len(final["wires"]) == 3 and final["open"] == []
    layer, net, points = final["wires"][0]
    assert layer == 0 and len(points) >= 2
    result = last["result"]
    assert result.stats["routed"] == 3 and result.layers == [] and len(result.polylines) == 3
    assert not job.running()


def test_job_can_be_cancelled(tmp_path):
    dsn = str(tmp_path / "board.dsn")
    write_dsn(random_grid(12, 12, nets=60, seed=5), dsn)        # long enough to still be running
    job = Job(dsn, Settings())
    job.start()
    deadline = time.time() + 30
    while time.time() < deadline and not any(e["type"] == "progress" for e in job.poll()):
        time.sleep(0.05)
    assert job.running()
    t = time.time()
    job.cancel()
    assert not job.running() and time.time() - t < 5
    assert job.poll() == [] or all(e["type"] != "done" for e in job.poll())


def test_job_reports_a_bad_file_instead_of_dying(tmp_path):
    path = tmp_path / "bad.dsn"
    path.write_text("(pcb broken (structure))")
    job = Job(str(path))
    job.start()
    last = job.wait(60)
    assert last["type"] == "error" and "DSN" in last["text"]
