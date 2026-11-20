from tests.conftest import demo_board
from weaveengine.router import route_board
from weaveengine.viz.svg import export_result, export_svg


def test_svg_export(tmp_path):
    board = demo_board()
    result = route_board(board)
    out = tmp_path / "board.svg"
    export_svg(board, str(out), result.polylines, result.wire_net, pmap=result.pmap)
    text = out.read_text(encoding="utf-8")
    assert text.startswith("<svg") and text.rstrip().endswith("</svg>")
    assert text.count("<title>wire") == 3
    assert text.count("<title>pad") == 6
    export_result(result, str(out))
    text = out.read_text(encoding="utf-8")
    assert text.count("<title>wire") == 3 and text.count("<polygon") >= 1 + 6 + 6  # keepout, pads, teardrops (the outline is a path)


def _run(est, rounds, at=0.0, heat="racing variants 1-4 of 8", complete=True):
    """Feeds an estimator one heat as the router reports it: every variant
    takes 4 s to reach rip-up, then ``rounds`` rounds of 1 s (the last 8
    without improvement), 10 s of refinement and geometry and one repair of
    15 s (in the proportions of real runs, design section 24). Yields the
    time after each event."""
    def say(t, v, phase, done=0.0, total=1.0, **more):
        est.event({"type": "progress", "pass": 1, "variant": v, "phase": phase, "done": done, "total": total, **more}, t)
        return t
    variants = range(4) if heat else range(1)
    if heat:
        say(at, 0, heat)
    for v in variants:
        say(at, v, "candidates")
    yield at
    yield say(at + 2.0, 0, "commit", 1, 10)
    for r in range(1, rounds + 1):
        for v in variants:
            say(at + 3.0 + r, v, "rip-up", min(r, rounds - 8) * 10, rounds * 10, round=r, quiet=min(8, rounds - r + 1), most=100 - r + 1)
        yield at + 3.0 + r
    end = at + 4.0 + rounds
    for v in variants:
        say(end, v, "refinement")
    yield end + 5.0
    for v in variants:
        say(end + 10.0, v, "design-rule repair", 0, 4)
        say(end + 10.0, v, "rip-up", 0, 5, round=rounds + 1, quiet=3, most=5)
    yield end + 10.0
    for v in variants:
        say(end + 18.0, v, "refinement")
    yield end + 18.0
    for v in variants:
        say(end + 25.0, v, "finished", complete=complete)
    yield end + 25.0


def test_time_left_is_a_range_that_holds_and_narrows():
    from weaveengine.progress import Estimator, clock, phrase
    assert clock(75) == "1:15" and clock(3725) == "1:02:05"
    est = Estimator(now=0.0)
    total = 4.0 + 20 + 25.0
    shares, widths = [], []
    for t in _run(est, rounds=20):
        if est.left(t) is None:
            assert t <= 2.0                                         # nothing to go on before commit is under way: it says nothing
            continue
        lo, mid, hi = est.left(t)
        assert lo <= total - t + 1e-9 <= hi + 1e-9, (t, lo, hi)     # the run ends inside the range, all the way through
        assert lo <= mid <= hi     # the run ends inside the range, all the way through
        shares.append(est.progress(t))
        widths.append(hi - lo)
    assert shares == sorted(shares) and shares[-1] > 0.95           # the bar never moves backwards, and ends at the end
    assert widths[-1] < 0.05 * total < widths[3] < widths[1]       # the range narrows (to the finishing's, at the end)
    assert est.more(total) == 0.0                                   # a variant connected everything: no other heat
    assert phrase((50.0, 54.0, 60.0)) == "about 0:55 left" and phrase((10.0, 30.0, 90.0)) == "about 0:30 left (0:10 to 1:30)"
    assert "more if no variant connects everything" in phrase((10.0, 11.0, 12.0), 30.0)


def test_another_heat_is_said_beside_the_range():
    from weaveengine.progress import Estimator
    est = Estimator(now=0.0)
    said = [(t, est.more(t)) for t in _run(est, rounds=12, complete=False)]
    end = said[-1][0]
    assert all(0.5 * end < more < 2.0 * end for t, more in said[4:-1]) and said[0][1] == 0.0  # while it may follow: about what this heat takes
    for n, t in enumerate(_run(est, rounds=12, at=end, heat="racing variants 5-8 of 8", complete=False)):
        assert est.more(t) == 0.0                                   # the last heat: nothing can follow
        if n == 5:
            lo, _, hi = est.left(t)
            assert lo <= 2 * end - t <= hi
            assert est.status().startswith("variants 5 to 8")


def test_rip_up_that_keeps_improving_is_counted_down():
    """Each improvement puts back the rounds the stop rule allows, but the
    rounds left are counted by how likely improvements still are: fewer the
    longer rip-up has run, so the middle falls with the truth (design 24)."""
    from weaveengine.progress import rounds_left
    rounds = [rounds_left(n, 8, 8, 1000, 100) for n in (1, 10, 20, 30)]  # an improvement every time
    assert all(r[0] == 8 for r in rounds)                           # at least: none improves from here
    mids = [r[1] for r in rounds]
    assert mids == sorted(mids, reverse=True) and mids[0] > 2 * mids[-1] and mids[-1] >= 8
    assert all(r[0] <= r[1] <= r[2] for r in rounds)
    assert rounds_left(10, 3, 8, 1000, 100)[1] < rounds_left(10, 8, 8, 1000, 100)[1]  # long since the last: soon over
    assert rounds_left(10, 8, 8, 20, 100)[1] < rounds_left(10, 8, 8, 5000, 100)[1]   # more to clear: longer
    assert rounds_left(1, 5, 8, 1000, 5) == (5, 5, 5)                 # never past the rounds still allowed


def test_a_run_on_record_is_estimated_by_the_time_it_took(tmp_path):
    from weaveengine.progress import Estimator, Timings
    board = tmp_path / "board.dsn"
    board.write_text("(pcb)", encoding="utf-8")
    timings = Timings(str(tmp_path / "timings.json"))
    key = Timings.key(str(board), {"seed": 0})
    assert timings.last(key) is None and key != Timings.key(str(board), {"seed": 1})
    timings.record(key, 100.0)
    est = Estimator(now=0.0, last=timings.last(key))
    lo, _, hi = est.left(40.0)
    assert est.as_before(40.0) and 45.0 < lo < 60.0 < hi < 80.0     # close to the 60 s that are left
    assert not est.as_before(120.0)                                 # outlasted: back to what the run itself says


def test_progress_bar_output():
    import io
    from weaveengine.progress import ProgressBar
    out = io.StringIO()
    bar = ProgressBar(out)
    for phase, done, total in (("candidates", 0, 1), ("commit", 5, 10), ("geometry", 0, 1)):
        bar.event({"type": "progress", "pass": 1, "variant": 0, "phase": phase, "done": done, "total": total})
    bar.event({"type": "progress", "pass": 1, "variant": 3, "phase": "rip-up", "done": 0, "total": 1})  # not the plain variant: not drawn
    text = out.getvalue()
    assert "commit" in text and "geometry" in text and "gone" in text and "rip-up" not in text



def test_svg_draws_no_via_areas_hatched(tmp_path):
    from shapely.geometry import box
    board = demo_board()
    board.no_via = [box(1, 1, 3, 3)]
    out = tmp_path / "board.svg"
    export_svg(board, str(out))
    text = out.read_text(encoding="utf-8")
    assert 'fill="url(#no-via)"' in text and "<title>no vias</title>" in text


def test_the_finishing_is_counted_by_the_board_and_said_step_by_step():
    from weaveengine.progress import FINISH_POURED, Estimator
    est = Estimator(now=0.0)
    est.event({"type": "progress", "pass": 1, "variant": -1, "phase": "setup", "done": 0, "total": 1, "pads": 1000, "poured": 400}, 0.0)
    times = list(_run(est, rounds=20))
    end = times[-1]
    lo, _, hi = est.left(end)                                       # the variants are done; the finishing is all that is left
    assert (lo, hi) == (FINISH_POURED[0] * 1000, FINISH_POURED[1] * 1000)
    est.event({"type": "progress", "pass": 1, "variant": -1, "phase": "finishing: joining pours", "done": 0, "total": 1}, end)
    assert est.status() == "finishing: joining pours"
    lo2, _, hi2 = est.left(end + 10.0)
    assert lo2 == max(lo - 10.0, 0.0) and hi2 == hi - 10.0         # it counts down from when the finishing began
    assert est.left(end + 10 * hi)[1] > 0.0                         # never says done while it is not


def test_another_heat_is_counted_in_when_every_variant_has_connections_open():
    from weaveengine.progress import Estimator
    plain, open_ = Estimator(now=0.0), Estimator(now=0.0)
    for est in (plain, open_):
        times = list(_run(est, rounds=12, complete=False))[:4]
    for v in range(4):
        open_.event({"type": "snapshot", "variant": v, "total": 50, "routed": 45}, times[-1])
    t = times[-1]
    assert plain.more(t) > 0.0 and open_.more(t) == 0.0             # counted in, so not said beside
    assert open_.left(t)[2] > plain.left(t)[2] + 0.5 * plain.more(t)
