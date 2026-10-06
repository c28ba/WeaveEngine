from tests.conftest import demo_board
from weaveengine.router import route_board
from weaveengine.viz.svg import export_result, export_svg


def test_svg_export(tmp_path):
    board = demo_board()
    result = route_board(board)
    out = tmp_path / "board.svg"
    export_svg(board, str(out), result.polylines, result.wire_net, pmap=result.pmap)
    text = out.read_text()
    assert text.startswith("<svg") and text.rstrip().endswith("</svg>")
    assert text.count("<title>wire") == 3
    assert text.count("<title>pad") == 6
    export_result(result, str(out))
    text = out.read_text()
    assert text.count("<title>wire") == 3 and text.count("<polygon") >= 1 + 1 + 6 + 6  # outline, keepout, pads, teardrops


def _run(est, rounds, at=0.0, heat="racing variants 1-4 of 8", complete=True):
    """Feeds an estimator one heat as the router reports it: every variant
    takes 4 s to reach rip-up, then ``rounds`` rounds of 1 s (the last 8
    without improvement), 2 s of geometry and one repair of 3 s. Yields the
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
        say(end + 2.0, v, "design-rule repair", 0, 4)
        say(end + 2.0, v, "rip-up", 0, 5, round=rounds + 1, quiet=3, most=5)
    yield end + 2.0
    for v in variants:
        say(end + 5.0, v, "finished", complete=complete)
    yield end + 5.0


def test_time_left_is_a_range_that_holds_and_narrows():
    from weaveengine.progress import Estimator, clock, phrase
    assert clock(75) == "1:15" and clock(3725) == "1:02:05"
    est = Estimator(now=0.0)
    total = 4.0 + 20 + 5.0
    shares, widths = [], []
    for t in _run(est, rounds=20):
        if est.left(t) is None:
            assert t <= 2.0                                         # nothing to go on before commit is under way: it says nothing
            continue
        lo, hi = est.left(t)
        assert lo <= total - t + 1e-9 <= hi + 1e-9, (t, lo, hi)     # the run ends inside the range, all the way through
        shares.append(est.progress(t))
        widths.append(hi - lo)
    assert shares == sorted(shares) and shares[-1] > 0.95           # the bar never moves backwards, and ends at the end
    assert widths[-1] < 1.0 < widths[3] < widths[1]                 # the range narrows
    assert est.more(total) == 0.0                                   # a variant connected everything: no other heat
    assert phrase((50.0, 60.0)) == "about 0:55 left" and phrase((10.0, 90.0)) == "about 0:30 left (0:10 to 1:30)"
    assert "more if no variant connects everything" in phrase((10.0, 12.0), 30.0)


def test_another_heat_is_said_beside_the_range():
    from weaveengine.progress import Estimator
    est = Estimator(now=0.0)
    said = [(t, est.more(t)) for t in _run(est, rounds=12, complete=False)]
    end = said[-1][0]
    assert all(0.5 * end < more < 2.0 * end for t, more in said[4:-1]) and said[0][1] == 0.0  # while it may follow: about what this heat takes
    for n, t in enumerate(_run(est, rounds=12, at=end, heat="racing variants 5-8 of 8", complete=False)):
        assert est.more(t) == 0.0                                   # the last heat: nothing can follow
        if n == 5:
            lo, hi = est.left(t)
            assert lo <= 2 * end - t <= hi
            assert est.status().startswith("variants 5 to 8")


def test_a_run_on_record_is_estimated_by_the_time_it_took(tmp_path):
    from weaveengine.progress import Estimator, Timings
    board = tmp_path / "board.dsn"
    board.write_text("(pcb)")
    timings = Timings(str(tmp_path / "timings.json"))
    key = Timings.key(str(board), {"seed": 0})
    assert timings.last(key) is None and key != Timings.key(str(board), {"seed": 1})
    timings.record(key, 100.0)
    est = Estimator(now=0.0, last=timings.last(key))
    lo, hi = est.left(40.0)
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

