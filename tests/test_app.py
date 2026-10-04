"""The desktop application, driven without a person (Qt's offscreen platform)."""
import os
import subprocess
import sys

import pytest

pytest.importorskip("PySide6")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def run_app(*args, timeout=240):
    env = dict(os.environ, QT_QPA_PLATFORM="offscreen")
    return subprocess.run([sys.executable, os.path.join(ROOT, "main.py"), *args], cwd=ROOT, env=env,
                          capture_output=True, text=True, timeout=timeout)


def test_app_opens_routes_shows_and_exports(tmp_path):
    from tests.conftest import demo_board
    from weaveengine.io.check import read_session
    from weaveengine.io.dsn import write_dsn
    dsn, ses, png = (str(tmp_path / n) for n in ("board.dsn", "board.ses", "shot.png"))
    write_dsn(demo_board(), dsn)
    done = run_app("--self-test", dsn, "--ses", ses, "--screenshot", png)
    out = done.stdout
    assert done.returncode == 0, out + done.stderr
    assert "kernels:" in out                                  # the compiled-kernel status is shown before routing
    assert "routed 3/3" in out and "live picture shown: True" in out
    assert "-> OK" in out                                     # the exported session measures clean
    # view controls: zoom stays on the point under the cursor, in small steps, within limits; pan is exact
    assert "point under cursor drifted 0.00 px" in out and "pan error 0.00 px" in out
    assert "zoom limits 0.25x .. 400x of fit" in out
    # selection: a pad and a trace report what they are
    assert "'kind': 'pad'" in out and "'kind': 'trace'" in out and "'length':" in out and "'net total':" in out
    assert len(read_session(ses)[0]) >= 3
    assert os.path.getsize(png) > 10_000                      # a real window was drawn
    for tab in range(1, 5):                                   # the four tabs of the settings editor
        assert os.path.getsize(os.path.splitext(png)[0] + f"_settings{tab}.png") > 5_000


def test_app_warns_when_kernels_are_unavailable(tmp_path):
    from tests.conftest import demo_board
    from weaveengine.io.dsn import write_dsn
    dsn = str(tmp_path / "board.dsn")
    write_dsn(demo_board(), dsn)
    env = dict(os.environ, QT_QPA_PLATFORM="offscreen", WEAVEENGINE_NO_NUMBA="1")
    done = subprocess.run([sys.executable, os.path.join(ROOT, "main.py"), "--self-test", dsn], cwd=ROOT, env=env,
                          capture_output=True, text=True, timeout=240)
    assert done.returncode == 0, done.stdout + done.stderr   # no crash: it routes on the fallback
    assert "WARNING: running without the compiled kernels" in done.stdout
    assert "routed 3/3" in done.stdout


def test_app_reports_a_bad_file_without_crashing(tmp_path):
    bad = tmp_path / "bad.dsn"
    bad.write_text("this is not a board")
    done = run_app("--self-test", str(bad))
    assert done.returncode == 2 and "could not open" in done.stdout
