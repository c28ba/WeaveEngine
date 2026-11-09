"""Arcs in the session: curves found in a polyline and written as ``qarc`` (at most 90 degrees each)."""
import math
import re

from bench.generators import channel
from weaveengine.cli import main
from weaveengine.io.arcs import arc_pieces, find_arcs
from weaveengine.io.dsn import write_dsn
from weaveengine.realize.smooth import _arc


def _curve(radius=2.0, turn_deg=100.0, step_deg=6.0, left=True):
    """A straight run, a rounded corner as ``smooth`` draws it, and a straight run."""
    a, b = (0.0, 0.0), (10.0, 0.0)
    t = math.radians(turn_deg) * (1 if left else -1)
    c = (b[0] + 10 * math.cos(t), b[1] + 10 * math.sin(t))
    d = radius / math.tan((math.pi - abs(t)) / 2.0)
    arc, sag = _arc(a, b, c, d)
    return [a] + arc + [c], radius, sag


def test_a_rounded_corner_is_found_as_one_arc():
    for left in (True, False):
        pts, radius, sag = _curve(left=left)
        (first, last, centre, found), = find_arcs(pts)
        assert (first, last) == (1, len(pts) - 2)
        assert math.isclose(found, radius, abs_tol=1e-6)
        assert sag < 5e-3  # the arc lies within this of the polyline it replaces
        for p in pts[first:last + 1]:
            assert math.dist(p, centre) >= radius - 1e-6
            assert math.dist(p, centre) - radius <= sag + 1e-6


def test_a_straight_run_and_a_sharp_corner_have_no_arc():
    assert find_arcs([(0, 0), (1, 0), (2, 0), (3, 0)]) == []
    assert find_arcs([(0, 0), (5, 0), (5, 5)]) == []


def test_arcs_are_cut_into_pieces_of_at_most_90_degrees():
    pts, radius, _ = _curve(turn_deg=170.0)  # an arc of 170 degrees
    (first, last, centre, found), = find_arcs(pts)
    pieces = arc_pieces(pts, first, last, centre, found)
    assert len(pieces) == 2
    assert pieces[0][1] == pieces[1][0] and pieces[0][0] == pts[first] and pieces[-1][1] == pts[last]
    for a, b in pieces:
        assert math.isclose(math.dist(a, centre), radius, abs_tol=1e-6)
        angle = abs((math.atan2(b[1] - centre[1], b[0] - centre[0]) - math.atan2(a[1] - centre[1], a[0] - centre[0]) + math.pi) % (2 * math.pi) - math.pi)
        assert angle <= math.pi / 2 + 1e-9


def _tracks(text):
    """What KiCad makes of the session: a track for each segment of a path and each qarc."""
    paths = re.findall(r"\(path \S+ \d+ ([^)]*)\)", text)
    return sum(len(p.split()) // 2 - 1 for p in paths) + text.count("(qarc ")


def _qarcs(text):
    return [tuple(map(int, m.groups()[1:])) for m in re.finditer(r"\(qarc (\S+) (\d+) (-?\d+) (-?\d+) (-?\d+) (-?\d+) (-?\d+) (-?\d+)\)", text)]


def test_the_session_holds_arcs_unless_turned_off(tmp_path):
    dsn = str(tmp_path / "b.dsn")
    write_dsn(channel(6, seed=2), dsn)
    on, off = str(tmp_path / "on.ses"), str(tmp_path / "off.ses")
    main([dsn, "-o", on, "--quiet"])
    main([dsn, "-o", off, "--quiet", "--no-ses-arcs"])
    with_arcs, without = open(on, encoding="utf-8").read(), open(off, encoding="utf-8").read()
    assert "(qarc " not in without
    from weaveengine.io.check import measure
    from weaveengine.io.dsn import read_dsn
    assert measure(read_dsn(dsn), on).ok  # read back with the arcs, the copper keeps every clearance
    assert "(qarc " in with_arcs
    assert _tracks(with_arcs) < _tracks(without)
    for width, x1, y1, x2, y2, cx, cy in _qarcs(with_arcs):
        # start and end stand at the same distance from the centre (to a unit of rounding), and are apart
        assert abs(math.hypot(x1 - cx, y1 - cy) - math.hypot(x2 - cx, y2 - cy)) <= 1.5
        assert (x1, y1) != (x2, y2)
        chord = math.hypot(x2 - x1, y2 - y1)
        assert chord <= 2.0 * math.hypot(x1 - cx, y1 - cy) * math.sin(math.pi / 4) + 2  # at most a quarter turn
