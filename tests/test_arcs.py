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


def _wrap(radius=1.0, corners=(3, 4, 5, 6), disc=0.0):
    """A trace round a round pad held as the router holds it: the pad's 16-gon,
    inflated, with the trace going round each of its corners on a disc of
    radius ``disc`` (as ``relax`` draws it) and along its sides between."""
    from weaveengine.board import circle_polygon
    ring = list(circle_polygon(0.0, 0.0, radius).exterior.coords)[:-1]
    ring = sorted(ring, key=lambda p: math.atan2(p[1], p[0]))
    pts = []
    for k in corners:
        x, y = ring[k]
        if disc == 0.0:
            pts.append((x, y))
            continue
        a = math.atan2(y, x)
        for t in (-11.25, -5.625, 0.0, 5.625, 11.25):  # (a rough disc arc: what matters is that it bulges out)
            b = a + math.radians(t)
            pts.append((x + disc * math.cos(b), y + disc * math.sin(b)))
    first, last = pts[0], pts[-1]
    # straight in and out along the tangents of the first and last sides
    a0 = math.atan2(first[1], first[0]) - math.radians(11.25)
    a1 = math.atan2(last[1], last[0]) + math.radians(11.25)
    start = (first[0] + 3 * math.sin(a0), first[1] - 3 * math.cos(a0))
    end = (last[0] - 3 * math.sin(a1), last[1] + 3 * math.cos(a1))
    return [start] + pts + [end]


def test_a_trace_round_a_polygon_pad_is_one_arc_round_its_centre():
    from weaveengine.io.arcs import fit_arcs
    for disc in (0.0, 0.3):
        pts = _wrap(disc=disc)
        runs = fit_arcs(pts, near=lambda p, d: [(0.0, 0.0)] if math.dist(p, (0.0, 0.0)) <= d else [])
        arcs = [r for r in runs if r[0] == "arc"]
        assert len(arcs) == 1, runs
        _, pieces, centre = arcs[0]
        assert centre == (0.0, 0.0)
        radius = math.dist(pieces[0][0], centre)
        # the arc keeps the run's own least distance from the centre: never nearer the pad
        nearest = min(math.dist(p, centre) for p in pts[1:-1])
        assert radius <= nearest + 1e-9 and radius >= nearest - 0.03 * radius
        assert all(math.isclose(math.dist(b, centre), radius, abs_tol=1e-9) for _, b in pieces)
        assert runs[0][1][0] == pts[0] and runs[-1][1][-1] == pts[-1]  # the ends are kept


def test_a_fitted_arc_is_turned_down_when_it_would_come_too_near():
    from weaveengine.io.arcs import fit_arcs
    pts = _wrap(disc=0.3)
    runs = fit_arcs(pts, accept=lambda *_: False, near=lambda p, d: [(0.0, 0.0)])
    # only the arcs of single discs (here none are true arcs), never one round the pad
    assert all(r[2] != (0.0, 0.0) for r in runs if r[0] == "arc")


def test_a_trace_piece_with_no_length_is_checked_as_a_point():
    """A piece of a tapered trace can start and end at one point: its outline
    is then one point, which the check takes as such (it crashed kicad-openair,
    kicad-cm5-minima and lumen-mobo)."""
    import shapely
    from shapely.geometry import Point
    from weaveengine.io.clearance import Copper
    copper = Copper.__new__(Copper)
    copper.items = {0: [[Point(5.0, 0.0), 2, 0.5, 0.2]]}
    copper.trees = {0: shapely.STRtree([copper.items[0][0][0]])}
    copper.reach = 0.7
    copper.trace_at = {(0, 0): (0, 0)}
    assert copper.allows(0, 1, 0.1, [(0.0, 0.0), (0.0, 0.0)], [(0.0, 0.0)])
    assert not copper.allows(0, 1, 0.1, [(4.5, 0.0), (4.5, 0.0)], [(4.6, 0.0)])  # nearer the pad than it was
    copper.replace(0, 0, [(5.0, 0.0)])
    assert copper.items[0][0][0].geom_type == "Point"


def test_an_oval_pad_is_measured_by_its_true_round_ends():
    from shapely.geometry import LineString
    from shapely.ops import unary_union
    from weaveengine.board import Pad, circle_polygon
    from weaveengine.io.clearance import pad_copper, round_parts
    a, b, half = (0.0, 0.0), (2.0, 0.0), 0.8
    shape = unary_union([LineString([a, b]).buffer(half, cap_style="flat", join_style="bevel"),
                         circle_polygon(*a, half), circle_polygon(*b, half)])  # as io.dsn holds an oval
    ends = round_parts(shape)
    assert sorted((round(c[0], 6), round(c[1], 6), round(r, 6)) for c, r in ends) == [(0.0, 0.0, 0.8), (2.0, 0.0, 0.8)]
    line, extent = pad_copper(Pad(0, 1, shape))
    assert line.geom_type == "LineString" and math.isclose(extent, half)
    rect = Pad.rect(1, 0.0, 0.0, 2.0, 1.0)
    assert pad_copper(rect) == (rect.shape, 0.0)
