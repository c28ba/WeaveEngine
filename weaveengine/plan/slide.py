"""Sliding vias (design section 13.3).

Where a via stands is part of the geometry, not of the topology: moved a
little, it leaves every wire on the gates it crosses, in the same order. The
search cannot place it well, because it measures through the middles of gates;
so it is placed here, from the traces as they really run.

Each via steps towards the straight line between the corners its two traces
make either side of it: there they would run through it without a bend. The
step is made in the maps (``Context.move_via``), the traces are pulled taut
again, all of them, since those that go round the via move with it, and the
check is run. A step the check objects to is taken back.
"""
import math

from shapely.geometry import LineString, Point

ROUNDS = 4                        # a via travels a part of its triangles per round
STEPS = (1.0, 0.5, 0.25, 0.125)   # of the way to where it wants to be: the furthest that is allowed
SMALL = 0.02                      # mm: nearer than this to where it wants to be, it stays
STRAIGHT = math.radians(2.0)      # a run is straight while it turns by less than this
NEAR = 4.0                        # via pitches round a violation within which the vias moved are taken back


def slide(ctx, lines: dict, violations: list, wire_net: dict, realize):
    """Moves the vias to where their traces run straighter, while that makes
    the board's copper shorter and the check no worse. ``realize(ctx)`` gives
    the geometry of the routing as it stands. Returns (lines, violations,
    wire_net) of the result."""
    legs = {pad: (conn.pieces[i], conn.pieces[i + 1])
            for conn in ctx.conns.values() if conn.parent is None for i, pad in enumerate(conn.sites)}
    rules = ctx.board.rules
    reach = NEAR * (rules.via_diameter + rules.clearance)
    for _ in range(ROUNDS if legs else 0):
        was: dict[int, tuple[float, float]] = {}
        for pad, (a, b) in legs.items():
            here = ctx.pad_centre(pad)
            want = _wanted(here, lines.get(a), lines.get(b))
            if want is not None and any(ctx.move_via(pad, (here[0] + part * (want[0] - here[0]), here[1] + part * (want[1] - here[1])))
                                        for part in STEPS):
                was[pad] = here
        if not was:
            break
        new = realize(ctx)
        if len(new[1]) > len(violations):
            # Take back the vias next to what the check found, and look again.
            back = [pad for pad in was if any(math.dist(ctx.pad_centre(pad), v.at) < reach for v in new[1])]
            for pad in back:
                ctx.move_via(pad, was.pop(pad), sure=True)
            new = realize(ctx)
        if len(new[1]) > len(violations) or _copper(new[0]) > _copper(lines) - 1e-6:
            for pad, point in was.items():   # this round has not helped: as before it, and no further
                ctx.move_via(pad, point, sure=True)
            return realize(ctx) if was else new
        lines, violations, wire_net = new
    return lines, violations, wire_net


def _copper(lines: dict) -> float:
    return sum(math.dist(p, q) for line in lines.values() for p, q in zip(line, line[1:]))


def _wanted(here, first, second) -> "tuple[float, float] | None":
    """Where a via at ``here`` would stand for its two traces to run straight
    through it: the nearest point of the line between the corner of the trace
    that ends on it and the corner of the trace that starts from it. None if
    it is there already, or the traces are not both there."""
    if not first or not second or math.dist(first[-1], here) > 1e-6 or math.dist(second[0], here) > 1e-6:
        return None
    a, b = _corner(first), _corner(second[::-1])
    if math.dist(a, b) < 1e-9:
        return None
    line = LineString([a, b])
    want = line.interpolate(line.project(Point(here)))
    return (want.x, want.y) if math.dist((want.x, want.y), here) >= SMALL else None


def _corner(points) -> tuple[float, float]:
    """Where the straight run that ends at the last of ``points`` begins."""
    i = len(points) - 1
    while i > 0 and math.dist(points[i - 1], points[i]) < 1e-9:
        i -= 1
    if i == 0:
        return points[0]
    heading = math.atan2(points[i][1] - points[i - 1][1], points[i][0] - points[i - 1][0])
    j = i - 1
    while j > 0:
        if math.dist(points[j - 1], points[j]) > 1e-9:
            turn = math.atan2(points[j][1] - points[j - 1][1], points[j][0] - points[j - 1][0]) - heading
            if abs((turn + math.pi) % (2 * math.pi) - math.pi) > STRAIGHT:
                break
        j -= 1
    return points[j]
