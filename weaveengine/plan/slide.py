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

import numpy as np
from shapely.geometry import LineString, Point

from weaveengine.plan.path import find, locate
from weaveengine.plan.ripup import via_worth

ROUNDS = 3                        # a via travels a part of its triangles per round
STEPS = (1.0, 0.5, 0.25, 0.125)   # of the way to where it wants to be: the furthest that is allowed
SMALL = 0.02                      # mm: nearer than this to where it wants to be, it stays
STRAIGHT = math.radians(2.0)      # a run is straight while it turns by less than this
TAKE_BACK = 3                     # times the check may object to a round before the whole round is undone

SHARP = 100.0                     # degrees: a via whose two traces meet at less than this is tried elsewhere
AIM = 0.75                        # mm round a place aimed at: the via may stand in any triangle that reaches into it
STUB = 4.0                        # mm: a trace this short between a pad and its via is a fan-out stub
TRIES = 4                         # places tried per via, the best expected first
GAIN = 0.05                       # mm: a new place is kept if the copper is shorter by this at least


def reseat(ctx, lines: dict, violations: list, wire_net: dict, realize):
    """Vias whose traces meet sharply, planned again at better places (13.3a).

    Sliding moves a via within the triangles round it. A via whose traces
    make a V cannot get to where they would part that way: other traces lie
    between. Here its connection is planned again from nothing with that via
    held to a small area round a better place (where its traces would run
    straight through it, back along either of them, or, for a short stub
    from a pad, round the pad), its other vias to where they are. The search
    measures through the middles of gates, so it would not choose such a
    place itself, and cannot judge it: the board is drawn and the new place
    kept if the copper, vias counted at their worth, is shorter and the check no worse; otherwise
    everything is as it was. Returns (lines, violations, wire_net)."""
    worth, pads = via_worth(ctx), {p.pad_id: p for p in ctx.board.pads}
    sharp = [(w, via) for w, via, _, _, into, out in _bends(ctx, lines) if _angle(into, out) < SHARP]
    for w, via in sharp:
        bend = next((b for b in _bends(ctx, lines) if b[:2] == (w, via)), None)
        if bend is None:
            continue  # gone with another via of its connection
        conn, (_, _, a, b, into, out) = ctx.conns[w], bend
        others = [ctx.pad_centre(v) for v in conn.sites if v != via]
        most = len(conn.sites)
        was = _copper(lines) + worth * _vias(ctx)  # a via counted as the track it takes away (``ripup.via_worth``)
        for point in _places(ctx, pads, into, out, ctx.conns[a].src, ctx.conns[b].dst)[:TRIES]:
            before = ctx.snapshot()
            ctx.lift(conn)
            plan = find(ctx, conn, hard_cap=True, congestion=False, max_vias=most, via_cost=worth, through=False,
                        within=_near(ctx, [point, *others]))
            if plan is not None and ctx.commit(conn, plan):
                new = realize(ctx)
                if len(new[1]) <= len(violations) and _copper(new[0]) + worth * _vias(ctx) < was - GAIN:
                    lines, violations, wire_net = new
                    break
            ctx.restore(before)
    return lines, violations, wire_net


def _vias(ctx) -> int:
    return sum(1 for s in ctx.layers[0].pmap.sites.values() if s.active)


def _bends(ctx, lines: dict):
    """(connection, via, piece into it, piece out of it, their lines) at each
    via a connection changes layer at."""
    site = ctx.layers[0].pmap.sites
    for conn in ctx.conns.values():
        if conn.parent is not None:
            continue
        for a, b in zip(conn.pieces, conn.pieces[1:]):
            via = ctx.conns[a].dst
            if via in site and a in lines and b in lines and math.dist(lines[a][-1], lines[b][0]) < 1e-6:
                yield conn.wire_id, via, a, b, lines[a], lines[b]


def _leaving(line) -> tuple[float, float]:
    """The direction ``line`` leaves its first point in."""
    for q in line[1:]:
        d = math.dist(line[0], q)
        if d > 1e-6:
            return ((q[0] - line[0][0]) / d, (q[1] - line[0][1]) / d)
    return (1.0, 0.0)


def _angle(into, out) -> float:
    """The angle in degrees between the traces at a via (180: straight through)."""
    u, v = _leaving(into[::-1]), _leaving(out)
    return math.degrees(math.acos(max(-1.0, min(1.0, u[0] * v[0] + u[1] * v[1]))))


def _length(line) -> float:
    return sum(math.dist(p, q) for p, q in zip(line, line[1:]))


def _along(line, dist: float) -> tuple[float, float]:
    """The point ``dist`` along ``line`` from its start."""
    for p, q in zip(line, line[1:]):
        step = math.dist(p, q)
        if dist <= step:
            f = dist / step if step else 0.0
            return (p[0] + f * (q[0] - p[0]), p[1] + f * (q[1] - p[1]))
        dist -= step
    return line[-1]


def _places(ctx, pads: dict, into, out, start: int, end: int) -> list:
    """Where a via with the traces ``into`` and ``out`` would bend them less,
    best expected first; only places that make the way from corner to corner
    shorter. A short stub from a pad (``start`` or ``end``): round the pad,
    where a via is legal. Otherwise the nearest point of the straight line
    between the traces' corners, and points back along either trace."""
    here = into[-1]
    for stub, pad, other in ((into, start, out), (out[::-1], end, into[::-1])):
        if pad in pads and _length(stub) < STUB:
            far, centre = _corner(other), pads[pad].centre
            now = _length(stub) + math.dist(here, far)
            ring = [q for q in _round(ctx, pads[pad]) if math.dist(q, here) > 0.3 and ctx.legal.exactly(*q)]
            return sorted((q for q in ring if math.dist(centre, q) + math.dist(q, far) < now - 0.1),
                          key=lambda q: math.dist(centre, q) + math.dist(q, far))
    a, b = _corner(into), _corner(out[::-1])
    points = []
    span = math.dist(a, b)
    if span > 1e-9:
        line = LineString([a, b])
        p = line.interpolate(line.project(Point(here)))
        points.append((p.x, p.y))
    for leg, corner in ((into[::-1], a), (out, b)):
        reach = 0.0
        for p, q in zip(leg, leg[1:]):
            reach += math.dist(p, q)
            if math.dist(q, corner) < 1e-9:
                break
        points += [_along(leg, part * reach) for part in (1.0, 0.75, 0.5, 0.25)]
    now = math.dist(a, here) + math.dist(here, b)
    scored = sorted((math.dist(a, p) + math.dist(p, b), p) for p in points if math.dist(p, here) > 0.3)
    return [p for cost, p in scored if cost < now - 0.1]


def _round(ctx, pad) -> list:
    """24 points round ``pad``, each as near it as a via may stand."""
    rules = ctx.board.rules
    keep = rules.via_diameter / 2.0 + rules.clearance + 0.05
    (cx, cy), out = pad.centre, []
    for i in range(24):
        dx, dy, t = math.cos(math.pi * i / 12), math.sin(math.pi * i / 12), 0.0
        while t < 10.0 and pad.shape.distance(Point(cx + t * dx, cy + t * dy)) < keep:
            t += 0.05
        out.append((cx + t * dx, cy + t * dy))
    return out


def _near(ctx, points) -> dict:
    """Per layer, the triangles that reach within ``AIM`` of any of ``points``."""
    offsets = [(dx, dy) for dx in np.arange(-AIM, AIM + 1e-9, 0.25) for dy in np.arange(-AIM, AIM + 1e-9, 0.25)
               if dx * dx + dy * dy <= AIM * AIM + 1e-9]
    grid = np.array([(x + dx, y + dy) for x, y in points for dx, dy in offsets])
    return {layer.index: {int(t) for t in locate(layer.pmap, grid) if t >= 0} for layer in ctx.layers}


def slide(ctx, lines: dict, violations: list, wire_net: dict, realize):
    """Moves the vias to where their traces run straighter, while that makes
    the board's copper shorter and the check no worse. ``realize(ctx)`` gives
    the geometry of the routing as it stands. Returns (lines, violations,
    wire_net) of the result."""
    # A via's two traces: a piece that ends on it and the next of its connection (of one connection, if several share it).
    site = ctx.layers[0].pmap.sites
    legs = {ctx.conns[a].dst: (a, b) for conn in ctx.conns.values() for a, b in zip(conn.pieces, conn.pieces[1:]) if ctx.conns[a].dst in site}
    held: set[int] = set()   # vias the check has sent back once: they stay
    for _ in range(ROUNDS if legs else 0):
        was: dict[int, tuple[float, float]] = {}
        for pad, (a, b) in legs.items():
            if pad in held:
                continue
            here = ctx.pad_centre(pad)
            want = _wanted(here, lines.get(a), lines.get(b))
            if want is not None and any(ctx.move_via(pad, (here[0] + part * (want[0] - here[0]), here[1] + part * (want[1] - here[1])))
                                        for part in STEPS):
                was[pad] = here
        if not was:
            break
        new = realize(ctx)
        for _ in range(TAKE_BACK):
            if len(new[1]) <= len(violations) or not was:
                break
            # Take back the vias that the traces the check names go past (or,
            # failing that, the nearest), and look again.
            back = [pad for pad in was if any(_beside(ctx, pad, legs[pad]).intersection(v.wires) for v in new[1])]
            back = back or [min(was, key=lambda pad: math.dist(ctx.pad_centre(pad), v.at)) for v in new[1]]
            for pad in set(back):
                ctx.move_via(pad, was.pop(pad), sure=True)
                held.add(pad)
            new = realize(ctx)
        if len(new[1]) > len(violations) or _copper(new[0]) > _copper(lines) - 1e-6:
            for pad, point in was.items():   # this round has not helped: as before it, and no further
                ctx.move_via(pad, point, sure=True)
            return realize(ctx) if was else new
        lines, violations, wire_net = new
    return lines, violations, wire_net


def _beside(ctx, pad: int, legs) -> set[int]:
    """The wires a via's place matters to: its own two, and those that cross a gate at it on any layer."""
    return set(legs).union(w for layer in ctx.layers for e in layer.pmap.sites[pad].spokes for w in layer.state.gate_order[e])


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
