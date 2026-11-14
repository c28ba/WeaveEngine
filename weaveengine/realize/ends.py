"""Traces of one net that ride together into a pad drawn as one there.

A connection that reaches its net's copper rides it to the pad (design 11):
two wires, one trace. Each wire is drawn on its own, though (its corners
rounded, its acute meetings blunted), so near a pad they share, two such
wires can take two ways from the pad to the point where they meet: a thin
wedge of bare board between copper of one net, an acid trap, and two
teardrops crossing. Here, where two wires of a net leave a pad on one layer
and meet again a little way out, staying near each other all the way (no
foreign copper can lie between them), the one with the longer way there
takes the shorter one's, so they are one trace from the pad. The taken way
is a drawn trace at least as wide, so every clearance it keeps the other
keeps too. Each wire still joins the same two pads, from their centres: it
is drawing, as rounding corners is (design 13.3c).
"""
import math

from shapely.geometry import LineString, Point
from shapely.ops import substring

STEP = 0.02    # mm between the points at which two wires are compared
REACH = 5.0    # mm from the pad within which they are compared
ON = 0.25      # a wire is back on another when nearer than this share of half their widths together


def _from(line: list, at) -> tuple[list, bool]:
    """The polyline as it leaves ``at`` (its first point there), and whether it was turned round for that."""
    if math.dist(line[0], at) < 1e-6:
        return list(line), False
    return list(reversed(line)), True


def _meet(a: LineString, b: LineString, gap: float, near: float, pad) -> float | None:
    """How far along ``a`` it is back on ``b`` (nearer than ``ON``) after
    the last stretch outside the pad with bare board between them (further
    apart than ``gap``) that they come back together after, staying within
    ``near`` of it all the way from the pad; None if there is none (inside the
    pad it is all copper)."""
    apart, back = False, None
    for k in range(1, int(min(REACH, a.length) / STEP) + 1):
        q = a.interpolate(k * STEP)
        d = b.distance(q)
        if d >= near:
            break
        if d > gap and not pad.contains(q):
            apart = True
        elif apart and d < ON * gap:  # (a gap counts once they are back together after it; after the last, they branch)
            apart, back = False, k * STEP
    return back


def _start(line: LineString, length: float) -> list:
    return list(substring(line, 0.0, length).coords)


def _rest(line: LineString, length: float) -> list:
    return list(substring(line, length, line.length).coords)


def _sharper(board, lines: dict, new: list, w: int, wire_net: dict, wire_layer: dict, ends: dict, least: float) -> bool:
    """Whether ``w`` drawn as ``new`` leaves its net meeting itself under ``least`` more often on its layer."""
    from weaveengine.realize.acute import net_meetings
    layer, net = wire_layer[w], wire_net[w]
    mine = {x: line for x, line in lines.items() if wire_layer.get(x) == layer and wire_net.get(x) == net and len(line) > 1}
    here = board.pads_on(layer)
    before = net_meetings(board, layer, net, mine, ends, here, least=least)
    after = net_meetings(board, layer, net, {**mine, w: new}, ends, here, least=least)
    return len(after) > len(before)


def merge(board, lines: dict, wire_net: dict, wire_layer: dict, ends: dict, skip=frozenset(),
          least: float | None = None) -> tuple[dict, int]:
    """``lines`` (wire -> polyline) with every pair of wires of one net that
    ride together into a pad they share on one layer drawn as one trace from
    it (see the module). ``ends``: wire -> its two pads. ``skip``: wires not to
    touch. ``least``: if given, a way is taken only if the net then meets
    itself under that angle no more often on that layer (``realize.acute``):
    the shorter way first, else the longer (the way blunting drew square into
    the pad is often the longer). Returns the new lines and how many wire ends
    took another's way."""
    pads = {p.pad_id: p for p in board.pads}
    rules = board.rules
    at: dict[tuple[int, int], list[int]] = {}
    for w in sorted(ends):
        if w in lines and w in wire_layer and w not in skip:
            for p in ends[w]:
                at.setdefault((p, wire_layer[w]), []).append(w)
    out = dict(lines)
    merged = 0
    for (p, _), ws in sorted(at.items()):
        pad = pads.get(p)
        if pad is None or len(ws) < 2:
            continue
        inside = pad.shape.buffer(1e-6)
        changed, rounds = True, 0
        while changed and rounds < 8:  # (three wires into one pad: each pair in turn, until none moves)
            changed, rounds = False, rounds + 1
            for i in range(len(ws)):
                for j in range(i + 1, len(ws)):
                    a, b = ws[i], ws[j]
                    if wire_net[a] != wire_net[b]:
                        continue
                    pa, fa = _from(out[a], pad.centre)
                    pb, fb = _from(out[b], pad.centre)
                    if math.dist(pa[0], pad.centre) > 1e-6 or math.dist(pb[0], pad.centre) > 1e-6:
                        continue  # (an end that does not start at the pad's centre: not ours to move)
                    la, lb = LineString(pa), LineString(pb)
                    wa, wb = rules.drawn(wire_net[a]), rules.drawn(wire_net[b])
                    gap, near = (wa + wb) / 2.0, (wa + wb) / 2.0 + rules.clearance
                    sa = _meet(la, lb, gap, near, inside)
                    if sa is not None:
                        sb = lb.project(la.interpolate(sa))
                    else:
                        sb = _meet(lb, la, gap, near, inside)
                        if sb is None:
                            continue
                        sa = la.project(lb.interpolate(sb))
                    if la.interpolate(sa).distance(lb.interpolate(sb)) >= ON * gap:
                        continue
                    # The shorter way is the one both take, if it is no narrower than the wire that takes it.
                    ways = sorted([(sa, a, la, wa, b, lb, sb, wb, fb), (sb, b, lb, wb, a, la, sa, wa, fa)], key=lambda x: (x[0], x[1]))
                    for length, _src, src, width, dst, dline, dlength, dwidth, flip in ways:
                        if width + 1e-9 < dwidth:
                            continue
                        # (From where the way it takes ends, a step to where its own goes on: under a quarter width, in copper.)
                        way = _start(src, length) + _rest(dline, dlength)
                        new = [tuple(q) for k, q in enumerate(way) if k == 0 or math.dist(q, way[k - 1]) > 1e-9]
                        new = new[::-1] if flip else new
                        if least is not None and _sharper(board, out, new, dst, wire_net, wire_layer, ends, least):
                            continue
                        out[dst] = new
                        merged += 1
                        changed = True
                        break
    return out, merged
