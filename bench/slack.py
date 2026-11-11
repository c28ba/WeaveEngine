"""Slack left in drawn traces (ROADMAP 1.1, "Traces pulled fully taut").

A trace is the taut line against discs at the vertices it passes (design
13.1). This routes a board and asks, for every drawn trace, whether it bends
round something that is not copper on the board:

- *by cause*: the trace is pulled again with one kind of thing taken out of
  its layer, and every trace that comes out shorter by more than ``TOL`` had
  that much slack because of it. The kinds: via sites that are asleep (no
  via), via sites that are vias of no drawn trace, the pours' skeletons (not
  drawn: they hold room for the fill, on purpose), and wires that are not
  drawn because their net joins those pads already (``router._redundant``).
- *by geometry*, independent of the map: every corner of a drawn trace (as
  pulled, before corners are rounded) that turns by more than a degree must
  have something drawn within its clearance of it: a foreign pad, via,
  keepout or trace, or the board edge. A corner with nothing there is listed
  with how far the nearest thing is beyond its clearance.

    python bench/slack.py BOARD.dsn [weaveengine.cli options]

It routes with one worker and the plain variant (``--workers 1 --portfolio 1``)
unless told otherwise, so that the routing can be looked into afterwards.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import shapely  # noqa: E402
from shapely.geometry import LineString, Point  # noqa: E402

from weaveengine import cli, router  # noqa: E402
from weaveengine.geom.inflate import MITRE_LIMIT  # noqa: E402
from weaveengine.realize import drc  # noqa: E402
from weaveengine.realize.relax import polyline_length, realize  # noqa: E402
from weaveengine.topo import sites  # noqa: E402

TOL = 0.005          # mm of slack that counts
TURN = math.radians(1.0)
NEAR = 0.03          # mm beyond its clearance that a corner's nearest copper may be (a rule from .kicad_dru, a pad polygon)
EDGE_MITRE = 5.0     # the board edge is inset with shapely's own mitre limit


def route(argv: list[str]):
    """Routes as the CLI does; returns (the routing's context, its result, the
    traces as pulled: before their corners are rounded)."""
    got: dict = {}
    realize, write, drawn = router._realize, cli.write_ses, router._drawn

    def spy(ctx):
        got["ctx"] = ctx
        return realize(ctx)

    def pulled(ctx, work, lines, *args):
        got["lines"] = lines  # (the last call is the routing's own; the earlier ones are drafts)
        return drawn(ctx, work, lines, *args)

    def keep(design, result, *args, **kw):
        got["result"] = result
        return write(design, result, *args, **kw)

    router._realize, cli.write_ses, router._drawn = spy, keep, pulled
    try:
        cli.main(argv)
    finally:
        router._realize, cli.write_ses, router._drawn = realize, write, drawn
    return got["ctx"], got["result"], got["lines"]


def _lengths(lines: dict) -> dict:
    return {w: polyline_length(line) for w, line in lines.items()}


def by_cause(ctx, result, pulled: dict) -> dict:
    """Per kind of thing that is not copper: [traces with slack, total slack,
    largest, DRC violations once it is gone, things taken out], over all
    layers. The kinds are taken out one after the other, each on top of the
    ones before, so each is credited with what it adds, against the traces
    as the router pulled them. (The routing is not usable afterwards.)"""
    drawn = set(result.polylines)
    used = {p for w in drawn if w in ctx.conns for p in (ctx.conns[w].src, ctx.conns[w].dst)}
    board = ctx.board_with_vias()
    kinds = ("redundant", "unused via", "asleep", "skeleton")  # (the redundant first, as the router takes them out: ``router._taut``)
    out = {kind: [0, 0.0, 0.0, 0, 0] for kind in kinds}
    for layer in ctx.layers:
        pmap, state = layer.pmap, layer.state
        wire_net = {w: ctx.conns[w].net_id for w in state.wire_path}
        mine = [w for w in state.wire_path if w in drawn]
        before = _lengths({w: pulled[w] for w in mine})
        for kind in kinds:
            row = out[kind]
            if kind == "asleep":
                for site in [s for s in pmap.sites.values() if not s.active]:
                    row[4] += sites.delete(pmap, state, site)
            elif kind == "unused via":
                for site in [s for s in pmap.sites.values() if s.active and s.pad not in used]:
                    for w in {w for e in site.hole for w in state.gate_order[e]}:
                        state.remove(w)
                    sites.set_net(pmap, state, site, -1)
                    row[4] += sites.delete(pmap, state, site)
            else:
                for w in list(state.wire_path):
                    c = ctx.conns.get(w)
                    if c is not None and w not in drawn and (kind == "skeleton") == bool(c.skeleton):
                        state.remove(w)
                        row[4] += 1
            lines = realize(state, board)[0]
            after = _lengths(lines)
            for w in mine:
                d = before[w] - after.get(w, before[w])
                if d > TOL:
                    row[0] += 1
                    row[1] += d
                    row[2] = max(row[2], d)
            row[3] += len(drc.check(board, {w: lines[w] for w in mine if w in lines}, wire_net, layer.index))
            before = after
    return out


def by_geometry(ctx, result, routing_lines: dict) -> list[tuple]:
    """Corners of drawn traces (as pulled) with nothing drawn within their
    clearance: (wire, layer, x, y, how far beyond the clearance the nearest
    copper is, what not drawn holds it there: "skeleton", "redundant",
    "asleep", "unused via", "own pad" or "nothing")."""
    board, rules = result.board, result.board.rules
    wide = ctx.board.rules  # the routing's (the skeletons' widths are in them)
    used = {p for w in result.polylines if w in ctx.conns for p in (ctx.conns[w].src, ctx.conns[w].dst)}
    pads = {p.pad_id: p for p in board.pads}
    found = []
    for layer in range(len(board.layers)):
        wires = [w for w, li in result.wire_layer.items() if li == layer and w in routing_lines]
        if not wires:
            continue
        copper = [(p.shape, p.net_id, rules.apart(p.clearance)) for p in board.pads_on(layer)]
        copper += [(o.shape, o.net_id, rules.apart(o.clearance)) for o in board.obstacles_on(layer)]
        traces = {w: LineString(routing_lines[w]) for w in wires if len(routing_lines[w]) > 1}
        edge = board.outline.boundary
        for w in wires:
            line, net, half = routing_lines[w], result.wire_net[w], rules.width(result.wire_net[w]) / 2.0
            for a, b, c in zip(line, line[1:], line[2:]):
                ux, uy, vx, vy = b[0] - a[0], b[1] - a[1], c[0] - b[0], c[1] - b[1]
                if math.hypot(ux, uy) < 1e-9 or math.hypot(vx, vy) < 1e-9:
                    continue
                turn = abs(math.atan2(ux * vy - uy * vx, ux * vx + uy * vy))
                if turn < TURN:
                    continue
                p = Point(b)
                # (The map's copper is inflated with mitred corners, standing up to MITRE_LIMIT
                # times the inflation out from a corner: a trace round such a corner keeps that.)
                best = p.distance(edge) - (rules.outline_inset - rules.base_width / 2.0) * EDGE_MITRE - half
                for shape, other, apart in copper:
                    if other < 0 or other != net:
                        best = min(best, p.distance(shape) - (apart + rules.base_width / 2.0) * MITRE_LIMIT - (half - rules.base_width / 2.0))
                for x, other in traces.items():
                    if x != w and result.wire_net[x] != net:
                        best = min(best, p.distance(other) - rules.clearance - half - rules.width(result.wire_net[x]) / 2.0)
                if best > NEAR:
                    found.append((w, layer, b[0], b[1], best, _holder(ctx, layer, w, p, wide, used, pads, routing_lines)))
    return found


def _holder(ctx, layer: int, w: int, p, rules, used: set, pads: dict, drawn: dict) -> str:
    """What that is not drawn keeps the corner of wire w at p where it is."""
    net, half = ctx.conns[w].net_id, rules.width(ctx.conns[w].net_id) / 2.0
    c = ctx.conns[w]
    for pad in (c.src, c.dst):
        if pad in pads and p.distance(pads[pad].shape) < rules.pitch + half:
            return "own pad"
    every = ctx.__dict__.get("_slack_lines", {})
    for x, line in every.items():
        other = ctx.conns.get(x)
        if x == w or other is None or other.layer != layer or other.net_id == net or x in drawn or len(line) < 2:
            continue
        if p.distance(LineString(line)) < rules.clearance + half + rules.width(other.net_id) / 2.0 + NEAR:
            return "skeleton" if other.skeleton else "redundant"
    pmap = ctx.layers[layer].pmap
    for site in pmap.sites.values():
        reach = (site.keep if site.active else sites.asleep_keep(pmap)) + half + NEAR
        if math.dist(site.centre, (p.x, p.y)) < reach:
            if not site.active:
                return "asleep"
            if site.pad not in used:
                return "unused via"
    return "nothing"


def main(argv: list[str]) -> int:
    if not any(a.startswith("--workers") for a in argv):
        argv += ["--workers", "1"]
    if not any(a.startswith("--portfolio") for a in argv):
        argv += ["--portfolio", "1"]
    ctx, result, pulled = route(argv)
    ctx._slack_lines = result.kept["lines"]  # every wire as pulled, drawn or not
    drawn = {w: line for w, line in pulled.items() if w in result.polylines}
    corners = by_geometry(ctx, result, drawn)
    print(f"slack: {len(result.polylines)} traces drawn; {sum(not s.active for s in ctx.layers[0].pmap.sites.values())} via sites asleep, "
          f"{sum(s.active for s in ctx.layers[0].pmap.sites.values())} vias")
    held: dict = {}
    for corner in corners:
        held[corner[5]] = held.get(corner[5], 0) + 1
    print(f"slack: {len(corners)} corners with nothing drawn within their clearance: "
          + ", ".join(f"{n} by {what}" for what, n in sorted(held.items()))
          + "".join([f"\n  wire {w} layer {li} at ({x:.3f}, {y:.3f}): nearest copper {d:.3f} mm beyond; {what}"
                     for w, li, x, y, d, what in sorted(corners, key=lambda c: -c[4]) if what not in ("skeleton", "own pad")][:15]))
    for kind, (n, total, most, bad, gone) in by_cause(ctx, result, pulled).items():
        print(f"slack: {kind}: {gone} taken out; {n} traces shorter by > {TOL} mm, {total:.3f} mm in all, at most {most:.3f}; "
              f"{bad} violations without them")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
