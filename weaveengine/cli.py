"""Command line: read a Specctra DSN, route it, write a SES (and optionally an SVG)."""
import argparse
import math
import os
import sys
import time

from weaveengine import __version__, accel, safe_output
from weaveengine.io.check import kicad_project_rules, measure
from weaveengine.io.dsn import read_dsn, walled_note
from weaveengine.io.ses import write_ses
from weaveengine.pours import note as pour_note, of_result as estimate_pours
from weaveengine.plan.context import Options
from weaveengine.progress import ProgressBar
from weaveengine.router import route_board
from weaveengine.settings import skip_nets
from weaveengine.topo import planar_map
from weaveengine.topo.costs import CostParams
from weaveengine.viz.svg import export_result


WHY_BURIED = {
    "outside": "lie outside the board edge (a part not yet placed on the board?)",
    "edge": "sit closer to the board edge than the edge clearance allows",
    "keepout": "are inside a keepout",
    "crowded": "sit closer to other copper than the clearance and the narrowest trace allow (a fine-pitch part, "
               "routed in KiCad under a custom rule, which WeaveEngine does not read)",
}


def why_buried(board, pad) -> str:
    """Why no trace reaches ``pad``: a key of ``WHY_BURIED``."""
    if not board.outline.contains(pad.shape):
        return "outside"
    rules = board.rules
    edge = rules.clearance if rules.edge_clearance is None else rules.edge_clearance
    if pad.shape.distance(board.outline.boundary) < edge + rules.base_width / 2.0:
        return "edge"
    if any(o.net_id < 0 and not o.copper and o.shape.intersects(pad.shape)
           and any(o.on(layer) for layer in (range(len(board.layers)) if pad.layers is None else pad.layers)) for o in board.obstacles):
        return "keepout"
    return "crowded"


def unreachable_note(buried: list[int], board) -> str:
    """What to tell the user about pads no trace can reach, by why."""
    pads = {p.pad_id: p for p in board.pads}
    groups: dict[str, list[str]] = {}
    for p in buried:
        groups.setdefault(why_buried(board, pads[p]), []).append(pads[p].name)
    parts = []
    for why in WHY_BURIED:
        names = sorted(groups.get(why, []))
        if names:
            shown = ", ".join(names[:12]) + (f" and {len(names) - 12} more" if len(names) > 12 else "")
            parts.append(f"{len(names)} {WHY_BURIED[why]}: {shown}")
    return (f"{len(buried)} pads cannot be reached by any trace under these rules, and their connections are left open. "
            + "; ".join(parts) + ".")


def narrowed_note(narrowed: dict[int, float], names: dict) -> str:
    """What to tell the user about nets routed narrower than their class to reach their pads."""
    shown = sorted(f"{names.get(n, n)} {w:g} mm" for n, w in narrowed.items())
    return (f"{len(shown)} nets routed narrower than their class, to reach fine-pitch pads: "
            + ", ".join(shown[:12]) + (f" and {len(shown) - 12} more" if len(shown) > 12 else ""))


def plane_left_note(left: list[int], board) -> str:
    """What to tell the user about pads of a plane net that the plane does not reach and no via could be placed for."""
    pads = {p.pad_id: p for p in board.pads}
    names = sorted(pads[p].name for p in left)
    return (f"{len(names)} pads of a plane net are not reached by its plane and have no room beside them for a via: "
            + ", ".join(names[:12]) + (f" and {len(names) - 12} more" if len(names) > 12 else "") + " (join them in KiCad)")


class Checkpoint:
    """Keeps the best routing so far in the output file while the router runs
    (``route_board``'s ``checkpoint``): written whenever a pass or a raced
    variant finishes with a better routing than the one written last, so a run
    that is stopped, crashes or is killed still leaves the best it had found.
    ``write_ses`` writes it whole or not at all."""

    def __init__(self, design, path: str, teardrops: bool, bar=None, arcs: bool = True):
        self.design, self.path, self.teardrops, self.bar, self.arcs = design, path, teardrops, bar, arcs
        self.best = None  # the key (``router._key``) of the routing written last

    def __call__(self, key: tuple, result) -> None:
        if self.best is not None and key >= self.best:
            return
        if self.bar is not None and self.bar.live:
            self.bar.stream.write("\r\x1b[K")  # (the bar's line: it is drawn again at the next event)
        try:
            write_ses(self.design, result, self.path, teardrops=self.teardrops, arcs=self.arcs)
        except OSError as error:  # (the routing goes on: the file is tried again with the next better one, and at the end)
            print(f"could not save the best routing so far to {self.path}: {error}", file=sys.stderr, flush=True)
            return
        self.best = key
        stats = result.stats
        print(f"saved the best routing so far: {stats['routed']}/{stats['connections']} connections routed, "
              f"{stats['vias']} vias -> {self.path}", flush=True)


def main(argv: list[str] | None = None) -> int:
    started = time.time()  # the time limit runs from here
    ap = argparse.ArgumentParser(prog="weaveengine", description="WeaveEngine: a topological PCB autorouter.")
    ap.add_argument("--version", action="version", version=f"WeaveEngine {__version__}")
    ap.add_argument("dsn", help="input Specctra DSN file")
    ap.add_argument("-o", "--output", help="output SES file (default: input name with .ses)")
    ap.add_argument("--layers", nargs="+", metavar="LAYER",
                    help="signal layers to route (default: all in the DSN); vias still keep clear of the others' copper and reach their planes")
    ap.add_argument("--svg", help="also write an SVG picture of the result")
    ap.add_argument("--no-vias", action="store_true", help="never use vias: every connection stays on one layer")
    ap.add_argument("--no-teardrops", action="store_true", help="no teardrops at all")
    ap.add_argument("--no-fanout", action="store_true",
                    help="no vias joining pads to their net's plane: those pads are left for the pour, or for you, to reach")
    ap.add_argument("--skeleton", action="store_true",
                    help="a skeleton in each pour, which traces go round, so that none cuts the pour into pieces "
                         "(design 11.3; some boards route fewer connections with it)")
    ap.add_argument("--no-narrowing", action="store_true",
                    help="never route a net narrower than its class, even where its pads cannot be reached otherwise")
    ap.add_argument("--no-ses-teardrops", action="store_true",
                    help="keep teardrops out of the SES (they are written as short widening traces; "
                         "leave them out to use the CAD tool's own teardrops instead)")
    ap.add_argument("--no-ses-arcs", action="store_true",
                    help="write curves in the SES as short straight segments, not arcs (qarc)")
    ap.add_argument("--edge-clearance", type=float, metavar="MM",
                    help="copper to board edge clearance (a DSN does not carry it; default: the trace clearance)")
    ap.add_argument("--kicad-pro", metavar="FILE",
                    help="KiCad project file to take the edge clearance from (default: one next to the DSN, if any)")
    ap.add_argument("--trace-width", type=float, metavar="MM", help="override the default class trace width from the DSN")
    ap.add_argument("--clearance", type=float, metavar="MM", help="override the clearance from the DSN")
    ap.add_argument("--via", nargs=2, type=float, metavar=("DIAMETER", "DRILL"), help="override the via size from the DSN (mm)")
    ap.add_argument("--layer-cost", action="append", metavar="LAYER:FACTOR",
                    help="traces on LAYER cost FACTOR times their length (1 or more), so it is used only where it must be: "
                         "a layer that also carries a plane, say (In1.Cu:5); once per layer")
    ap.add_argument("--ignore-keepouts", action="store_true", help="drop the keepout areas of the DSN")
    ap.add_argument("--margin", type=float, default=0.01, metavar="MM",
                    help="extra clearance to route with, so nothing sits exactly on a limit (default 0.01)")
    ap.add_argument("--workers", type=int, metavar="N", help="processes to use (default: all cores; 1 = no parallelism)")
    ap.add_argument("--portfolio", type=int, default=0, metavar="N",
                    help="routing variants raced per pass, best kept (default: one per worker, at most 8; 1 = off)")
    ap.add_argument("--max-rounds", type=int, metavar="N",
                    help="rip-up rounds at most, as the app's rip-up round limit (default 100; design-rule repair adds up to 5 "
                         "of its own each time it runs): fewer finish sooner, with more left open")
    ap.add_argument("--time-limit", type=float, metavar="MINUTES",
                    help="stop bettering the routing after this many minutes from the start, and write the best found so far "
                         "(its geometry and check take a little longer still; default: no limit)")
    ap.add_argument("--skip-net", action="append", metavar="NAME", default=[],
                    help="a net not to route, as one with a pour is (repeatable; * and ? match any text: GND*)")
    ap.add_argument("--quiet", action="store_true", help="no progress bar")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args(argv)
    if args.max_rounds is not None and args.max_rounds < 0:
        ap.error("--max-rounds: not below 0 (0 = no rip-up)")
    if args.time_limit is not None and not 0 < args.time_limit < math.inf:
        ap.error("--time-limit: a number of minutes above 0")
    deadline = started + 60.0 * args.time_limit if args.time_limit is not None else None
    safe_output()

    status = accel.check()
    print(status.warning if status.warning else status.message + (f" [{status.seconds:.0f} s to compile]" if status.seconds > 3 else ""),
          file=sys.stderr if status.warning else sys.stdout, flush=True)
    design = read_dsn(args.dsn, args.layers)
    rules = design.board.rules
    if args.ignore_keepouts:
        kept = [o for o in design.board.obstacles if o.net_id >= 0]
        print(f"ignoring {len(design.board.obstacles) - len(kept) + len(design.board.no_via)} keepout areas")
        design.board.obstacles, design.board.no_via = kept, []
    if args.trace_width:
        rules.trace_width = args.trace_width
    if args.clearance:
        rules.clearance = args.clearance
    if args.via:
        rules.via_diameter, rules.via_drill = args.via
    given = {"trace_width": args.trace_width, "clearance": args.clearance, "via_diameter": args.via, "via_drill": args.via}
    missing = [text for name, text in design.defaulted.items() if not given[name]]
    if missing:
        print("not in the DSN, defaults used: " + ", ".join(missing))
    if design.dropped_wiring:
        print(design.dropped_wiring)
    if design.kicad:
        print(design.kicad)
    for text in design.warnings:
        print(f"WARNING: {text}")
    if design.walled:
        print(walled_note(design.walled))
    if design.board.unrouted:
        print(f"not routed: {', '.join(design.board.unrouted)} (vias still keep clear of their copper, and reach their planes)")
    print(f"rules: trace {rules.trace_width:g} mm, clearance {rules.clearance:g} mm, via {rules.via_diameter:g}/{rules.via_drill:g} mm; "
          f"{len(design.board.layers)} layers, {len(design.board.pads)} pads")
    pro = args.kicad_pro or args.dsn.rsplit(".", 1)[0] + ".kicad_pro"
    if args.edge_clearance is not None:
        rules.edge_clearance = args.edge_clearance
    elif os.path.exists(pro):
        project = kicad_project_rules(pro)
        rules.edge_clearance = max(project["edge_clearance"], rules.clearance)
        print(f"from {os.path.basename(pro)}: edge clearance {rules.edge_clearance:g} mm"
              + (f" (hole clearance {project['hole_clearance']:g} mm is not kept: a DSN has no drill sizes, and there is no "
                 f"{os.path.basename(os.path.splitext(pro)[0])}.kicad_pcb beside it)" if project["hole_clearance"]
                 and not os.path.exists(os.path.splitext(pro)[0] + ".kicad_pcb") else ""))
    else:
        print(f"edge clearance {rules.clearance:g} mm (the DSN does not carry one; set it with --edge-clearance or --kicad-pro)")
    if design.board.plane_nets:
        names = design.net_names
        print(f"{len(design.board.plane_nets)} nets have a copper plane and are not routed as traces: "
              + ", ".join(sorted(names[n] for n in design.board.plane_nets)))
    note = skip_nets(design, args.skip_net)
    if note:
        print(note)
    layer_cost = {}
    for item in args.layer_cost or ():
        name, _, factor = item.rpartition(":")
        try:
            factor = float(factor)
        except ValueError:
            factor = 0.0
        if name not in design.board.layers or not 1.0 <= factor < math.inf:
            ap.error(f"--layer-cost {item}: a layer routed ({', '.join(design.board.layers)}) and a factor of 1 or more")
        if factor > 1.0:
            layer_cost[design.board.layers.index(name)] = factor
    if layer_cost:
        print("traces cost more on " + ", ".join(f"{design.board.layers[i]} ({f:g} times their length)" for i, f in sorted(layer_cost.items())))
    # Route slightly wider than the rules; report against the rules themselves.
    want_clearance = rules.clearance
    want_edge = rules.clearance if rules.edge_clearance is None else rules.edge_clearance
    rules.clearance = want_clearance + args.margin
    rules.edge_clearance = want_edge + args.margin
    rules.margin = args.margin
    options = Options(vias=not args.no_vias, teardrops=not args.no_teardrops, portfolio=args.portfolio, narrow=not args.no_narrowing,
                      fanout=not args.no_fanout, skeleton=args.skeleton)
    # (As the app makes them from its settings: scaled to the board, with the round limit and layer costs set.)
    overrides = {}
    if args.max_rounds is not None:
        overrides["max_rounds"] = args.max_rounds
    if layer_cost:
        overrides["layer_cost"] = layer_cost
    params = CostParams.for_map(planar_map.build(design.board), **overrides) if overrides else None
    if deadline is not None:
        print(f"time limit {args.time_limit:g} minutes: the routing is bettered until "
              f"{time.strftime('%H:%M:%S', time.localtime(deadline))}, then the best so far is finished and written")
    bar = None if args.quiet else ProgressBar(deadline=deadline)
    out = args.output or (args.dsn.rsplit(".", 1)[0] + ".ses")
    saver = Checkpoint(design, out, not args.no_ses_teardrops, bar, not args.no_ses_arcs)
    try:
        result = route_board(design.board, params, options=options, seed=args.seed,
                             workers=args.workers, events=bar.event if bar else None, deadline=deadline, checkpoint=saver)
    except KeyboardInterrupt:
        if bar:
            bar.finish()
        print("stopped" + (f": the best routing found is in {out}" if saver.best is not None else f", before any routing was finished to save in {out}"),
              file=sys.stderr)
        return 130
    if bar:
        bar.finish()
    stats = result.stats
    write_ses(design, result, out, teardrops=not args.no_ses_teardrops, arcs=not args.no_ses_arcs)
    per_layer = ", ".join(f"{name}: {count}" for name, count in stats["wires_per_layer"].items())
    print(f"{stats['routed']}/{stats['connections']} connections routed ({per_layer}; {stats['vias']} vias), "
          f"length ratio {stats['length_ratio']:.3f}, {stats['time_total']:.1f}s -> {out}"
          + (f" (the time limit was reached {time.time() - deadline:.0f} s before this)" if deadline is not None and time.time() > deadline else ""))
    names = design.net_names
    pads = {p.pad_id: p for p in result.board.pads}
    for wire in result.unrouted:
        conn = result.connections[wire]
        print(f"  unrouted: net {names.get(conn.net_id, conn.net_id)}: {pads[conn.src].name} -> {pads[conn.dst].name}")
    if stats["skeletons"] > stats["skeletons_routed"]:
        print(f"  pours: {stats['skeletons'] - stats['skeletons_routed']} of {stats['skeletons']} joins of pads through their pour "
              "gave way to traces")
    if stats["fanout"]:
        print(f"  {stats['fanout']} vias placed to join pads to their net's plane")
    if stats["plane_left"]:
        print(f"  {plane_left_note(stats['plane_left'], result.board)}")
    if stats["narrowed"]:
        print(f"  {narrowed_note(stats['narrowed'], names)}")
    if stats["buried_pads"]:
        print(f"  {unreachable_note(stats['buried_pads'], result.board)}")
    for pour in estimate_pours(result):
        print(f"  pours: {pour_note(pour, result.board)}")
    m = measure(design, out, want_clearance, want_edge)
    print(f"measured from {os.path.basename(out)}: smallest gap trace-pad {m.track_to_pad:.4f}, trace-trace {m.track_to_track:.4f} "
          f"(rule {m.required_clearance:g}), trace-edge {m.track_to_edge:.4f} (rule {m.required_edge:g})"
          f"{'' if m.track_to_keepout == float('inf') else f', trace-keepout {m.track_to_keepout:.4f} (not inside)'} mm; "
          f"{m.crossings} crossings; {m.vias_in_keepout} vias in keepouts; widths {'as ruled' if not m.wrong_width else 'WRONG for ' + ', '.join(m.wrong_width)}"
          f" -> {'OK' if m.ok else 'VIOLATIONS'}")
    if args.svg:
        export_result(result, args.svg)
    return 0 if not result.unrouted and not stats["buried_pads"] else 1


if __name__ == "__main__":
    sys.exit(main())
