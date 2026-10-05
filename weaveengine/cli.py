"""Command line: read a Specctra DSN, route it, write a SES (and optionally an SVG)."""
import argparse
import os
import sys

from weaveengine import accel
from weaveengine.io.check import kicad_project_rules, measure
from weaveengine.io.dsn import read_dsn
from weaveengine.io.ses import write_ses
from weaveengine.plan.context import Options
from weaveengine.progress import ProgressBar
from weaveengine.router import route_board
from weaveengine.viz.svg import export_result


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="weaveengine", description="WeaveEngine: a topological PCB autorouter.")
    ap.add_argument("dsn", help="input Specctra DSN file")
    ap.add_argument("-o", "--output", help="output SES file (default: input name with .ses)")
    ap.add_argument("--layers", nargs="+", metavar="LAYER", help="signal layers to route (default: all in the DSN)")
    ap.add_argument("--svg", help="also write an SVG picture of the result")
    ap.add_argument("--no-vias", action="store_true", help="never insert vias")
    ap.add_argument("--max-via-rounds", type=int, default=4, help="how many times to add vias and route again")
    ap.add_argument("--no-teardrops", action="store_true", help="no teardrops at all")
    ap.add_argument("--no-ses-teardrops", action="store_true",
                    help="keep teardrops out of the SES (they are written as short widening traces; "
                         "leave them out to use the CAD tool's own teardrops instead)")
    ap.add_argument("--edge-clearance", type=float, metavar="MM",
                    help="copper to board edge clearance (a DSN does not carry it; default: the trace clearance)")
    ap.add_argument("--kicad-pro", metavar="FILE",
                    help="KiCad project file to take the edge clearance from (default: one next to the DSN, if any)")
    ap.add_argument("--trace-width", type=float, metavar="MM", help="override the default class trace width from the DSN")
    ap.add_argument("--clearance", type=float, metavar="MM", help="override the clearance from the DSN")
    ap.add_argument("--via", nargs=2, type=float, metavar=("DIAMETER", "DRILL"), help="override the via size from the DSN (mm)")
    ap.add_argument("--ignore-keepouts", action="store_true", help="drop the keepout areas of the DSN")
    ap.add_argument("--margin", type=float, default=0.01, metavar="MM",
                    help="extra clearance to route with, so nothing sits exactly on a limit (default 0.01)")
    ap.add_argument("--workers", type=int, metavar="N", help="processes to use (default: all cores; 1 = no parallelism)")
    ap.add_argument("--portfolio", type=int, default=0, metavar="N",
                    help="routing variants raced per pass, best kept (default: one per worker, at most 8; 1 = off)")
    ap.add_argument("--quiet", action="store_true", help="no progress bar")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args(argv)

    status = accel.check()
    print(status.warning if status.warning else status.message + (f" [{status.seconds:.0f} s to compile]" if status.seconds > 3 else ""),
          file=sys.stderr if status.warning else sys.stdout, flush=True)
    design = read_dsn(args.dsn, args.layers)
    rules = design.board.rules
    if args.ignore_keepouts:
        kept = [o for o in design.board.obstacles if o.net_id >= 0]
        print(f"ignoring {len(design.board.obstacles) - len(kept)} keepout areas")
        design.board.obstacles = kept
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
    print(f"rules: trace {rules.trace_width:g} mm, clearance {rules.clearance:g} mm, via {rules.via_diameter:g}/{rules.via_drill:g} mm; "
          f"{len(design.board.layers)} layers, {len(design.board.pads)} pads")
    pro = args.kicad_pro or args.dsn.rsplit(".", 1)[0] + ".kicad_pro"
    if args.edge_clearance is not None:
        rules.edge_clearance = args.edge_clearance
    elif os.path.exists(pro):
        project = kicad_project_rules(pro)
        rules.edge_clearance = max(project["edge_clearance"], rules.clearance)
        print(f"from {os.path.basename(pro)}: edge clearance {rules.edge_clearance:g} mm"
              + (f" (hole clearance {project['hole_clearance']:g} mm is not checked: a DSN has no drill sizes)" if project["hole_clearance"] else ""))
    else:
        print(f"edge clearance {rules.clearance:g} mm (the DSN does not carry one; set it with --edge-clearance or --kicad-pro)")
    if design.board.plane_nets:
        names = design.net_names
        print(f"{len(design.board.plane_nets)} nets have a copper plane and are not routed as traces: "
              + ", ".join(sorted(names[n] for n in design.board.plane_nets)))
    # Route slightly wider than the rules; report against the rules themselves.
    want_clearance = rules.clearance
    want_edge = rules.clearance if rules.edge_clearance is None else rules.edge_clearance
    rules.clearance = want_clearance + args.margin
    rules.edge_clearance = want_edge + args.margin
    options = Options(vias=not args.no_vias, teardrops=not args.no_teardrops, portfolio=args.portfolio)
    bar = None if args.quiet else ProgressBar()
    result = route_board(design.board, options=options, seed=args.seed, max_via_rounds=args.max_via_rounds,
                         workers=args.workers, progress=bar)
    if bar:
        bar.finish()
    stats = result.stats
    out = args.output or (args.dsn.rsplit(".", 1)[0] + ".ses")
    write_ses(design, result, out, teardrops=not args.no_ses_teardrops)
    per_layer = ", ".join(f"{name}: {count}" for name, count in stats["wires_per_layer"].items())
    print(f"{stats['routed']}/{stats['connections']} connections routed ({per_layer}; {stats['vias']} vias), "
          f"length ratio {stats['length_ratio']:.3f}, {stats['time_total']:.1f}s -> {out}")
    names = design.net_names
    pads = {p.pad_id: p for p in result.board.pads}
    for wire in result.unrouted:
        conn = result.connections[wire]
        print(f"  unrouted: net {names.get(conn.net_id, conn.net_id)}: {pads[conn.src].name} -> {pads[conn.dst].name}")
    if stats["buried_pads"]:
        print(f"  {len(stats['buried_pads'])} pads have no reachable boundary (overlapping clearances)")
    m = measure(design, out, want_clearance, want_edge)
    print(f"measured from {os.path.basename(out)}: smallest gap trace-pad {m.track_to_pad:.4f}, trace-trace {m.track_to_track:.4f} "
          f"(rule {m.required_clearance:g}), trace-edge {m.track_to_edge:.4f} (rule {m.required_edge:g}) mm; "
          f"{m.crossings} crossings; widths {'as ruled' if not m.wrong_width else 'WRONG for ' + ', '.join(m.wrong_width)}"
          f" -> {'OK' if m.ok else 'VIOLATIONS'}")
    if args.svg:
        export_result(result, args.svg)
    return 0 if not result.unrouted else 1


if __name__ == "__main__":
    sys.exit(main())
