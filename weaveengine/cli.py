"""Command line: read a Specctra DSN, route it, write a SES (and optionally an SVG)."""
import argparse
import os
import sys

from weaveengine.io.check import kicad_project_rules, measure
from weaveengine.io.dsn import read_dsn
from weaveengine.io.ses import write_ses
from weaveengine.plan.context import Options
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
    ap.add_argument("--no-teardrops", action="store_true", help="no teardrops in the SVG")
    ap.add_argument("--ses-teardrops", action="store_true", help="write teardrop outlines into the SES as extra traces")
    ap.add_argument("--edge-clearance", type=float, metavar="MM",
                    help="copper to board edge clearance (a DSN does not carry it; default: the trace clearance)")
    ap.add_argument("--kicad-pro", metavar="FILE",
                    help="KiCad project file to take the edge clearance from (default: one next to the DSN, if any)")
    ap.add_argument("--margin", type=float, default=0.01, metavar="MM",
                    help="extra clearance to route with, so nothing sits exactly on a limit (default 0.01)")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args(argv)

    design = read_dsn(args.dsn, args.layers)
    rules = design.board.rules
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
    # Route slightly wider than the rules; report against the rules themselves.
    want_clearance = rules.clearance
    want_edge = rules.clearance if rules.edge_clearance is None else rules.edge_clearance
    rules.clearance = want_clearance + args.margin
    rules.edge_clearance = want_edge + args.margin
    options = Options(vias=not args.no_vias, teardrops=not args.no_teardrops)
    result = route_board(design.board, options=options, seed=args.seed, max_via_rounds=args.max_via_rounds)
    stats = result.stats
    out = args.output or (args.dsn.rsplit(".", 1)[0] + ".ses")
    write_ses(design, result, out, teardrops=args.ses_teardrops)
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
