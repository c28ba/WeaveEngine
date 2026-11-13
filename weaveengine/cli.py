"""Command line: read a Specctra DSN, route it, write a SES (and optionally an SVG)."""
import argparse
import math
import os
import sys
import time

from weaveengine import __version__, accel, safe_output
from weaveengine.io.check import kicad_project_rules, measure
from weaveengine.io.dsn import read_dsn, walled_note
from weaveengine.io.kicad_board import TEARDROP_STYLES, routed_board_path, source_board, write_routed_board
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


# Each routing switch of ``Options`` and the flag that turns it off, and the numbers of ``Options`` and ``CostParams``
# the settings carry and their flags (a test holds these to the dataclasses).
OPTION_SWITCHES = {
    "global_selection": ("--no-global-selection", "no global choice of routes for all nets together (phase 1)"),
    "regret_order": ("--no-regret-order", "commit routes shortest first, not most constrained first (phase 2)"),
    "lookahead": ("--no-lookahead", "no airwire-crossing and closure costs in the search"),
    "demand": ("--no-demand", "no demand map reserving scarce channels for the nets that need them"),
    "ripup": ("--no-ripup", "no rip-up and reroute (phase 3)"),
    "refine": ("--no-refine", "do not reroute each wire once more to shorten it (phase 4)"),
    "slide": ("--no-slide-vias", "do not move vias to where their traces run straighter"),
    "smooth": ("--no-smooth", "do not round sharp corners"),
    "untangle": ("--no-untangle", "do not undo long detours by moving the wires in their way"),
    "taper": ("--no-taper", "draw a net narrowed to reach fine-pitch pins narrow all along, not tapered to the pins "
                            "(a wedge from the full width over the last stretch)"),
}
OPTION_FLAGS = {"vias": "--no-vias", "narrow": "--no-narrowing", "fanout": "--no-fanout", "teardrops": "--no-teardrops",
                "skeleton": "--no-skeleton", "portfolio": "--portfolio", "teardrop_max_length": "--teardrop-max-length",
                "teardrop_max_width": "--teardrop-max-width", "teardrop_breathing": "--teardrop-breathing", "least_angle": "--least-angle",
                **{name: flag for name, (flag, _) in OPTION_SWITCHES.items()}}
COST_FLAGS = {"K": "--candidates", "K_reroute": "--reroute-candidates", "max_rounds": "--max-rounds", "h_weight": "--heuristic-weight",
              "layer_cost": "--layer-cost"}


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
    ap.add_argument("--teardrops", choices=list(TEARDROP_STYLES), default="tracks",
                    help="teardrops where traces meet pads and vias: " + "; ".join(f"{k}: {v}" for k, v in TEARDROP_STYLES.items())
                         + " (default %(default)s)")
    ap.add_argument("--no-teardrops", action="store_true", help="no teardrops at all (the same as --teardrops none)")
    for name, (flag, text) in OPTION_SWITCHES.items():
        ap.add_argument(flag, dest="no_" + name, action="store_true", help=text)
    ap.add_argument("--teardrop-max-length", type=float, default=Options.teardrop_max_length, metavar="MM",
                    help="longest teardrop beyond the pad's edge (default %(default)g)")
    ap.add_argument("--teardrop-max-width", type=float, default=Options.teardrop_max_width, metavar="MM",
                    help="widest teardrop, across at the pad (default %(default)g)")
    ap.add_argument("--teardrop-breathing", type=float, default=Options.teardrop_breathing, metavar="N",
                    help="clearances a teardrop keeps from foreign copper, else it is left out (default %(default)g)")
    ap.add_argument("--least-angle", type=float, default=Options.least_angle, metavar="DEG",
                    help="the least angle at which copper of one net may meet itself (a trace and its pad, traces at a via or "
                         "pad, a branch off a trunk): sharper meetings, which trap etchant, are redrawn where there is room; "
                         "0: not (default %(default)g)")
    ap.add_argument("--no-fanout", action="store_true",
                    help="no vias joining pads to their net's plane: those pads are left for the pour, or for you, to reach")
    ap.add_argument("--no-skeleton", action="store_true",
                    help="no skeleton in each pour: traces may cut a pour into pieces (by default each pour gets one once the "
                         "traces are routed, which traces move off where they can; design 11.3)")
    ap.add_argument("--skeleton", action="store_true", help=argparse.SUPPRESS)  # (the default now; kept for scripts that ask for it)
    ap.add_argument("--pour-min-width", type=float, default=0.0, metavar="MM",
                    help="the narrowest copper a pour's fill may have, in the estimate of the fill and the skeletons "
                         "(default: each zone's min_thickness from the .kicad_pcb, else 0.25)")
    ap.add_argument("--no-narrowing", action="store_true",
                    help="never route a net narrower than its class, even where its pads cannot be reached otherwise")
    ap.add_argument("--no-ses-teardrops", action="store_true",
                    help="no teardrops in the SES (the same as --teardrops none: KiCad's own Edit > Edit Teardrops can add them)")
    ap.add_argument("--routed-board", metavar="FILE",
                    help="also write the routing straight into a copy of the .kicad_pcb beside the DSN, as FILE (with "
                         "--teardrops zones this is where the teardrops go; default then: NAME-routed.kicad_pcb beside the output). "
                         "The original board is never changed")
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
    ap.add_argument("--candidates", type=int, metavar="K", help="alternative routes considered per connection in phase 1 (default 6)")
    ap.add_argument("--reroute-candidates", type=int, metavar="K", help="alternative routes considered per reroute in rip-up (default 2)")
    ap.add_argument("--heuristic-weight", type=float, metavar="W",
                    help="search heuristic weight: 1 is exact, above 1 is faster and less exact (default 1)")
    ap.add_argument("--drc-rounds", type=int, default=4, metavar="N",
                    help="reroute attempts for traces that fail the final check (default 4)")
    ap.add_argument("--time-limit", type=float, metavar="MINUTES",
                    help="stop bettering the routing after this many minutes from the start, and write the best found so far "
                         "(its geometry and check take a little longer still; default: no limit)")
    ap.add_argument("--skip-net", action="append", metavar="NAME", default=[],
                    help="a net not to route, as one with a pour is (repeatable; * and ? match any text: GND*)")
    ap.add_argument("--quiet", action="store_true", help="no progress bar")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args(argv)
    for flag, value in (("--candidates", args.candidates), ("--reroute-candidates", args.reroute_candidates)):
        if value is not None and value < 1:
            ap.error(f"{flag}: at least 1")
    if args.heuristic_weight is not None and args.heuristic_weight < 1.0:
        ap.error("--heuristic-weight: 1 or more")
    if not 0.0 <= args.least_angle <= 90.0:
        ap.error("--least-angle: 0 to 90 degrees (a branch square to its trunk meets it at 90)")
    if args.drc_rounds < 0:
        ap.error("--drc-rounds: not below 0")
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
        rules.edge_clearance = max(project["edge_clearance"], rules.clearance, rules.edge_clearance or 0.0)
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
    if args.pour_min_width > 0:
        design.board.pour_override = args.pour_min_width
        print(f"pour minimum width {args.pour_min_width:g} mm, set with --pour-min-width (the zones' own are not used)")
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
    style = "none" if args.no_teardrops or args.no_ses_teardrops else args.teardrops
    options = Options(vias=not args.no_vias, teardrops=style != "none", portfolio=args.portfolio, narrow=not args.no_narrowing,
                      fanout=not args.no_fanout, skeleton=not args.no_skeleton, teardrop_max_length=args.teardrop_max_length,
                      teardrop_max_width=args.teardrop_max_width, teardrop_breathing=args.teardrop_breathing, least_angle=args.least_angle,
                      **{name: not getattr(args, "no_" + name) for name in OPTION_SWITCHES})
    # (As the app makes them from its settings: scaled to the board, with the round limit and layer costs set.)
    overrides = {}
    if args.max_rounds is not None:
        overrides["max_rounds"] = args.max_rounds
    for name, value in (("K", args.candidates), ("K_reroute", args.reroute_candidates), ("h_weight", args.heuristic_weight)):
        if value is not None:
            overrides[name] = value
    if layer_cost:
        overrides["layer_cost"] = layer_cost
    params = CostParams.for_map(planar_map.build(design.board), **overrides) if overrides else None
    if deadline is not None:
        print(f"time limit {args.time_limit:g} minutes: the routing is bettered until "
              f"{time.strftime('%H:%M:%S', time.localtime(deadline))}, then the best so far is finished and written")
    bar = None if args.quiet else ProgressBar(deadline=deadline)
    out = args.output or (args.dsn.rsplit(".", 1)[0] + ".ses")
    saver = Checkpoint(design, out, style == "tracks", bar, not args.no_ses_arcs)
    try:
        result = route_board(design.board, params, options=options, seed=args.seed, drc_rounds=args.drc_rounds,
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
    write_ses(design, result, out, teardrops=style == "tracks", arcs=not args.no_ses_arcs)
    board_out = args.routed_board or (routed_board_path(out) if style == "zones" else None)
    if board_out:
        source = source_board(args.dsn)
        if source is None:
            print(f"WARNING: no routed copy of the board: there is no {os.path.splitext(os.path.basename(args.dsn))[0]}.kicad_pcb "
                  "beside the DSN" + (", so the teardrops (zones) are not written anywhere" if style == "zones" else ""))
        else:
            try:
                counts = write_routed_board(design, result, source, board_out, style, not args.no_ses_arcs)
                print(f"routed copy of the board: {counts['tracks']} tracks, {counts['arcs']} arcs, {counts['vias']} vias, "
                      f"{counts['zones']} teardrop zones -> {board_out} (fill its zones in KiCad, B, before checking it)")
            except (OSError, ValueError) as error:
                print(f"WARNING: could not write the routed copy of the board {board_out}: {error}")
    if style == "none" and not args.quiet:
        print("no teardrops written: KiCad's own Edit > Edit Teardrops can add them")
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
        print(f"  of their traces: {stats['tapered']} tapered, {stats['narrow_all_along']} narrow all along")
    if "acute" in stats:
        print(f"  acute meetings of one net's copper: {sum(stats['acute'].values())} ("
              + ", ".join(f"{kind} {n}" for kind, n in stats["acute"].items()) + f"); {stats['blunted']} traces redrawn for them")
    if stats["buried_pads"]:
        print(f"  {unreachable_note(stats['buried_pads'], result.board)}")
    found = estimate_pours(result)
    for pour in found:
        print(f"  pours: {pour_note(pour, result.board)}")
    if found:  # (what KiCad's check should find open: connections not routed, pads no pour or via joins)
        lost = {p for pour in found for piece in pour.pieces if piece.kind == "cut" for p in piece.pad_ids} | set(stats["plane_left"] or ())
        # (a pad drawn in several pieces, its name then ``@1``, ``@2``..., is one pad to KiCad)
        left = {pads[i].name.split("@")[0] if pads[i].name else i for i in lost}
        print(f"  estimate: {stats['connections'] - stats['routed'] + len(left)} items left unconnected in KiCad")
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
