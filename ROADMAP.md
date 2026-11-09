# Roadmap

What is planned after 1.0. Bugs are fixed as they are found; this is for
features.

## 1.1

### 1.0.2 (released 2026-11-08): fixes users found

From the first users' reports (r/PCB, 2026-11): rule areas that forbid only
tracks or only vias are read (#1); fan-out vias share locked vias and keep
their drills apart (#3); `--max-rounds` (#2); `--layers` keeps the other
layers' pours and copper (#4); `--layer-cost` (#5); `--time-limit`, with the
best routing written as it goes; locked traces count as reaching their pad;
blind and buried locked vias are kept on their own layers; vias checked
against keepouts and no-via areas, which are drawn in the app; the sweep
routes the same DSN every run (lily58-pro's two results were KiCad exporting
its outline two ways).

Left over from it:
- [ ] **A time estimate that is not hours out** without a time limit, on a
  board whose rip-up goes on improving for hours (design 24, Open).
- [x] **The board sweep on GitHub** keeps each board's prepared DSN in the
  Actions cache, so its runs route the same DSNs: two runs of tiers 1-3 came
  out identical (Linux baseline: `sweep-cache-a-d4de30f` on sweep-results).
  Results differ slightly between macOS and Linux, from floating point:
  compare runs only on one platform.
- [ ] Whether a DSN's power layers should be left out of the routing by
  default, and plane layers made dearer, measured with the pour counts.
- Tried and kept as is: where vias may stand on every board. #5's filter
  (`Reach.placeable`) stays on only with a layer cost: applied always it lost
  a connection on kicad-openair and solo-a, and letting via seeds not record
  a triangle's best cost lost one on hackrf-operacake and solo-a instead.

### 1.1 checklist

Each item on its own branch, merged only when the board sweep (`bench/sweep`,
quick boards first) shows nothing worse than before. Done items are on main,
not yet released.

Done:
- [x] Arcs in the session (`qarc`, at most 90 degrees each; `--no-ses-arcs`).
- [x] Nets left unrouted by choice (`--skip-net`, a setting).
- [x] The pour fill estimate, shown in the app (a key, cut-off pieces and the
  pads they lose in magenta, islands on request, a Pours switch).
- [x] Pours that stay whole: built, as an option off by default ("Keep pours
  whole", `--skeleton`); the zones' own minimum width, with an override.
- [x] Fan-out vias only where the plane's fill really is (no via to nowhere).
- [x] Teardrop notches.
- [x] KiCad custom rules (`.kicad_dru`): the pad and net subset
  (`io/kicad_rules.py`).
- [x] A Warnings section in the app, with a warning for Margin-layer drawings.
- [x] Settings: a Pours tab and a search box.

To do, in order:
1. [ ] Traces pulled fully taut. Likely cause: via sites left unused stay in
   the map as holes that traces keep passing round (design 13.1). Check it on a
   board with many vias, or the reporter's.
2. [ ] Pours that stay whole by default: skeletons that cost traces less,
   room for thermal spokes, each zone's clearance and priority (below).
3. [ ] Room used where there is room.
4. [x] Traces tapered to fine-pitch pins (on branch `taper`, `realize/taper.py`).
5. [ ] The Margin layer as board edge: a directional rule (below).
6. [ ] No acute angles where copper of one net meets.
7. [ ] The 45-degree option (a second way of drawing, off by default).
8. [ ] A via size per net class.

### Trace shape

- **Traces pulled fully taut.** Users saw thin traces on a complex board that
  were not pulled tight, and traces that cross a gap at an angle where they
  could run along the clearance edge. Find the case on the reporter's board
  and fix the relaxation (design 13.1), with a test; check the sweep's
  boards for slack left in drawn traces.
- **Room used where there is room.** Traces are drawn at exactly the
  clearance even where the board has space, so they bunch in narrow places
  and leave open areas empty. Where a gap holds fewer traces than it could,
  spread them across it: easier to make, less crosstalk, and easier to read.
- **Arcs in the session.** Write rounded corners and the curves of taut traces
  as arcs (`qarc`: start, end and centre) instead of many short straight
  segments. KiCad's session import turns a `qarc` into a true track arc of
  any angle up to 180 degrees (checked with KiCad 10: arcs of 7.5, 37, 90,
  135 and 180 degrees came in as written). Beyond that it takes the shorter
  way round between start and end (250 degrees came in as 110), so a longer
  arc is written in pieces; pieces of at most 90 degrees, since the Specctra
  format means a quarter arc and other tools may hold to it. Far fewer tracks
  per connection, and ones KiCad's drag tools handle as a curve.
- **A 45-degree output option.** Traces drawn as straight runs at multiples of
  45 degrees, as many boards are expected to look, instead of taut curves:
  the same topology, realised on octilinear paths, with the clearances checked
  again.
- **No acute angles where copper of one net meets.** Sharp junctions and
  acid traps are one fault: a trace meeting its pad, a trace of its net, or
  two more at a junction, via or fan-out stub, at an angle that traps etchant
  and looks wrong. One setting gives the least angle. A trace into its pad or
  onto a trunk bends away over its last stretch; a junction of three or more
  (about one via in five now has traces meeting under 90 degrees) moves with
  all its connections, which re-planning one connection cannot do (design
  13.3a). Design 13.3b.
- **Traces tapered to fine-pitch pins.** Where a trace is wider than the pin
  it enters, narrow it over its last stretch to the pin's width instead of
  routing the whole net narrower (`router.reach`) or running a wide trace
  into a narrow pad.
  Done as post-processing: `router.reach` still plans a net narrower where its
  pins need it; then each such trace is redrawn at the net's width except a
  narrow stretch (the pad's reach plus 1, 2, 4 or 8 trace widths) at the
  fine-pitch pin, where the wide part clears pads, keepouts, the edge and the
  other traces as drawn; else it stays narrow all along. It is several wires
  in the SES (`Result.widths`). Left: the planner does not reserve room for the
  wide part (it fails the check where traces were planned too close), and the
  app draws such traces at the narrow width.

### Routing control

- **Nets left unrouted by choice.** A setting naming nets not to route (GND,
  say, before its pour is drawn), as a net with a pour already is.

### Pours and planes

- **Pours that stay whole.** A ground or power pour is left to KiCad to fill
  around the traces, and the routing often cuts it into islands: pieces of
  copper joined to nothing, or sections cut off from the rest of the net.
  The plan: route each pour as one more wire, a skeleton on its layer as wide
  as the pour's minimum width, joining its pads and taking its room in the
  gates like any other wire. Wires never cross, so no trace can cut it, and
  KiCad's fill covers it. Where a dense board has no room for it, its pad is
  joined by a via to the net's plane on another layer, as now, and named.
  Design 11.3. Merged as an option ("Keep pours whole", `--skeleton`, off
  by default): cut-off pads 379 to 57 on tier 2 and 412 open items instead
  of 512 in KiCad, but 29 connections fewer routed on three dense boards and
  twice the time. Still to do, to make it the default:
  - skeletons that cost traces less: thinner where room is short, or priced
    below a trace in negotiation;
  - room for KiCad's thermal-relief spokes where a skeleton meets a pad;
  - each zone's own clearance (the board's is used now) and zone priorities.
- **Pours shown while routing.** Done after routing: the estimated fill on
  the board view, a key, cut-off pieces and the pads they lose marked in
  magenta, islands on request. Still to do:
  - the estimate during routing, not only on the result;
  - thermal-relief spokes in the estimate: it misses pads KiCad leaves
    unconnected because their spokes are starved (lumen-ringlight,
    hackrf-lna915).

### Rules

- **A via size per net class.** Route each net with its class's via (the
  DSN's `use_via`), not the board's default: room for a larger via has to be
  planned for in the triangulation, not added once the routing is done.
- **KiCad custom rules (`.kicad_dru`).** Read the rules that let fine-pitch
  parts be routed at less than the board's clearance.
- **The Margin layer as board edge**, without burying pads that sit inside
  it. Tried (2026-11, branch `margin-edge`, not merged): Margin drawings as
  obstacles with room carved round each pad. On the boards with these errors
  the drawings run within the edge clearance of their own pads, so the
  carve-outs are where the errors are: a generous one removed none (solo-a,
  solo-c and hackrf-lna915 unchanged) and still cost Solo about ten
  connections; a tight one removed some but lost connections. Needs a
  directional rule: a trace leaves such a pad only away from the drawing.

## 1.2

### Editing the routing before it is exported

A way to correct the routing in WeaveEngine itself before writing the SES:
move a via, reroute or delete a connection, pin a trace where you want it,
then let the router fill in around what you fixed.

Why here and not in KiCad: WeaveEngine models a connection as a topology (which
side of each pad and trace it passes) with taut geometry, rounded corners and
teardrops drawn from it. In KiCad it becomes plain tracks and arcs, and its
teardrops are fans of tracks, so one change there means fixing each piece by
hand. Editing before export keeps the model, and draws the geometry again
after each change, so an edit costs a drag, not a cleanup.

Wanted:
- select a trace, a via or a whole connection on the board view;
- move a via, with the traces through it redrawn taut as it moves;
- delete a connection, or rip it up and route it again;
- mark copper as fixed, so later routing works around it;
- undo and redo;
- the same rule checks as a full routing, live, before export.

## Later

- **A KiCad plugin.** Route from inside KiCad's PCB editor: one button that
  exports the board, routes it with WeaveEngine (showing its progress) and
  imports the session, with no DSN or SES files to handle by hand. Through
  KiCad's IPC API (KiCad 9 and later), which can also read the board's rules
  and zones directly instead of from the files beside a DSN.
