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
- [x] Pours that stay whole by default (branch `pours-default`): skeletons put
  in after the traces are negotiated, traces moved off them only where every
  one goes back; each zone's clearance, thermal reliefs and priority read and
  used; `--no-skeleton` to leave them out. Design 11.3.
- [x] Fan-out vias only where the plane's fill really is (no via to nowhere).
- [x] Teardrop notches.
- [x] KiCad custom rules (`.kicad_dru`): the pad and net subset
  (`io/kicad_rules.py`).
- [x] A Warnings section in the app, with a warning for Margin-layer drawings.
- [x] Settings: a Pours tab and a search box.

Fix first (bugs in features just added, seen on a user's board, 2026-11-10;
probably partly one cause; before any new feature):
- [ ] Skeleton and KiCad's fill disagree where two pads' copper overlaps.
- [ ] KiCad's DRC finds many more unconnected items than our estimate; all
  the ones we do not account for are on the GND or 5V pours.
- [ ] KiCad's DRC finds a great many disconnected pour pieces (islands).
- [ ] The pour clearance and tolerance we use differ greatly from KiCad's.

Details under Pours and planes, "Pour estimate against KiCad".

To do, in order:
1. [ ] Traces pulled fully taut. Likely cause: via sites left unused stay in
   the map as holes that traces keep passing round (design 13.1). Check it on a
   board with many vias, or the reporter's.
2. [x] Pours that stay whole by default (done above; what is left is below).
3. [ ] Room used where there is room.
4. [x] Traces tapered to fine-pitch pins (on branch `taper`, `realize/taper.py`).
5. [ ] The Margin layer as board edge: a directional rule (below).
6. [ ] No acute angles where copper of one net meets.
7. [ ] The 45-degree option (a second way of drawing, off by default).
8. [ ] A via size per net class.
9. [ ] Most curves still exported as segments, not arcs (below).
10. [ ] Curves that are not arcs: check how many, and whether a Bézier or
   spline output would help (below).
11. [ ] Teardrops as polygons instead of fans of tracks, if supported (below).
12. [ ] Pour pieces and pins joined by stitching vias or traces, chosen by cost
   (below).

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
  Left (user report, 2026-11-10): it works, but most of what looks like an
  arc in the app still comes out as short segments. A guess, not checked: a
  round pad is held as a polygon, so a trace wrapped round it follows that
  polygon's corners and the arc fit does not see one circle. Count, on a few
  boards, the curved runs written as `qarc` and those that are not, and see
  why the fit misses them (pad polygons, tolerance, or runs too short).
- **Curves that are not arcs.** Some curves may not be circular at all (a
  taut trace bending between two obstacles of different size, a smoothed
  corner), so no arc fits them. Measure how much of the drawn curve that is;
  if it is a lot, look at whether Specctra sessions or KiCad take a Bézier or
  spline track (KiCad's tracks are only lines and arcs, as far as known; check
  KiCad 10), or else fit those runs with a few arcs each.
- **Teardrops as polygons.** Teardrops and taper wedges are written as fans
  of tracks. Check whether the session format and KiCad's import take a
  filled polygon for them (a Specctra `wire` with a polygon shape, or KiCad's
  own teardrop zones), which would be one clean shape to edit instead of many
  tracks.
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
  fine-pitch pin and an even wedge from there up to the full width (3 times
  the difference, written as a fan of tracks like a teardrop; a step where no
  straight run has room for it), where the wide part clears pads, keepouts, the edge and the
  other traces as drawn; else it stays narrow all along. It is several wires
  in the SES (`Result.widths`). Left: the planner does not reserve room for the
  wide part (it fails the check where traces were planned too close), and the
  app draws such traces at the narrow width.

### Routing control

- **Nets left unrouted by choice.** A setting naming nets not to route (GND,
  say, before its pour is drawn), as a net with a pour already is.

### Pours and planes

- **Pours that stay whole.** A ground or power pour is left to KiCad to fill
  around the traces, and the routing often cuts it into islands. Each pour is
  now given a skeleton, a wire on its layer joining its pads that no trace
  crosses (design 11.3), on by default ("Keep pours whole", `--no-skeleton`).
  The skeletons go in once the traces are negotiated, and traces in a
  skeleton's way are moved only where every one of them goes back, so no
  connection is lost to them; a pad its skeleton cannot reach, or whose
  thermal spokes the estimate finds starved, gets a via down to the net's
  plane (or a short trace into the fill), and is named. Measured on the Mac against main:
  tier 1 routes the same, KiCad open items 134 to 104, cut-off pads 80 to
  51; tier 2 one more routed, open items 515 to 364, cut-off pads 357 to
  125; time +24 % on tier 1 and +11 % on tier 2 (cm5-minima twice main's).
  Still to do:
  - room for the thermal spokes where a skeleton meets a pad: the estimate
    now sees starved spokes and fan-out joins such pads where there is room,
    but the skeleton does not yet arrive along a spoke (pic-programmer's DIP
    pins on a one-layer pour);
  - more skeletons in where traces could move with a via more, or several
    moved at once (about one join in four still gives way on the dense
    boards; fomu and sweep-half-swept keep more pads cut off than the old,
    negotiated skeleton did);
  - pads whose copper is stacked or packed closer than their relief gaps
    (connectors' stitching pins, jumper footprints) are estimated with care
    but not checked against KiCad on every board.
- **Pours shown while routing.** Done after routing: the estimated fill on
  the board view, a key, cut-off pieces and the pads they lose marked in
  magenta, islands on request. Still to do:
  - the estimate during routing, not only on the result;
  - (done on `pours-default`: thermal-relief spokes, zone clearance and
    priority in the estimate; lumen-ringlight's pads were those its zone
    does not join at all, now given a trace into the fill.)

- **Pour estimate against KiCad.** Found on a user's board (not public,
  not in `boards/`): routed, then filled and checked in KiCad, four
  disagreements, likely related and all in features just added, so to be
  fixed before more features:
  - where two pads' copper overlaps (or touches), the skeleton does not
    match the fill KiCad makes there: check how overlapping pads are merged
    in the pour estimate and the skeleton;
  - KiCad's DRC reports many more unconnected items than the estimate; every
    one the estimate misses is on the GND or 5V pour, so the estimate thinks
    pads joined that KiCad's fill leaves cut off;
  - KiCad's DRC reports a great many disconnected pour pieces. Find whether
    they come from our traces cutting the pour or are KiCad's own, and look
    at a way to keep pour out of places it does not need to be (zone keepouts
    written with the session, a smaller zone, or the islands removed with
    KiCad's "remove islands" setting read and honoured);
  - the distance and tolerance the estimate uses for the pour (zone
    clearance, minimum width, thermal gap, fill tolerance) seem quite
    different from KiCad's: compare each value read from the `.kicad_pcb`
    against what KiCad applies, including `.kicad_dru` rules and net class
    clearances, which may override the zone's own.
  Make the board a regression check (local only, from its owner), and add a
  sweep metric comparing the estimate's unconnected count with KiCad's.

- **Pour pieces and pins joined by the cheaper way.** Where a pour piece is
  far from the rest of its net, or a pin needs to reach a pour, there are
  two ways to join it: stitching vias to the net's pour or plane on another
  layer, or a trace from the pin (or the piece) to the pour. Try both and
  weigh them (length, vias, the room each takes from other traces, how sure
  the fill is to reach it), keeping the cheaper; today a starved pad gets a
  via or a short trace by a fixed rule. After the "Fix first" pour bugs.
  The ways to weigh, cheapest first:
  1. the pad's thermal relief changed (solid connection, or spokes turned,
     added or widened): no copper moves, but it departs from the zone's
     rules, so off by default, a setting and a command-line flag to allow it
     (written back to the `.kicad_pcb` as a per-pad override);
  2. a via in the pad, or a stitching via beside it;
  3. a trace, or a small filled shape or higher-priority zone of the net,
     bridging to the pour;
  4. the traces in the way moved (as `make_way`), or one taken to the other
     layer with a pair of vias so the fill passes over it.
  A narrower clearance or minimum width for that pad is the same kind of rule
  change as 1: off by default, behind its own setting and flag. Pin swaps,
  net ties and moved parts change the design, so they are only suggested in
  the Warnings section, never done.

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
