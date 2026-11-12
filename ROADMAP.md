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
- [x] A warning when the DSN has no `.kicad_pcb`/`.kicad_pro` beside it (or one that cannot be read), naming other boards in the folder.

Fix first (bugs in features just added, seen on a user's board, 2026-11-10;
probably partly one cause; before any new feature):
- [x] Skeleton and KiCad's fill disagree where two pads' copper overlaps
  (the estimate's thermal spokes between packed pads: now KiCad's own rule).
- [x] KiCad's DRC finds more unconnected items than our estimate, all on the
  pour (spokes, and fan-out's own traces not counted as joining).
- [x] KiCad's DRC finds a great many disconnected pour pieces (islands): the
  zone keeps them (island removal "never"); now read and said.
- [x] The pour clearance and tolerance we use differ from KiCad's (the
  traces' edge clearance used for the fill).
  (Branch `pour-kicad`; what was found is under Pours and planes, "Pour
  estimate against KiCad".)
Details under Pours and planes, "Pour estimate against KiCad".

To do, in order:
1. [x] Traces pulled fully taut, as far as found (branch `taut`): a trace
   no longer goes round a sleeping via site, or a wire that is not drawn
   (below; design 13.1). The reported case itself is on the back burner.
2. [x] Pours that stay whole by default (done above; what is left is below).
3. [-] Room used where there is room: moved to 1.2, one feature with traces spread for the most space (below).
4. [x] Traces tapered to fine-pitch pins (on branch `taper`, `realize/taper.py`).
5. [ ] The Margin layer as board edge: a directional rule (below).
6. [ ] No acute angles where copper of one net meets.
7. [-] The 45-degree option: moved to 1.2, with the editor (below).
8. [ ] A via size per net class.
9. [x] Most curves still exported as segments, not arcs (branch
   `export-shape`; below).
10. [x] Curves that are not arcs: few; no Bézier or spline track exists in
   KiCad or the session format (below).
11. [x] Teardrops as polygons instead of fans of tracks: as KiCad teardrop zones in a
   routed copy of the board, a setting of three (branch `export-shape`; below).
12. [ ] Pour pieces and pins joined by stitching vias or traces, chosen by cost
   (below).


Back burner: bugs seen once that could not be found again. Each waits for a
new example (a board and where on it); then it moves up to be fixed.
- [ ] At a three-way junction on a via, only two of the three traces got a
  teardrop to the via (seen 2026-11-10). Not reproduced: each trace end gets
  its own teardrop, one never blocks another of its net, and every 3-way via
  junction on lily58-pro, coldfire and a user's board had all its teardrops.
  Known causes of a missing via teardrop, all deliberate: a trace wider than
  0.9 of the via's radius, a trace too short, and fan-out stubs (added after
  the teardrop pass, so they never get one).
- [ ] Slack thin traces, and traces crossing a gap at an angle where they could
  run along the clearance edge (seen 2026-11 on a user's board; the user cannot
  make it happen again). `bench/slack.py` routes a board and finds, for each
  drawn trace, corners with no copper within their clearance and how much
  shorter each trace gets with each kind of thing that is not copper taken
  out. On a user's board (217 connections, 99 vias) and 11 tier-1 boards it
  found: sleeping via sites left in the map (6 on the user's board, 2 that
  `sites.delete` refuses; one trace made a U-turn round one, 0.47 mm from any
  copper); wires not drawn because their net joins those pads already
  (`router._redundant`) holding drawn traces off (31 corners, 3 traces, up to
  0.075 mm); vias of a trace's own net that it passes without ending on
  them, bending it as if they were foreign (lumen-blade13: corners 2.3 mm
  from any foreign copper); and the pours' skeletons, which hold traces off
  on purpose (up to 1.9 mm on lumen-blade13, 18 traces). The first two are
  fixed (design 13.1). Still open: the U-turn above stays where a wire of
  another net inside the U is across the site from others; and letting a
  trace pass straight over its own net's vias was tried and cost the pours
  room (tier 1: lumen-blade13 KiCad open 18 to 20, fomu mask/thermal 2 to
  14), so it waits for a way that keeps fan-out's and the fill's room. If the
  symptom is seen again, run `bench/slack.py` on that board: a slack trace
  held by a skeleton is the pour's room, not a bug.

### Trace shape

- **Traces pulled fully taut.** Users saw thin traces on a complex board that
  were not pulled tight, and traces that cross a gap at an angle where they
  could run along the clearance edge. Done as far as found (branch `taut`):
  relaxation does not go round a sleeping via site unless a wire of another
  net there needs it to, and the traces are pulled once more without the
  wires that are not drawn (`router._taut`; design 13.1). `bench/slack.py`
  checks a board for slack left in drawn traces. What is left is on the back
  burner list.
- **Arcs in the session.** Write rounded corners and the curves of taut traces
  as arcs (`qarc`: start, end and centre) instead of many short straight
  segments. KiCad's session import turns a `qarc` into a true track arc of
  any angle up to 180 degrees (checked with KiCad 10: arcs of 7.5, 37, 90,
  135 and 180 degrees came in as written). Beyond that it takes the shorter
  way round between start and end (250 degrees came in as 110), so a longer
  arc is written in pieces; pieces of at most 90 degrees, since the Specctra
  format means a quarter arc and other tools may hold to it. Far fewer tracks
  per connection, and ones KiCad's drag tools handle as a curve.
  Whole wraps (user report, 2026-11-10: most of what looks like an arc in
  the app still came out as short segments). Measured and fixed on branch
  `export-shape`. The guess was right: a round pad is held as a 16-gon
  (inflated with mitred corners), so a trace wrapped round it goes round each
  corner on a disc of its own, with straight runs along the sides between.
  The fit only found the arc of each single disc, so a wrap came out as a
  short arc per corner and a segment per side; a trace hugging the polygon
  itself (no disc) came out as segments only. Oval pads (THT DIP pins) are
  the same at their ends. Measured as the share of the traces' turning (at
  vertices under 30 degrees, and along arcs) written as `qarc`, tier-1 boards,
  main -> branch: pic-programmer 34 % -> 90 %, sweep-mini 50 % -> 82 %, sonde
  50 % -> 95 %, ringlight 71 % -> 71 %, ftp 47 % -> 88 %, ecc83 83 % -> 84 %.
  KiCad tracks + arcs after import (teardrop fans included): pic-programmer
  1174 + 52 -> 982 + 92, sweep-mini 1753 + 147 -> 1535 + 186, sonde 578 + 40
  -> 413 + 48; KiCad DRC and unconnected counts unchanged on all six.
  How (`io/arcs.py` `fit_arcs`, `io/clearance.py`): a run is whole turns of
  the polyline in a row (single-disc arcs and corners), turning one way; it
  is fitted round the centre of a round pad, a via or an oval's end found
  near it (`clearance.round_parts` finds an oval's ends and a rounded
  corner's centre in a pad polygon), else round a fitted centre if it is a
  circle to 10 microns. The arc is the largest circle every edge of the run
  keeps out of, so it lies inside the run, at the run's own least distance
  from the pad's true circle; it joins the trace by tangents, or by the line
  tangent to two circles where two runs meet (a trace passing between two
  pads). Each arc is checked against the copper of other nets as written so
  far (pads as their true circles and ovals, traces, teardrops, wedges,
  keepouts, the edge): it may come no nearer than the rule (with the margin)
  or, if the drawn traces were nearer, than they were. The fit runs twice, so
  an outer trace of a stack can follow the inner one in, then every trace is
  checked whole and falls back to its single-disc arcs if it fails. The
  independent check (`io/check.py`) now also measures ovals by their true
  round ends, as KiCad does.
  What is left as segments: single sharp corners at rectangular pads (the
  router bevels their corners; the true clearance contour is an arc round
  the corner, not fitted yet), wraps the clearance check turns down (mostly
  where a neighbour in a stack keeps its polygon corners), and traces along
  a polygonal board edge (ringlight's round outline: the arc fits to 6
  microns but its neighbours stop it).
- **Curves that are not arcs** (measured on `export-shape`). After the
  whole-wrap fit, 5 to 18 % of the turning is still written as segments
  (ringlight 29 %, its edge-following traces), and nearly all of it is
  circular in fact (above): the router only draws arcs of discs and fillets,
  so there is no real share of non-circular curve to fit with several arcs.
  Bézier or spline output is not possible anyway: KiCad 10's tracks are
  lines (`PCB_TRACK`) and arcs (`PCB_ARC`) only (Bézier exists for graphic
  shapes, not tracks), and a Specctra wire is a `path`, `qarc`, `polygon`,
  `rect` or `circle`.
- **Teardrops as polygons** (branch `export-shape`). Not through the
  session: KiCad 10's SES import drops a `wire` of any shape but `path` and
  `qarc` without a word (a `polygon` wire, with or without `(type route)`, and
  a `rect` wire were tried on pic-programmer: no zone, no drawing, no track
  came in; the import's own source says such a wire is a zone the board
  already has). So the teardrops are now a setting of three (`teardrop_style`,
  `--teardrops`): WeaveEngine's as tracks in the SES (the default, as before);
  WeaveEngine's as KiCad teardrop zones (`(attr (teardrop (type padvia)))`, as
  KiCad makes its own) in a routed copy of the board; or none (KiCad's own
  Edit > Edit Teardrops can add them; no pad or via is marked for KiCad's, so
  none means none). `--no-teardrops` and `--no-ses-teardrops` are kept and
  mean none. The routed copy (`io/kicad_board.py`, `--routed-board FILE`,
  setting "Routed copy of the board"; default NAME-routed.kicad_pcb beside the
  SES, written for zones) is the original `.kicad_pcb` text with the unlocked
  tracks, vias and old teardrops taken out as KiCad's session import does, the
  zones' stale fills taken out, and the routing (tracks, arcs, vias) and
  teardrop zones put in; the `.kicad_pro` and `.kicad_dru` are copied beside
  it so KiCad checks it by the same rules. The original board is never
  written. A taper's wedge stays a fan of tracks: as a `track_end` teardrop
  zone KiCad's check found the narrow piece unconnected (pic-programmer: 2
  unconnected, 4 dangling tracks). KiCad 10 has no teardrop generator outside
  its PCB editor (neither kicad-cli nor the Python module), which is why
  "left to KiCad" can only mean none plus that note.
  Checked with KiCad 10.0.6 (copy loaded, zones filled, DRC), against the SES
  imported with fans: pic-programmer, sweep-mini, sonde, ringlight, ecc83 all
  have the same violations and unconnected counts; KiCad items per board,
  fans -> zones copy: pic-programmer 982 tracks + 92 arcs -> 231 + 92 + 169
  teardrop zones, sweep-mini 1535 + 186 -> 376 + 186 + 168, sonde 413 + 48 ->
  104 + 48 + 88.
  Teardrops after routing: they are made last, round the finished routing,
  so the app makes them again (`router.redo_teardrops`), and only them, when
  the teardrop settings change after routing (style on or off, longest,
  widest, breathing room); the drawing, the pour estimate and what Export
  writes follow. The style itself and arcs in the session are read when
  exporting. Made again with unchanged settings, the teardrops come out the
  same as the router's (checked on three boards). Not made live, since they
  change the routed geometry the design-rule repair and fan-out checked:
  rounding corners, tapering, straightening vias, fan-out and pour skeletons.
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
  in the SES (`Result.widths`).
  Planned wide (branch `wide-plan`): such a net is now planned at its own width
  except near its fine-pitch pads, so the room for the wide part is kept.
  `Rules.net_full`/`fine` give the full width and a circle round each fine pad
  (its reach + one trace width + the wedge, as `taper` measures it). Each
  `TopoState.zone` holds, per net, the gates in those circles, where its wires
  weigh the narrow weight (`TopoState.at`). insert/remove/tally/fits, the
  search kernel (`narrow`, `zone` mask), relax (`half_at` per gate), the exits
  (`Rules.width_at`) and smoothing all use it. DRC still checks the narrow drawn
  width, which is what is drawn where the taper does not fit. If a connection is
  left open after rip-up, the nets planned wide that it belongs to or that block
  it are planned narrow all along again (`Context.demote`, kept in snapshots),
  and rip-up runs once more. Tier 1 against main: nothing worse;
  narrowed-net traces went from 7 tapered / 4 narrow all along to 11 / 0. The
  counts are in the CLI output, the sweep records (`tapered`, `narrow`) and compare.py.
  Left: tier 2 (and tier 3) not swept yet; that is the next step before merging.
  The selection and demand estimates (`select`, demand map) still count the full
  weight near fine pads. Circles apply on every layer, not just the pad's own.
  The app, the SVG and the pour estimate draw each piece of a tapered trace at
  its own width, the wedges as drawn; a trace left narrow all along is drawn narrow.

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

- **Pour estimate against KiCad.** Done on branch `pour-kicad`. Found on a
  user's board (not public, not in `boards/`), routed in the app as the user
  did (edge clearance 1.5 mm in its settings), the SES imported into KiCad 10,
  zones refilled, DRC run, and compared pad by pad and piece by piece with the
  estimate (and with KiCad's `zone_filler.cpp`, 10.0). Causes, all in the
  estimate:
  - *Thermal spokes.* The estimate joined a pad whose spoke ended in a
    neighbour's relief by a spoke bent from centre to centre: the strange
    shapes between packed pads, and pads joined that KiCad leaves cut off (a
    USB connector's surface and through pins). KiCad keeps a spoke only where
    its end is in the fill, or in a spoke of another pad whose end is in it
    (two in line, facing). Its spokes run from the middle of the pad's box, in
    the pad's own frame, to that box grown by the gap, 0.005 mm and half the
    least width; at the pad's spoke angle (`thermal_bridge_angle`, else 45
    degrees for a round pad, 90 for others); no wider than the pad, and none
    if narrower than the least width; cut by other nets' clearances and pruned
    with the fill. Now the same (`pours.spokes`), with the pad's angle read.
  - *Edge clearance.* The fill was kept the traces' edge clearance from the
    edge and from holes in the board (1.5 mm from the app's setting); KiCad's
    fill keeps the board's own (0.5 mm here), so 1.2 mm too little pour all
    round the edge and round every mounting hole (about 600 mm2 a layer).
    Now `Board.pour_edge`, from the .kicad_pro and custom rules; Margin
    drawings are kept as clear.
  - *Solder-mask openings* were taken as keepouts of the fill; KiCad fills them.
  - *Fan-out's own traces* (a short trace from a cut-off pad into the fill)
    were not counted as joining, so pads they join stayed "cut off".
  - *Blobs:* KiCad drops a piece of fill less than the least width across
    either way (after shrinking it by half of it); now the estimate does too.
  - *Islands.* This zone keeps its islands (`island_removal_mode` 1, "never"),
    so KiCad reports every one as isolated copper, while the app said KiCad
    removes them and hid them. Now each zone's island removal is read; islands
    KiCad removes are not counted, the ones it keeps are named as KiCad's
    isolated copper (a zone joined to nothing at all is left whole, as KiCad
    leaves it). KiCad's fill alone makes 67 islands on this board with no
    traces at all; the routing makes about 120 more, small pieces between
    traces. Keeping pour out where it is not needed (keepouts written with the
    session, a smaller zone) is not done: setting the zone to remove islands is
    the simple way, and the app now says so.
  - What was right: the clearance to other nets (0.15 mm: the board's minimum
    over the zone's 0.125, both measured on KiCad's fill), the thermal gap
    (0.3), and the least width (0.125). A custom rule asking 0.3 mm from the
    zone to surface pads is skipped by the estimate, and KiCad 10's fill does
    not apply it either (measured 0.15). Net classes' clearances were taken as
    the largest of them for every pour (`Rules.clearance`); see the follow-ups.
  Measured on the Mac against main (99d465d), `--jobs 4`: tier 1 routes the
  same, KiCad open 104 to 104, DRC 9 to 9, isolated copper the same, cut-off
  pads 53 to 47; tier 2 (fresh main baseline) routes the same on all 16,
  KiCad open 364 to 352 (castor-mainboard 36 to 30, tinytapeout 64 to 59,
  bling-mx 4 to 3), DRC 275 to 275, cut-off pads 125 to 93, none worse;
  time 2023 to 2174 s (+7 %). The estimate's open items are 74 from KiCad's
  over tier 2, against 171 before.
  On the user's board, as the user routed it: KiCad 3 unconnected and 184
  isolated copper against the estimate's 1 cut-off pad (the wrong one) and
  223 islands; now (fan-out joins the pads KiCad would cut off) KiCad 0
  unconnected, 183 isolated, the estimate 0 and 184, pieces 167 against 169
  on the front and 28 against 28 on the back. The CLI says what the estimate
  expects KiCad to find unconnected, and the sweep compares it and the
  islands with KiCad's on every board ("estimate off by").
  Follow-ups (same branch), measured against the numbers above:
  - *Net-class clearance per pour.* The fill keeps, from each other net, the
    larger of the two nets' class clearances (and the zone's, and a pad's own
    local or rule clearance), never more than the board's
    (`Board.net_clearance`, `default_clearance`, read from the DSN's classes;
    `pours.estimate`). Test: `test_a_pour_keeps_the_clearance_of_its_nets_classes_from_each_net`.
  - *Island removal "below area limit"* (`island_removal_mode` 2 with
    `island_area_min`) was already read and applied (islands of less than that
    area removed); checked, nothing to change.
  - *Estimate too high on kicad-cm5-minima (+17) and sweep-bling-mx (+23).*
    Cause: the "open" count added the pads fan-out found no room for a via
    beside (`plane_left`), but a pad with no room is often joined all the same
    through another pad of its piece (a through pad) that did get a via, and
    the count was made before that; and a pad drawn in pieces (U1-23,
    U1-23@1 ... @4) was counted once per piece, KiCad once. Now the router
    drops from `plane_left` the pads the final estimate finds joined, and the
    CLI counts a pad drawn in pieces once. bling-mx: 26 to 4 (KiCad 3).
  - Measured, Mac, `--jobs 4`, against the runs above: tier 1 routes the same,
    KiCad open 104 to 104, DRC 9 to 9; the estimate's open items off by 52 to
    38 over its 12 boards, islands off by 208 to 40 (sweep-half-swept open +12
    to 0). Tier 2: all 16 route the same, KiCad open 352 to 352, DRC 275 to 275;
    the estimate's open items off by 74 to 40 (cm5-minima +17 to +4,
    bling-mx +23 to +1), islands off by 5. What is left is mostly
    kicad-interf-u (+15) and fomu (+23): not looked into. Note that
    `bench/sweep/run.py` runs the router in each board's work directory, so
    `PYTHONPATH` must be an absolute path to the checkout.

  Left: the sweep's prep strips tracks but keeps KiCad's teardrop zones on
  them, and KiCad writes each in the DSN as a plane of its net
  (kicad-complex-hierarchy: 165 of them); a user who deletes the tracks
  and not the teardrops gets the same. Not changed here, as it remakes every
  prepared DSN and so needs a new baseline; the router could also skip zones
  that are teardrops (the .kicad_pcb says so).

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

### The 45-degree option

Moved from 1.1 to come with the editor. Traces drawn as straight runs at
multiples of 45 degrees, as many boards are expected to look, instead of taut
curves: the same topology, realised on octilinear paths, with the clearances
checked again. A second way of drawing, off by default (a setting and a
command-line flag); like the other drawing settings, switchable after routing
where the clearances allow.

### Traces spread for the most space

A routing goal that gives each trace as much room as it can have, instead of
the shortest length: a setting (and a command-line flag) choosing between
shortest traces (now) and most space. With most space, the traces through
each gate are spaced evenly across its width, and between gates each trace
runs down the middle of the room it has (the largest clearance to its
neighbours on both sides), longer where that buys space. The topology can
also be chosen for it: a trace may take a wider gate over a shorter, narrower
one. Wider gaps are easier to make, cross-talk less and are easier to inspect
and edit by hand.

This is also 1.1's "Room used where there is room", moved here as one
feature with it (user, 2026-11-12): traces are drawn at exactly the clearance
even where the board has space, so they bunch in narrow places and leave open
areas empty. Where a gap holds fewer traces than it could, spread them across
it: easier to make, less crosstalk, and easier to read.

## Later

- **A KiCad plugin.** Route from inside KiCad's PCB editor: one button that
  exports the board, routes it with WeaveEngine (showing its progress) and
  imports the session, with no DSN or SES files to handle by hand. Through
  KiCad's IPC API (KiCad 9 and later), which can also read the board's rules
  and zones directly instead of from the files beside a DSN.
