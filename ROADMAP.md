# Roadmap

What is planned after 1.0. Bugs are fixed as they are found; this is for
features.

## 1.1

### Trace shape

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
- **Teardrop notches.** Where a teardrop's fan meets a rounded pad corner, the
  round ends of its traces leave small gaps (cosmetic only).
- **Traces tapered to fine-pitch pins.** Where a trace is wider than the pin
  it enters, narrow it over its last stretch to the pin's width instead of
  routing the whole net narrower (`router.reach`) or running a wide trace
  into a narrow pad.

### Pours and planes

- **Pours that stay whole.** A ground or power pour is left to KiCad to fill
  around the traces, and the routing often cuts it into islands: pieces of
  copper joined to nothing, or sections cut off from the rest of the net.
  The plan: route each pour as one more wire, a skeleton on its layer as wide
  as the pour's minimum width, joining its pads and taking its room in the
  gates like any other wire. Wires never cross, so no trace can cut it, and
  KiCad's fill covers it. Where a dense board has no room for it, its pad is
  joined by a via to the net's plane on another layer, as now, and named.
  Design 11.3.
- **Pours shown while routing.** Draw each zone (its net, layer and outline)
  on the board view, and, as the routing grows, an estimate of the copper
  KiCad will fill there, with islands and cut-off sections marked, so a
  broken plane is seen before export.

### Rules

- **A via size per net class.** Route each net with its class's via (the
  DSN's `use_via`), not the board's default: room for a larger via has to be
  planned for in the triangulation, not added once the routing is done.
- **KiCad custom rules (`.kicad_dru`).** Read the rules that let fine-pitch
  parts be routed at less than the board's clearance.
- **The Margin layer as board edge**, without burying pads that sit inside
  it.

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
