# Roadmap

What is planned after 1.0. Bugs are fixed as they are found; this is for
features.

## 1.1

### Editing the routing before it is exported

A way to correct the routing in WeaveEngine itself before writing the SES:
move a via, reroute or delete a connection, pin a trace where you want it,
then let the router fill in around what you fixed.

Why here and not in KiCad: WeaveEngine's traces are taut curves and its
teardrops are fans, and a session can carry them only as many short straight
segments. Once imported, KiCad sees hundreds of tiny tracks per connection,
which its push-and-shove router and drag tools handle badly. Editing before
export keeps the routing as WeaveEngine models it (topology first, then the
taut geometry, rounded corners and teardrops drawn again after each change),
so an edit costs a drag, not a cleanup.

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
- **Traces tapered to fine-pitch pins.** Where a trace is wider than the pin
  it enters, narrow it over its last stretch to the pin's width instead of
  routing the whole net narrower (`router.reach`) or running a wide trace
  into a narrow pad.
- **Sharp vias at junctions.** About one via in five has its traces meeting
  at under 90 degrees, mostly where several connections of a net meet or a
  fan-out stub is pressed against its pad. Re-planning one connection cannot
  move such a via (design 13.3a); moving a junction with all of its
  connections might.
- **Teardrop notches.** Where a teardrop's fan meets a rounded pad corner, the
  round ends of its traces leave small gaps (cosmetic only).
- **A via size per net class.** Route each net with its class's via (the
  DSN's `use_via`), not the board's default: room for a larger via has to be
  planned for in the triangulation, not added once the routing is done.
- **KiCad custom rules (`.kicad_dru`).** Read the rules that let fine-pitch
  parts be routed at less than the board's clearance.
- **The Margin layer as board edge**, without burying pads that sit inside
  it.
