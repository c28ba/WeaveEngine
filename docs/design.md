# WeaveEngine: Design Document (v0.1)

**WeaveEngine** (Python package `weaveengine`) is a topological PCB autorouter (the approach TopoR brought to commercial PCB design) that converts a board into a planar topological map, solves routing as a choice of *topology* (which way each wire passes each obstacle), and then realises that topology as the shortest legal geometry.

---

## 0. Status and honesty notes

- This is a design, not a tested result. Anything marked **[HYPOTHESIS]** is a design bet that must be validated by the experiments in section 17 before it is relied on.
- TopoR's internal algorithms are not public in anything I could verify. This design is built from general topological-routing ideas (homotopic routing, rubber-band sketches, negotiated congestion), not from TopoR's source. Do not describe it as a reimplementation.
- Boards named in the measurements below (ALU, blinkSP1, Micromouse2026PCB, RAM Selector Tree, Word of RAM, ulx3s) were the development boards and are not in the repository. The example boards in `boards/` are four open-source designs (see `boards/README.md`); the 44-board sweep of 4.1 used open-source boards too.
- The riskiest piece is the slot-aware topological search (section 8). Milestone M2 is a deliberate spike to prove or kill it early.
- Geometry. Since M14e a trace is the taut line against discs at the vertices it passes (13.1). On the five benchmark boards every written result checks clean, and the board that needs most vias routes 368 of 400 in half the time, with no repair round (12.8, which also records that rip-up stops too soon there).
- Vias. The machinery for putting a via into the map is built and tested (12.2, M13). The layer that decides *where* vias go has been prototyped (12.3) and is not good enough: it uses several vias per connection and no board that needs vias routes completely. It is to be reworked (12.5). Section 12.4 records what the prototype showed, with numbers.
- Library API details (`triangle`, `shapely`) are from memory. Check them when implementing.

---

## 1. Goals, non-goals, success metrics

**Goals**
1. Read a board, route all nets, write the routed result back.
2. Routes are crossing-free by construction and DRC-clean after realisation (trace width and clearance from a single net class in v1).
3. Total trace length is as short as practical. Any-angle traces are allowed (not limited to 45°/90°).
4. Pure Python source. Dependencies: `numpy`, `scipy`, `shapely>=2`, `triangle` (a wheel wrapping Shewchuk's Triangle). Optional `numba` behind a flag. No C/Cython written by us.

**Non-goals for v1**
- Differential pairs, length matching, impedance control, copper pours/zones, per-net-class widths, blind/buried vias, curved (arc) traces (post-processing only, M8).
- Beating commercial routers. The aim is a working, understandable, correct router.

**Success metrics** (all measured on the benchmark suite, section 17)
- Completion rate: % of connections routed with zero DRC violations.
- Length ratio: total routed length / total airwire (straight-line) length. Lower is better. This ratio is the main "shortest traces" number.
- Via count (once vias exist).
- Wall-clock runtime and peak memory.

---

## 2. The central question: is one-net-at-a-time, forward-only routing bad?

**Yes, if it is pure greedy.** Routing nets one at a time and never revisiting a choice has these problems:
1. The result depends on net order. A good early choice for net A can make net F impossible.
2. An early wire is a wall. Walls make later regions unreachable, and nothing in a greedy search "sees" that.
3. There is no recovery path when a later net fails.
4. Shortest-first behaviour spends the scarce channels (gaps between pads) on whoever asks first.

**A clarification that shapes the whole design.** On one layer, a single wire between two interior pads does *not* disconnect the board. A curve between two interior points never cuts the plane. Disconnection happens when wires, pads and full channels chain together into a *closed barrier* (including via the board outline). Before that point, the harm is softer: a wire forces any net whose straight line crosses it to detour around its ends. So there are two things to minimise:
- **Soft harm:** how many unrouted connections the new wire's path separates (their airwires cross it).
- **Hard harm:** closing a barrier loop that severs connections outright.

**Design response (four layers, each independently testable):**
1. **Global candidate planning before committing anything** (Phase 1): generate several topologically distinct candidates per net, then choose one candidate per net for *all* nets at once, minimising length, congestion and mutual conflicts.
2. **Commit in regret order** (Phase 2): route the nets that have the most to lose first, instead of shortest-first or input order.
3. **Lookahead costs** (section 9): demand-aware congestion, an airwire-crossing penalty (your "keep other pads on the same side" idea, made precise), and a barrier-closure penalty.
4. **Nothing is final** (Phase 3): negotiated-congestion rip-up and reroute, so early decisions can be undone.

---

## 3. Python implementation rules (efficiency)

1. **Two data tiers.** NumPy arrays for setup and bulk work (triangulation, capacity, geometry relaxation, vectorised tests). Convert to plain Python `list`/`tuple` with `.tolist()` for anything touched in an inner loop. Scalar indexing into a NumPy array is several times slower than into a list.
2. **Hot loops use only local variables, lists, tuples, ints, floats, and `heapq`.** No dataclasses, no attribute lookups, no `shapely` calls inside the pathfinder.
3. **Precompute transition tables.** Everything the search needs about a triangle (successor edges, which corner each transition cuts, which end of each edge that corner is) is stored in per-half-edge tuples once.
4. **Incremental updates only.** Committing or ripping up a wire touches only the gates and triangles it crosses. Never rebuild global structures per net. The only rebuild-per-round structure is the barrier union-find (section 9.5).
5. **Sparse search state.** Use `dict`s keyed by an integer node id, not arrays sized to the whole map.
6. **`shapely` is used in preprocessing and DRC only**, in batch (vectorised calls and `STRtree`), never per search step.
7. **Parallelism.** Candidate generation (Phase 1) is read-only per net: use `ProcessPoolExecutor` with `fork` so arrays are shared copy-on-write. Phase 2 and 3 are sequential in v1. Wider use of multiple cores is a later step: see section 22.
8. **Profile before optimising.** `cProfile` and `line_profiler` on the benchmark boards. Allowed accelerations in order: better algorithms, an A* landmark heuristic (via `scipy.sparse.csgraph`), `numba` on the relaxation kernel, only then anything else.

---

## 4. Input and output

**Format: Specctra DSN in, SES out** (the same interchange TopoR 4.0 added, which most CAD tools can produce). It keeps the router independent of any one CAD tool.
- A DSN parser is a small s-expression reader. Needed sections: `boundary`, `structure` (layers, rules), `placement`, `library` (images and padstacks), `network` (nets, classes), and optionally `wiring` for fixed existing copper.
- **Open item:** confirm which CAD versions you use can export DSN and import SES (KiCad has historically supported both; check your version).
- Routing already in the file (`wiring`): traces and vias marked fixed (`type` `protect`, `fix` or `shove_fixed`) are obstacles. Anything else is what an earlier run left behind and is discarded, with a note, and those connections are routed afresh. Keeping its vias while redoing its traces would only leave walls on the board (this happened with `boards/blinkSP1.dsn`, a saved Freerouting result). **Open:** fixed traces written as `polyline_path` are not read and are ignored; the connections they make are routed again.
- A clearance written as `clear` (some tools) is read like `clearance`.
- **Missing rules fall back to defaults (M12, done).** A DSN with no `rule`, or with only a width or only a clearance, is valid and is routed, not rejected. Each value is resolved separately, in this order: the user's override in the settings, the DSN (default class, then the `structure` rule, then the other classes), the built-in default.

  | Value | Built-in default |
  |---|---|
  | Trace width | 0.2 mm |
  | Clearance | 0.2 mm |
  | Via diameter / drill | 0.6 / 0.3 mm (also used when the `via` padstack is missing or has no circle) |
  | Edge clearance | the clearance (as now) |
  | Unit / resolution | um / 10 (as now) |

  The built-in defaults live in one place (`board.Rules`), which the settings editor's starting values also read. The reader records what it defaulted in `Design.defaulted`; whatever the user's settings do not then set is reported, as a line on the command line and a note in the app's log. A `via` that names a padstack the library does not define counts as missing. The only things still refused are a file that is not a DSN, one with no signal layer, and one with no boundary, because nothing sensible can be assumed for those.
- A DSN does not carry the copper-to-board-edge clearance or drill sizes. The edge clearance is taken from `--edge-clearance`, or from the KiCad project file (`--kicad-pro`, or a `.kicad_pro` next to the DSN); otherwise it defaults to the trace clearance. The clearance of plated holes cannot be checked from a DSN (holes without copper can: see the next item).
- **Cutouts and holes without copper.** KiCad writes a cutout in the board (Edge.Cuts inside the outline, a footprint's included) as a `structure` keepout on the layer `signal`, and a keepout area as one keepout per copper layer. So a keepout on `signal` (or `pcb`) is taken as a hole in the outline: it is board edge and keeps the edge clearance; everything that measures the edge measures its whole boundary, and the app draws it as a hole. A part's own keepouts are placed with the part (mirrored on the back) and are keepouts; KiCad writes each hole without copper (NPTH) so, once per layer, with its diameter already enlarged by the hole clearance (1.4 mm drill: 1.9 mm keepout). Until 2026-10 cutouts were keepouts held to the trace clearance and a part's keepouts were not read at all; KiCad's DRC of a Micromouse2026PCB routing found 36 edge-clearance violations at cutouts (0.31 mm against 0.5) and 33 hole-clearance violations, traces across the NPTH holes. Another tool that writes an all-layer keepout area on `signal` gets the edge clearance for it instead of the trace clearance.
- **Rule areas that forbid tracks or vias, not both.** KiCad (checked with 9.0.8) writes a rule area that forbids tracks and vias as a `keepout`, one that forbids tracks only as a `wire_keepout`, and one that forbids vias only as a `via_keepout`, on the board or in a part's image. Only `keepout` was read, so traces ran through the second kind and vias stood in the third. A `wire_keepout` is now a keepout (kept out of by vias too, which errs on the safe side). A `via_keepout` is an area no via's copper may touch, on whatever layer it is drawn, since every via goes through all of them; traces cross it as if it were not there (`Board.no_via`, kept out of by `plan.path.Legal` and `plan.fanout`).
- **Round copper is never under-counted.** A circle is read as a 16-gon around it (`circle_polygon`). A `path` (KiCad's oval pads; an oval of equal sides is a path of no length) was a 16-gon inside its round ends, smaller by r(1 − cos 11.25°): 17 µm on a 1.73 mm pad, more than the 10 µm routing margin, and KiCad found traces 0.193 mm from such pads against 0.2. A path is now discs around its points joined by its straight stroke, and a path of no length is a round pad with its radius (for teardrops, and the checker measures it as a true circle). The fix is the shape, not a larger clearance: a margin big enough for one pad size is too small for a bigger one.
- The checker of the written SES (`io/check.measure`) measures traces against pads, keepouts and fixed copper of other nets, and against the whole board edge, cutouts included. It did not see keepouts or cutouts before, and passed the routing above.
- Routing uses a small extra margin (`--margin`, default 0.01 mm) so that nothing sits exactly on a limit; the written SES is read back and measured against the unpadded rules.
- SES writer emits each wire as a polyline `path` with width. Arcs are approximated by polylines until M8.

### 4.1 What the KiCad board says that its DSN does not (`io/kicad.py`)

Found by the board sweep (2026-11): 44 open-source KiCad boards (KiCad's own demos, HackRF, Solo, Corne, Lily58, Sweep, Olimex, UPduino and others), each exported to DSN, routed, imported back into KiCad, and checked by KiCad's DRC with the zones refilled, against the same board with its traces removed. The sweep is `bench/sweep/` (`run.py`, `compare.py`): the boards in three tiers by routing time, so a change is tried on the quick ones first (`--ramp --baseline` goes on to the next tier only while nothing is worse). On GitHub, `.github/workflows/sweep.yml` runs each board on a runner of its own when a branch named `sweep/...` is pushed, and commits the results, compared with the latest sweep of main, to the branch `sweep-results`. Each board is stripped and exported once and kept (`WE_SWEEP_DIR/prepared`, under a hash of the board's files, `prep.py` and the KiCad version), and every later run routes that same DSN: KiCad's export is not the same from run to run. Where the segments of an outline nearly meet (lily58-pro has four joints whose ends miss by 0.3 to 1.3 µm), it writes one end or the other, as the order its shapes happen to lie in memory has it (shown by adding the same shapes to a board in shuffled orders: two DSNs). lily58-pro routed to 90 vias from one DSN and 81 from the other, which the review took for a router that is not deterministic; each DSN routes the same, to the byte, every time and with any number of workers. kicad-tinytapeout was also seen exported two ways, and kicad-openair, which has no such joints, gave other points on its rounded corners once, with many exports running at the same time; so closing the joints before the export would not be enough, and would change the DSN of five boards on which KiCad usually takes the other end. The kept DSNs are the ones main's baselines were run on; `--keep-from LABEL` takes a board not kept yet from an earlier run's work folder, so that a new machine's first run routes the DSNs its baseline routed. On GitHub each runner prepares its board afresh. KiCad's DSN export leaves out or misstates much of what that check holds the board to:

| Lost in the DSN | What happened | Now |
|---|---|---|
| Teardrop zones (KiCad 7+) | written as planes like any zone: their net was taken to be poured and left unrouted; boards came out with 0 connections | a plane counts only if it spreads over two of its net's pads and is larger than they are (`dsn.py`) |
| Rule areas that allow tracks and vias (forbidding parts or pours only) | written as plain keepouts | a keepout whose bounds are a rule area that does not forbid tracks is left out |
| Local clearances, hole clearance, drill sizes | not written | read per pad from the `.kicad_pcb`; `Pad.clearance`, `Rules.apart` |
| The board's minimum clearance, via ring and drill, minimum track width | not written | from the `.kicad_pro`; vias are made as large as the minimums ask (the padstack name carries the drill to KiCad's SES import) |
| Text and drawings on copper | not written | obstacles of their net (or none); text as a box a little larger than KiCad's stroke font draws it (checked to cover all 27 of KiCad's own text boxes on the demo boards) |
| Holes on layers their pad has no copper on | not written | obstacles there, at the hole clearance |
| Custom pads | written as the convex hull of their copper | the real outline (anchor and primitives), checked against that hull; joined at the anchor, since a bent pad's centroid is off its copper |
| Solder-mask openings | not written | a pad whose opening reaches past the clearance (its margin, its part's, else the board's `pad_to_mask_clearance`) asks that of foreign copper; mask left open on purpose (drawn on a mask layer, or a pad of mask only) is a keepout on the copper below, or copper of the one net it bares. KiCad calls a trace bared in another net's opening a solder mask bridge: 429 on sweep 3, 195 of them on Corne-cherry, whose 0.2 mm openings sat exactly at the 0.19 mm clearance (plus margin); the designers' own routing had none |
| Hole shapes | not written | a round hole is a polygon drawn round its circle, an oval one a slot (a line with round ends); a pad's local clearance holds its hole too |

The custom pads were the worst. A reversible Pro Micro footprint (the Sweep keyboards) bends each jumper strip round the next pin's hole; its hull is a solid trapezoid over that hole, and traces and teardrops were routed into copper that is not there. On sweep-bling-mx: 85 of 128 connections, 42 pads unreachable, 21 shorts and 199 dangling teardrop tracks before; 128 of 128, no shorts and no new DRC errors after, in 12.5 s instead of 49. Custom pads are on 16 of the 44 boards.

Placement is verified against pcbnew itself: about 30,000 pads on 44 boards, positions and drills exact. A footprint's rotation turns its pads and drawings (rectangles corner by corner); a custom pad's primitives turn with the pad, whose angle in the file includes its part's, and the mirrored reading is tried as well, the one whose hull is the DSN's being taken.

Without the `.kicad_pcb` beside the DSN none of this is known, and the result can break those rules; the command line and the app say what was read.

**Fine-pitch pins (`router.reach`).** An RP2040 (0.2 mm pins at 0.4 mm pitch) cannot be joined by a 0.25 mm trace at 0.19 mm clearance: a trace joins a pad at its centre, 0.3 mm from the next pin's edge. Designers route such pins by hand with thinner traces. Where a net's pads cannot be reached, it is routed halfway down to the board's minimum track width (`Rules.min_width`: the project's, never below 0.127 mm; not known without a KiCad project, and then nothing is narrowed), else at it, else at it with half the safety margin given up (routed narrower, written at the minimum: `Rules.net_drawn`). Corne at 0.2 mm and 0.01 mm margin needs exactly 0.3 mm, so only the last step reaches it: 119 unreachable pads became 4, and 189 of 309 connections 281, clean in KiCad's DRC; but routing took 950 s instead of 85, the RP2040's fan-out being most of the board. A net routed with part of its margin given up gets no teardrops: there is none to spare. Pins closer than the clearance itself (lily58-pro-v2, tinytapeout) are routed in KiCad under custom `.kicad_dru` rules, which are not read; they stay open and are named.

**Pads that touch.** Pads of one net whose copper meets (front and back of a pin, a custom pad's pieces, a pad drawn twice) are one piece of copper: the spanning tree (section 11) is over the groups they make, and no connection is made inside one. Before, royalblue's connectors asked for a trace from each pin to its own copy. A route may still pass through such a pair as a layer change; the piece between them has no length and is not written.

**Measured.** Routing-caused DRC errors over the 44 boards (errors of a kind the stripped board did not already have): 3,075 before this section's first round (planes, rule areas, local and hole clearances, board minimums, text), 1,160 after it, and 714 after the second (custom pads, narrowing, pads that touch). Of those 714, 275 are the board edge on the Margin layer (Solo, not read: the margin layer as edge buried pads), 215 hole clearance at Corne-chocolate's switch holes (a hole was a polygon inside its circle, 0.03 mm short on a 3.4 mm hole; now drawn round it), 77 on Lily58-lite and 75 on Olimex POE-M2. hackrf-one went from 659 to 0, the Sweep keyboards from 261 and 183 to 0, lumen-mobo from 499 to 0.

A third round (hole shapes drawn round their circles, net ties, teardrop fills checked as written, vias down to the planes: section 11.2) brought the 714 to 363, then 337. Of the 337, 284 are the Margin layer, and 49 are closed solder jumpers on olimex-poe-c and upduino: a jumper's copper bar joins its two pads, KiCad gives it no net, and its check reports any trace that meets one of those pads, however it is routed (WeaveEngine keeps other nets clear of the bar, as of a net tie's copper). The rest were three teardrop fill lines that KiCad called dangling (on a long pin met from its side, the outer lines ran past the pin; now they end on it) and one via on tinytapeout whose net class asks for a larger drill than the board's via, which KiCad drills on import, leaving a 0.125 mm ring: every via is the board's default (a via per net class is on the roadmap). Solder-mask bridges and starved thermals, which are warnings, are counted apart: 429 and 118 over the 44 boards before the third round, 45 and 110 after it.

The connection counts changed with the touching pads, and are now KiCad's own: on lily58-pro KiCad counts 311 connections to make on the board stripped of its traces, as WeaveEngine now does (427 before: pads of one piece of copper were joined to each other), and 4 left after routing, all at the ground pour; sweep-bling 44 against KiCad's 45, half-swept 28 against 29.

Internal model: `Board` (outline, layers, rules), `Obstacle` (shapely polygon, net id, layer set), `Net` (list of pad ids). Units are float64 millimetres with a global epsilon of 1e-6.

---

## 5. Preprocessing

Single net class in v1: trace width `t`, spacing `s`.

1. **Collect copper per layer:** pads, fixed vias, fixed tracks, keepouts, board cutouts.
2. **Inflate for centreline routing.** A wire centreline must stay at least `s + t/2` from foreign copper, so inflate every obstacle by `s + t/2` (`shapely.buffer`, with `join_style=mitre` limited, or round with a low `resolution`). In inflated space a wire is a zero-width curve. Shrink the board outline inward by `edge_clearance + t/2`.
3. **Merge overlapping inflated obstacles** (`unary_union`). Overlap means no centreline fits between them. Each merged piece gets an integer **obstacle id**. The outline is obstacle id 0.
4. **Terminal ownership.** For each pad, record which segments of the (merged) obstacle boundary lie on that pad's own inflated boundary. A wire of net N may start or end only through boundary segments owned by a pad of net N. All other boundary segments are walls for N.
5. **Same-net rule:** a pad of a wire's own net is that net's copper. The wire may end on it, and it may run into it and go on from any of its edges (section 11, "Its net's copper"). Until then (v1) it could only end on one, and went round every other.

---

## 6. The planar map

### 6.1 Triangulation

- Build a **constrained Delaunay triangulation** of free space: vertices are the obstacle boundary vertices and outline vertices; constrained segments are the obstacle boundaries; obstacle interiors and the outside of the outline are holes.
- Use `triangle.triangulate(pslg, 'p')` with each segment's marker set to its obstacle id (Triangle should carry markers to output segments and vertices; verify), plus `'n'` to get neighbour indices.
- **No Steiner points by default.** Every vertex then lies on an obstacle, so every gate (below) is a true choke between two obstacle boundaries. Long skinny triangles are fine in this representation. An optional `max_area` setting can add Steiner points later (to create via sites, section 12). If used, vertices not on any obstacle get obstacle id `-1`.
- Fallback if `triangle` is unavailable: `scipy.spatial.Delaunay` over boundary points sampled densely enough that boundary edges survive, then drop triangles inside holes and *verify* every constrained segment exists (insert midpoints where it does not).

### 6.2 Array layout (built once with NumPy, then `.tolist()`)

```
tri_v   [T][3]  vertex ids
tri_n   [T][3]  neighbour across the edge opposite local vertex i (-1 = wall)
tri_e   [T][3]  global edge id of the edge opposite local vertex i
vx, vy  [V]     vertex coordinates
v_obs   [V]     obstacle id of vertex (-1 = free)
edge_v  [E][2]  endpoints, ordered (u, v) with u < v   -> defines edge orientation
edge_t  [E][2]  the two triangles on each side (-1 if none)
edge_len[E], edge_mid[E][2]
edge_cap[E]     capacity (6.3)
edge_kind[E]    0 = interior gate, 1 = wall, 2 = pad-terminal edge (owner pad id in edge_owner[E])
```

### 6.3 Gates and capacity

- A **gate** is an interior edge (two incident triangles). Every wire is represented as the sequence of gates it crosses. Pad-terminal edges are the first and last gates of a wire (one side is the pad hole).
- **Width estimate:** `w(e) = min(|e|, dist(u, polygon(obs(v))), dist(v, polygon(obs(u))))`. The distance terms avoid overestimating the width of slanted edges between parallel obstacles. This is an *estimate*; final DRC is authoritative. Use vectorised `shapely.distance`.
- **Capacity** (how many centrelines fit): `cap(e) = floor(w(e) / (t + s)) + 1`.
- For pad-terminal edges the same formula applies (conservative: same-net wires need not keep spacing).

---

## 7. Topological state (the key data structure)

All state lives in a `TopoState` object (one per layer in the multi-layer version).

```
gate_order  : list[list[int]]   per edge, wire ids ordered from endpoint u to endpoint v
corner_cnt  : list[[int,int,int]] per triangle, count of wires cutting off local corner k
wire_path   : dict[wire_id -> list of (edge_id, tri_id, corner_k)]   steps, for rip-up
usage[e]    = len(gate_order[e])
```

### 7.1 Why this works (read this before coding)

Take triangle `t` with local vertices 0,1,2. A wire that passes through `t` enters through one edge and leaves through another. It therefore **cuts off exactly one corner** (the vertex shared by its two edges). Call that the wire's *corner* in `t`.

Along edge `a` of `t`, with endpoints `V` and `W`, the wires through `t` appear in this order, walking from `V` to `W`: first all wires cutting corner `V` (nested: the closest to `V` is the innermost), then all wires cutting corner `W`. The wires cutting corner `V` occupy the same nesting order on the other edge at `V`.

If this holds in every triangle, the whole set of wires is a **planar embedding** (no crossings). Capacity (6.3) then says whether it is also geometrically feasible.

### 7.2 Invariant (checked by a test helper `check_invariants`)

For every triangle `t` and corner `k` with adjacent edges `a`, `b`: let `c = corner_cnt[t][k]`. The first `c` wires of `a` measured from corner `k`, and the first `c` wires of `b` measured from corner `k`, are the *same wires in the same order*. All of them are exactly the wires cutting corner `k`.

### 7.3 Insert and remove

- **Insert** wire `w` along a path of steps `(gate g_i, triangle t_i, corner k_i)` at slot positions `p_i` (section 8). For each gate `g_i`, insert `w` into `gate_order[g_i]` at index `p_i`. For each triangle step, `corner_cnt[t_i][k_i] += 1`. Positions are computed from pre-insertion state, so insert all gates from the same snapshot. (A path must never cross the same gate twice. Reject such paths.)
- **Remove** wire `w`: delete it from each `gate_order[g]` and decrement each `corner_cnt`. Other wires keep their relative order, so the invariant stays true. This is what makes rip-up cheap.

---

## 8. Slot-aware topological search

### 8.1 State

A node is `(half_edge, p)`: the search has just crossed gate `e` into triangle `t` (the half-edge fixes which side), at absolute slot position `p` in `gate_order[e]` (`p` in `0..usage[e]`, meaning `p` existing wires lie on the `u` side of the new wire). Encode as an integer `half_edge * 16 + p` (cap slots at 15 per gate; a gate with more wires is treated as full).

### 8.2 Transition (all constant-time table lookups)

From node `(h, p)` in triangle `t`, for each of the two other edges `b` of `t`:
1. Let `C` be the vertex shared by the entry edge `a` and `b`. This is the corner this wire will cut in `t`.
2. `r` = number of existing wires between corner `C` and the new wire on `a`:
   `r = p` if `C` is the `u` end of `a`, else `r = usage[a] - p`.
3. **Feasibility:** `r <= corner_cnt[t][corner_index(C)]`. If not, the new wire would have to cross wires that cut corner `C`. Skip.
4. The slot on `b`, counted from `C`, is also `r`. Absolute position `p_b = r` if `C` is the `u` end of `b`, else `usage[b] - r`.
5. New node: `(half_edge crossing b away from t, p_b)`. Cost per section 9.

**Why this is exact:** the new wire cuts corner `C`, so it nests among the existing corner-`C` wires at rank `r` on both edges. The rank is conserved inside a triangle and recomputed at each new triangle from the position on the shared gate.

### 8.3 Start and goal

- **Start (multi-source):** every pad-terminal edge owned by a pad of the net, at every slot `p` in `0..usage`. Cost 0. (The first step uses the same transition rule with the triangle on the free side.)
- **Goal:** crossing a pad-terminal edge owned by the target pad. Any slot is valid, because the feasibility check already bounds it.

### 8.4 A*

- Heuristic: Euclidean distance from the current edge midpoint to the nearest target-pad point (the target pad's centroid distance minus its radius; weight 1.0). That is admissible only while every length costs in full. Beside copper of its own net a wire costs a twentieth (11), so for a net that has copper on the layer the estimate is: of the straight line `d`, only the way to the nearest of that copper, `f`, has to be new, `RIDE * d + (1 - RIDE) * min(d, f)` (`kernel.estimate`). `f` is read from a coarse map (64 cells along the board's longer side, a cell's diagonal taken off so that it never says too much), made by one distance transform per net and kept while neither the wires nor the map change (`search._own`).
- Priority queue: `heapq` with `(f, counter, node)`. Costs are floats; `counter` breaks ties.
- Reuse `dict`s per search; clear instead of reallocating.
- Return the gate/slot sequence, or `None`.
- **A cheapest path that crosses a gate twice is no route, and nothing else is looked for.** Such a path runs along one side of a wire, round its end and back along the other side, crossing again every gate the wire crosses. It cannot be inserted (7.3). Until M17 the search then forbade the node of the repeat crossing and ran again, up to 12 times. Measured on ALU: 80 % of all nodes the search touched were in those repeats (92 of 114 million), 95 searches used all 12 and still found nothing, and what the others found were longer ways round the same wire (a first path of 140 gates and 85 mm became, six runs later, one of 433 gates and 197 mm). Without the repeats, over perturbed runs: ALU 409 (408 to 409) of 409 in 18 s (24 s before), ulx3s 201 (196 to 202) of 203 in 24 s (41 s), blinkSP1 49 (46 to 53) of 58 (46, 40 to 52). The connection is left to a via or to rip-up of the wire in the way, which is what the path was saying. The `banned` table and the retry loop are gone.
- What such a search reached on the way is still reported (8.6), so the connection may go through a via from there. Leaving that out as well was measured, since it is what the first trial did by accident: the same completion within the spread (ALU 409 in all six runs, ulx3s 201 (200 to 203), blinkSP1 50 (41 to 53)), fewer vias (ALU 0 instead of 19, ulx3s 34 instead of 43, blinkSP1 34 instead of 45) and 4 to 5 % more copper on ALU and ulx3s. It was not kept, because it is a special case (a search that finds no path at all does report its reach), but it says something about the open question of 12.8: there the choice between a via and moving the wire in the way went to the priced, relaxed search instead of to "any legal route first", and nothing was lost.

### 8.5 Relaxed search (used for rip-up decisions)

When no feasible path exists, run the same search with the feasibility check relaxed: if `r` exceeds `corner_cnt`, clamp `r` to the limit and add `cross_penalty * (r - limit)` to the cost, recording the wires of `a` lying in the violated range as the *blocking set*. The cheapest relaxed path tells you which wires to rip up (Phase 3, section 10).

### 8.6 Seeds, reach and bound (for routes across layers, 12.3)

There is one search (`topo/kernel.py` `astar`, read by `topo/search.py` `route`). It always runs on one layer. Three additions let a chain of them route across layers:
- **Starts at a cost.** Besides the connection's pad at cost 0, it may start from other pads of its net, each at the cost at which a search on another layer came to it (11): a through-hole pad or a via the net already has is a change of layer that costs nothing.
- **Seeds.** Besides a pad's edges, it may start from points: (triangle, cost so far, x, y). A seed stands in the *middle cell* of its triangle, the part no wire has cut off, which is where a via site would be put. Its start nodes cross each edge of the triangle outwards at the slot between the wires cutting the two corners.
- **Reach.** On the way it records, per triangle, the cheapest cost at which the middle cell was reached, and how; and, per pad of its net, the cheapest cost at which it came to that pad. Because the search is A* with an admissible heuristic, every triangle through which a via could lead to a cheaper route has been reached by the time the goal is. (With riding this was not so until the estimate above knew of it.)
- **Bound.** It gives up on anything that cannot cost less than a bound the caller already has.

Without numba the same function runs as plain Python (there is no second implementation).

---

## 9. Cost model

All costs are in millimetre-equivalents so they add up. Defaults are starting guesses to be tuned (section 20).

| Term | Where used | Definition |
|---|---|---|
| Length | every step | midpoint-to-midpoint distance between entry and exit gate in the triangle, times the layer's `layer_cost` (1 unless set: a layer that also carries a plane, made dear so traces take it only where they must; `PlanarMap.price_lengths`). The price only: lengths measured (reports, detours, the best round) are the lengths |
| Present congestion | every gate | `pres_fac * max(0, usage[e] + 1 - cap[e])` |
| History | every gate | `hist[e]`, grows each rip-up round where `e` overflowed |
| Demand | Phase 1/2 candidate scoring | `w_d * max(0, (usage[e] + demand[e]) - cap[e])` (9.2) |
| Airwire crossing | candidate scoring | `lambda_x` per crossing (9.4) |
| Barrier closure | candidate scoring | `lambda_sever` per severed net (9.5) |
| Via | search across layers (12.3) | `via_cost` per layer change, plus the overflow the via's keep-off causes on the gates around it |

### 9.1 Why costs are split between "in-search" and "candidate scoring"

In-search costs (length, congestion, history) are cheap per step and live inside the pathfinder. The lookahead costs (demand, airwire crossing, barrier closure) need whole-path information, so they are applied when *scoring complete candidates* (top K per net), not inside the inner loop.

### 9.2 Demand map

Route every net once on an *empty* map (plain shortest path, congestion off). For each gate, `demand[e]` is the number of nets whose route uses it. During real routing, decrement a net's contribution when that net is committed. **[HYPOTHESIS]** Using `usage + demand` to price gates reserves scarce channels for the nets that need them, which is the most direct way to "think about the whole board" at low cost.

### 9.3 Candidate generation (K diverse topologies per net)

- Route net, then add a penalty to every gate on the result and search again. Repeat until K distinct gate sequences are found or the cost exceeds `(1 + alpha) *` the shortest.
- Distinct means a different gate-sequence hash.
- Defaults `K = 6`, `alpha = 0.5`.

### 9.4 Airwire crossing: "keep other pads on the same side"

- Each unrouted connection has an **airwire path**: the sequence of gates crossed by the straight segment between its two pads (found once by walking triangles along the segment, then cached).
- **`cross_count(pathA, pathB)`** (no geometry needed): two wires can only cross inside a triangle, and a triangle has only three edges, so crossing wires always share a gate. For each *maximal shared run of gates* between the two paths:
  - At each end of the run, the two paths diverge into different edges. Whichever path exits through the edge adjacent to a given endpoint `C` of the last shared gate is nearer to `C`. This fixes the order of the pair along the gate (as "A nearer the `u` end" or "B nearer the `u` end").
  - At a run end where both paths terminate on the same pad-terminal edge, the order is free (undetermined).
  - The run counts as **one crossing** if both ends are determined and the orders are opposite.
- Airwire cost of a candidate = `lambda_x * sum(cross_count(candidate, airwire))` over unrouted airwires. Find the airwires to test using an inverted index `gate_id -> airwire ids`.
- This is exactly "choose, among the 100 possible routes, the one that keeps the most other pads on the same side": it minimises how many airwires the route separates.

### 9.5 Barrier closure (hard severance)

- Maintain a union-find over **obstacle groups** (nodes: obstacle ids, plus pad groups).
- Edges ("welds") are: (a) each committed wire welds its two end obstacles (all pads of one net's tree weld into one group); (b) each gate that becomes full (`usage == cap`) welds its two endpoint obstacles.
- **Closure test:** welding two obstacles already in the same group closes a barrier loop (an enclosed region). Pure union-find, near-constant time.
- When a loop closes, extract the loop (path in the barrier forest plus the closing weld) and count **severed nets**: unrouted airwires whose endpoints are both off the loop and whose crossings with the loop elements (wires via `cross_count`, saturated gates as single-gate paths) have odd parity. Cost = `lambda_sever * severed_count`. Closure is rare, so this extraction is cheap.
- Union-find cannot delete. Use it only in append-only phases (Phase 1/2). Rebuild from scratch once at the start of each Phase 3 round, then route that round's ripped nets append-only.
- Caveat: on multi-layer boards a barrier on one layer can be bypassed with a via, so scale `lambda_sever` down by layer availability. **[HYPOTHESIS]** Closure counting corresponds to real routability loss. Validate with experiment E3.

---

## 10. Global strategy (the whole pipeline)

### Phase 0: setup
Parse, preprocess, triangulate, build tables, airwire paths, demand map.

### Phase 1: global candidate selection (all nets at once)
1. Generate K candidates per net (9.3), in parallel.
2. Score each candidate alone: length + demand-aware congestion + airwire crossings against *airwires*.
3. Build a sparse **conflict table** between candidates of different nets: `cross_count(candA, candB)` for every pair that shares a gate (inverted index gate -> candidates), plus overflow where summed usage exceeds capacity.
4. **Select one candidate per net** minimising: sum of single-candidate scores + `lambda_conf * sum of pairwise conflicts`. Use iterated conditional modes: visit nets in random order, pick the candidate with minimal marginal cost given the others, repeat until stable, with a few random restarts. Optionally simulated annealing if ICM stalls.
This is the step that makes decisions with the whole board in view.

### Phase 2: commit in regret order
1. For each net, `regret = cost(second best candidate) - cost(best candidate)`. Process highest regret first (the net that loses most if it does not get its best route).
2. For each net, first try to insert its selected candidate exactly (replaying its gate sequence through the section 8 feasibility rules). On failure, run the normal search restricted to a corridor around the candidate. On failure again, run the unrestricted search.
3. After each commit, update `usage`, `corner_cnt`, union-find, demand map, and mark affected nets' regrets dirty (via a `gate -> nets whose candidates use it` index). Recompute lazily.
4. Nets that cannot be routed go to Phase 3.

### Phase 3: negotiated rip-up and reroute
Loop until zero overflow and zero unrouted, or the iteration limit:
1. Increase `pres_fac` (multiply by 1.5 per round, start 0.5). Add `hist[e] += overflow(e)`.
2. Choose the set to rip up: all wires crossing an over-capacity gate, plus the blocking sets from relaxed searches (8.5) for unrouted nets.
3. Rebuild the barrier union-find from the remaining wires.
4. Reroute ripped nets in regret order with the full cost model (candidates re-generated if cheap).
5. Track the best solution (fewest violations, then shortest) and keep it.

### Phase 4 (optional, M8+): topology refinement
For each net, try alternative homotopy classes and accept a change if the **realised** length (section 13) of the whole board improves. Optionally simulated annealing over such moves.

### Where vias enter
Nowhere in particular. Wherever a phase routes a connection it may get a route that changes layer (12.3): the fallback in Phase 2, every reroute in Phase 3 (also the relaxed search that names the wires to rip), the legal placement after it. A via is a cost in the search like length or congestion. Rip-up removes a connection whole, with its vias.

Not yet: Phase 1 candidates are single-layer (on an empty map a via never pays), and Phase 4 leaves connections through vias as they are.

---

## 11. Multi-pin nets

v1: decompose each net into 2-pin connections by a minimum spanning tree over pad positions, then route the connections as independent 2-pin items (each a separate wire id). This loses Steiner-tree optimality, so it is a known limitation. Branches start at pads only. Since realisation, wires of one net owe each other no spacing: where two of them leave a pad the same way they are drawn as one shared trace until they part, which gives the look and the copper of a branching trace without changing the topology (they are still two wires in the state, so capacity is counted conservatively). Later improvement: restart a net's routing as a tree grown from its existing copper, with start states on any gate slot adjacent to an existing branch. That requires splitting the wire at the branch point, so it is deferred.

**Riding (built).** A connection is still a whole wire from pad to pad, but beside a wire of its own net it is the same trace, and is treated as one. The state knows each wire's net and counts a run of same-net neighbours on a gate as one wire (its widest). The search is told the net it is routing; a place directly beside a wire of that net costs no room, and from one such place to the next 5 % of the length (`kernel.RIDE`). So a connection that reaches its net's copper follows it to its pad for next to nothing, and only the branch is new copper. Nothing depends on anything: a rider is a complete wire whatever happens to the wire it rode on.

Measured, one variant, all results clean:

| Board | Without | With riding |
|---|---|---|
| ALU (3 runs each) | 409 of 409; 13,390 to 13,585 mm of copper; 7 to 12 vias; 79 to 104 s | 409 of 409; 11,905 to 11,947 mm; 7 vias; 64 to 69 s |
| RAM Selector Tree | 382 of 400, 255 vias, 12 min | 393 of 400, 258 vias, 24 min |
| blinkSP1 (18 runs) | 48 (47 to 51) of 58 | 46 (42 to 54) |
| Word of RAM | 85 of 85 | 85 of 85 |

Little of ALU's saving is copper that coincides (100 to 200 mm): routes come out shorter. RAM Selector Tree takes twice as long because rip-up goes on finding better states (its last improvement comes at 19 minutes instead of 8) and only then stalls. On blinkSP1 rip-up never settles with or without riding (20 to 30 wires ripped up every round to the end), so the result is the best state it happened to pass; riding changes which, not whether. Three things suspected of riding were checked and are not so: a trunk ripped up leaving its rider to overflow (a removal cannot raise a load; a foreign wire going between trunk and rider happens 140 times in 71,000 crossings and never put a gate over), searches being slower (same number and cost), and bad geometry on RAM Selector Tree (a gap in the string kernel, 13.1, since closed).

Not built: places counted between runs rather than wires (a gate holds 15 wires in the search, riders included); merging coincident segments in the output.

**Its net's copper (built).** Three things were seen, all of one kind. Two traces of a net ran side by side, or took separate ways, where one trace would plainly have done. Two connections of a net never used the same via. And a trace went round a pad of its own net that stood in its way. On the five boards they are rare, so each was first made to happen on a small board of its own (`tests/test_own_copper.py`).

*Why.* A connection could touch the copper of its net at its own two pads and nowhere else, and two places that judge a route did not know that copper beside its net is free.

1. *The search's estimate.* Riding makes a length beside the net's own wire cost a twentieth, but the estimate of what is left (8.4) took the straight line in full. A way that first leads away from the goal, to the net's trace, and then along it, looked dearer than it is by nearly the whole ride, so the goal was reached by the direct way first and the search stopped. Riding was only found where the trace lay on the direct way. (On the test board: 72 mm of new trace round a wall and back, where about 40 mm to the trace on the far side and the rest along it would do.)
2. *Refinement.* Phase 4 moved a wire only if its route came out shorter, measured through the middles of its gates. A route that joins its net's trace is usually longer and adds less copper. Phase 1 plans on the empty board, where there is nothing to ride on, and Phase 2 puts those plans in as they are; so whatever the first two phases laid side by side stayed side by side. Counted on ALU, for each single-layer wire of a multi-pin net, by taking it out and searching exactly: 104 of 261 had a route adding less copper than the one in place (148 mm in all), 9 of them missed by the estimate alone.
3. *Pads and vias.* A pad edge was a wall to every wire but the one that ends there (the same-net rule of section 5), and a via's edges belong to its pad id. A wire could therefore neither pass a pad of its net nor come to a via of its net, and the layer change a net already had was of no use to its other connections (12.7, "still open").

*What is done.*
- **The estimate knows the net's wires** (8.4): of the straight line, only the way to the nearest wire of the net has to be new.
- **One measure, copper** (`Context.wire_copper`): a wire's length through the middles of its gates, a stretch beside a wire of its net counted as the search counts it. Refinement and the tidying of vias (13.2a) compare this, and the best state rip-up keeps is the one with least of it (a shared trunk counted once). Refinement and tidying are now one routine (`ripup._replan`): lift the connection, plan it again, keep the plan if it adds less, else put it back exactly.
- **A pad of the net is a junction.** With `through`, the search may cross a pad edge of its net, and goes on from every edge of that pad (`kernel.astar`, `THROUGH`). What it returns is then in pieces (`Route.lead`), each an ordinary wire from pad to pad, as a route through vias is in pieces. `Path.joints` holds what lies between consecutive pieces: a point, where a via is to be made, or the id of a pad or via that is there. Phase 1 candidates stay in one piece.
- **A pad or via the net has on two layers is a change of layer.** A search reports the pads of its net it came to; the next search of the chain (12.3) starts from those on its own layer, at the cost so far and no price for a via. So a connection goes down the via another connection of its net made, or through a through-hole pad of its net.
- **A via stays while any trace is on it.** `Connection.sites` are the vias a connection has pieces on, made for it or not; `Context.rip` deletes one only when no trace of another connection ends on it.
- **Against a via of its own net a trace takes no room.** A wire that came to a via from outside used to be put through its nearest spoke by hand (`sites.leave`, `arrive`), and its weight added back to that spoke's capacity. Now any wire of the net may come to the via by any spoke, so the rule is general: the run of wires of the via's net at the via's end of a gate is not counted in the gate's load (`TopoState.tally`), and that place is one beside the net's copper for the search. The add-back is gone.
- Pieces of one plan that use a gate in common (a route that leaves a pad beside the piece it came by) are found again in those gates once the earlier pieces are in, since their places there were not known (`Context.commit`). Lifting a connection records each piece's place among the pieces before it, not among all of them: put back in order with the old rule, two pieces on one gate could change places (found by the invariant check on Word of RAM).

*What a via is worth here.* While the board is negotiated a new via costs more than any detour (9), so a connection will go a long way round to a via its net has. Tidying puts that right in one direction only, as before: a connection may be planned with as many new vias as the vias it goes through now, its own or borrowed, each new one at `via_worth`. So a borrowed via is given up for one of its own when the way round by it is longer than a via is worth, and a connection that goes through no via is never given one. Letting tidying add a via wherever it saved more copper than its worth was tried: ALU went from 18 vias to 134 and Word of RAM from 0 to 14, for a fifth less copper. Not kept: that is the opposite of 13.2a. Allowing as many new vias as the connection changes layer (through-hole pads included) gave Word of RAM 2 vias it does not need. Not kept either.

*Measured*, one variant, against the program before; every result clean, and the written files of the first four measure clean. Copper is what is drawn, a shared trunk once (`stats["copper"]`; `stats["length"]` adds up every wire and so counts it twice).

| Board | Routed | Vias | Copper | Time |
|---|---|---|---|---|
| Word of RAM | 85 of 85, as before | 0, as before | 2002 to 1972 mm | 1.7 to 1.8 s |
| ALU | 409 of 409, as before | 18 to 8 | 11,556 to 10,862 mm | 14.6 to 15.3 s |
| ulx3s (6 perturbed runs, all alike) | 201 of 203, as before | 38 to 31 | 1457 to 1539 mm | 10.4 to 8.4 s |
| blinkSP1 (18 perturbed runs) | 48 (47 to 51) to 55 (47 to 58) of 58 | 31 (29 to 38) to 35 (21 to 43) | 973 to 1129 mm | 5.4 to 4.9 s |
| RAM Selector Tree (one run) | 394 to 400 of 400 | 175 to 119 | 25,836 to 22,952 mm | 207 to 160 s |

- blinkSP1 is the board the open item of 12.7 was about (a net changes layer once and branches). With the layer change at the net's own copper switched off and everything else as it is: 52 (47 to 56) routed, 40 vias. With raced variants from the command line it routed 58 of 58 in both runs made, with 38 and with 50 vias (ulx3s, raced: 203 of 203 with 36); the routing saved in its file makes 51.
- RAM Selector Tree, three nets of 73, 35 and 19 through-hole pads, is where a net's own copper is most of the board: complete for the first time on this code, with a third fewer vias. Two earlier versions of this work gave 398 and 399 of 400 with 140 and 139 vias, so the gain is not one draw.
- The times are with what relaxation keeps between runs (13.1), measured in turn with the program before (medians of three for the first three boards). Without that ALU took 19.2 s: there is more to do, more searches (a route may go on at a pad) and more wires to draw, since a connection through a pad is two.
- ulx3s is not steady: 203 of 203 came out in two of the intermediate versions and 201 in the last, with between 1539 and 1761 mm of copper. Nothing here was tuned to it.

*Limits.*
- **Two wires on one stretch each count it as shared.** Where two connections of a net ride each other and neither has a wire of its own underneath, each one's copper there is next to nothing, so neither is moved alone though moving both would save the stretch (seen on a board made for it: three back pads fed from one via beside the first of them).
- **Where a shared via stands** is where the first connection put it.
- **The estimate knows wires, not pads**, so it can be too high by a pad's width for a route through a pad. Marking the pads as well would make it exact and weaker wherever the net has a pad, which for a net of seventy pads is everywhere (not measured apart: RAM Selector Tree took 259 s with the pads marked and 211 s without, with other changes in between). Charging the way across a pad in full keeps it exact too, but put 5 % more copper on ALU (11,311 against 10,782 mm), where pads are large.
- **A connection through a pad that is joined already is drawn once (built 2026-10).** The piece from A to P beside the wire from A to P is that wire's copper; in the state it is still its own wire (nothing depends on anything, as with riding, at the price of more wires: a gate holds 15). Until 2026-10 it was in the output too: a second trace from A to P, on top of the first or beside it, seen in KiCad as a connection routed from A to B and again from B to A. A whole connection can be redundant the same way, when connections through pads have joined its two pads already (on Micromouse2026PCB one ran through four pads of its net, every piece on top of a wire there). Now the output of each net is a tree (`router._redundant`): its wires by drawn length, shortest first, each kept only if it joins pads or vias not yet joined, and then wires to a via nothing else uses taken out with the via. The state, and the routing kept to go on from (23a), are untouched: it is a choice of what to draw. Measured against the program before, one variant, small perturbations of the margin (median and range): routing the same everywhere the board was read the same.

  | Board | Redundant wires before | Drawn trace before / now | Routed |
  |---|---|---|---|
  | blinkSP1 (18) | 22.5 (15 to 35) | 1391 / 1199 mm | 55.5 (51 to 58) of 58, both |
  | ALU (3) | 53 (40 to 56) | 11,608 / 10,833 mm | 409 of 409, both |
  | Word of RAM (6) | 8.5 (5 to 10) | 2014 / 1955 mm | 85 of 85, both |
  | ulx3s (6) | 29 (22 to 36) | 1758 / 1603 mm | 202 (201 to 203) / 203 (201 to 203) of 203; its parts' keepouts are now read (section 4) |
  | Micromouse2026PCB (6) | 4.5 (2 to 12) | 1796 / 1770 mm | 69 of 69, both; read differently (section 4) |
  | RAM Selector Tree (1) | 33 | 23,620 / 22,969 mm | 400 of 400, 119 vias, both |

  The redundant wires were never violations (same-net copper owes no spacing), which is why nothing caught them.
- A connection still starts and ends on pads. It does not end on a trace of its net, and cannot cross one except at a pad.

**Tried on the branch `any-target` (2026-11), not kept: a connection joins the copper either side of it.** The question first: how much could branching save at all? Per net of three pads or more, the Euclidean Steiner tree (iterated 1-Steiner, obstacles ignored) is 0.9 to 1.8 % shorter than the spanning tree the connections are drawn from, on all six boards; the copper drawn is 12 to 46 % above that spanning tree, which is detours, not the shape of the tree. So true branching (traces from the middle of traces) was not built. What was built is the cheap part: an open connection may start from any pad or via its net's wires join to its first pad and end on any joined to its second (`Context.sides`; the search takes a set of goal pads, `own[e] = -1`, and an estimate to the nearest). A connection that relies on copper made by others is broken when that copper is taken out; rip-up and `ripup.mend` route those again, so a finished net is always joined. With the option off the routings are identical to `main` on five boards (hashes).

| Board (runs) | Routed, off / on | Vias | Copper, runs with equal completion |
|---|---|---|---|
| blinkSP1 (18) | 55.5 (51 to 58) / 55.5 (50 to 58) | 40.5 / 38 | not comparable |
| Word of RAM (6) | 85 / 85 | 0 / 0 | -1.2 % (-3.1 to +0.2) |
| Micromouse2026PCB (6) | 69 / 69 | 0.5 / 1 | +0.2 % (-1.8 to +1.7) |
| ulx3s (6) | 203 (201 to 203) / 202 (200 to 203) | 32 / 34 | -1.7 % (-4.4 to +1.0) |
| ALU (3) | 409 / 409 | 10 / 8 | +2.0 % (-2.9 to +3.2) |
| RAM Selector Tree (1) | 400 / 394 | 119 / 175 | +20 % (27,457 against 22,895 mm) |

Within noise on the small boards and clearly worse on the board of large nets, where it was meant to help. Not diagnosed; the likely cause is that every connection of a net of 73 pads now depends on the wires around it, so a rip-up round breaks and reroutes far more than it takes out. Found on the way and kept on `main`: a state snapshot did not carry its wires' nets, so a routing restored in another process (a raced variant's parent, Improve) lost riding and the one-trace counting of same-net neighbours. The code stays on the branch `any-target` (option `any_target`, off by default), shelved.

---

### 11.2 Pads joined to their plane (`plan.fanout`)

A net with a plane is not routed as traces (section 4): the CAD tool's pour joins its pads. A pad the pour does not reach on a layer the pad has copper on stays open: a ground pad on top of a board whose ground is an inner plane, or poured only on the back. Sweep 3 left 1,933 such connections open over 28 boards (hackrf-one 411, marzipan 394, coldfire 167, the Cornes 140 each), every one a via for the designer to add by hand.

Once the board is routed, each such pad gets a via beside it: the nearest place, along 24 rays out to 2 mm from the pad, where a via may stand among everything already there (pads, keepouts, the routed traces, teardrops and vias of other nets, on every layer; the hole-to-hole minimum from through-hole pads and fixed vias, even of its own net; the board edge, and KiCad's Margin drawings) and lands inside a plane of its net on a layer the pad has no copper on, joined by a straight trace that keeps its clearances too. A via of the net within 1 mm, one placed for another pad or a fixed one that lands in the plane, is shared. A plane of its net on the pad's own layer that stops short of it is reached by a short trace instead, with no via (a via there would join nothing: lumen-mobo's local pours, 13 vias KiCad called dangling). A pad that touches copper of its net the plane reaches, other pads (an exposed pad's own holes) or fixed copper (a locked trace to a locked via), needs nothing. Until 2026-11 fixed copper did not count: on a board with locked plane vias, each joined to its pad by a locked trace, 27 pads got a second via beside the locked one, drilled into it (KiCad's hole-to-hole, 0.000 mm). Coverage is the pour's outline, as the DSN gives it; KiCad's fill keeps clear of other copper, so now and then a via or trace lands where the fill does not reach.

Done after routing, it takes no room a trace needed. Placed first, the vias were in the way: on hackrf-lna915 four signal connections that routed before were left open. Measured: lna915 30 open items in KiCad to 11 (the rest the Margin layer and its four open signals), olimex POE-M2 77 to 66, lumen-mobo 217 to 104, with every signal routed as before.

### 11.3 Pours that stay whole (built 2026-11, branch `pour-skeleton`; on by default since branch `pours-default`)

**The fault.** A pour is left to KiCad to fill round the traces (section 4), and nothing in the routing keeps it in one piece. Traces of other nets that run across it, or close a ring round part of it, cut it into islands with no pad, or into sections with pads that are no longer joined to the rest of the net. Fan-out (11.2) only joins pads the pour does not reach at all; it does not see a pour the routing has cut.

**The idea: the pour is one more wire in the gates.** For each pour (a net and a layer), a *skeleton*: a tree joining the pads of the net that lie inside the pour's outline, routed on that layer only, as a wire as wide as the pour's minimum width (KiCad's zone minimum width) and keeping the pour's clearance (the zone clearance, often larger than the trace clearance). It takes room in the gates like any wire of that width, by its weight in the load (7, `TopoState.weight`), as a net class's wider traces already do. Wires never cross (7.1), so no trace can cut the skeleton; and as it keeps the pour's clearance from everything, KiCad's fill covers it, so the pour round it is one piece reaching every pad it joins. The skeleton is not written to the SES (or, as an option, written as traces, which holds whatever the fill's own rules do).

**What is harder than that:**
- *Inside the outline.* The skeleton is held to the triangles inside the pour's outline: the search can already be held to a region (`Context.commit(within=...)`, from 13.3a).
- *Giving way.* On a dense board the skeleton takes room traces need: a gate that took five traces takes four and the pour. It is priced in negotiation like any wire, not placed first (it would win every contest) or last (there would be no room). Where it loses, its pad is left to fan-out (11.2, a via down to the net's plane on another layer) and said so.
- *Thermal reliefs.* KiCad joins a pad to its pour by spokes; the skeleton's end at a pad must leave room for them, at the least where one spoke lands.
- *Islands with no pad.* A piece of pour the skeleton does not pass can still be cut off. It joins nothing, so it is harmless (KiCad removes it or not, by the zone's island setting), and the fill estimate (below) reports it.
- *Several pours of a net on one layer, and zone priorities.* One skeleton per pour; a pour inside another of higher priority takes the outline KiCad fills.

**Measured first.** A fill estimate, the pour's outline less every other net's copper and its clearance, gives the islands and cut-off sections of a routing; shown in the app while routing (`ROADMAP.md`, 1.1), and counted over the sweep boards before and after skeletons, with completion and copper beside them.

**As built (`plan/skeleton.py`, option `skeleton`, `--skeleton`).**
- Per pour on a routed layer, a spanning tree over the pads of its net inside the outline that a trace reaches there (pads whose copper meets are joined already). Each edge is a `Connection` with `skeleton` set and `layers` its pour's layer, in the same Phase 1 to 4 as the traces.
- Width: 1.4 times a fill's minimum width (`fanout.FILL`; never below the narrowest trace), set as the net's width in a copy of the rules the routing runs on. So its weight in the load is that width plus the clearance, realisation keeps traces a clearance from it, and the check sees it. The drawn board keeps the real rules (fan-out's stubs).
- Held inside the outline: every search for a wire of a skeleton's net uses the corridor of the kernel, the gates whose middles are in its net's pours on that layer and the edges of its net's pads; none on another layer (`skeleton.region`, kept up to date as via sites add and change edges). It never takes a via (`path.find`).
- Negotiated like any wire. What tells routings apart puts traces first: rip-up's best state is fewest open traces and over-full gates, then fewest skeletons open, then copper; the kept routing likewise; a design-rule violation drops the skeleton first. And at settling (`ripup.give_way`) an open trace whose way is barred by skeletons has those taken out and goes in, the skeletons put back where they fit; failing that, every skeleton out, kept only if a trace goes in.
- Not drawn, not written, not counted as open or as copper (`router._drawn`, `Context.open`).
- Where it loses: with the option on, a pad the fill estimate puts in a piece cut off from its pour is treated by fan-out (11.2) as one the pour does not reach: a via down to the net's plane on another layer.
- Not built then: thermal relief room at pad ends; the zone's own clearance and minimum width (the board's clearance and KiCad's default 0.25 mm are used); zone priorities. (See below for what was done since.)

**Measured** against the base it was branched from, same DSNs. Pour counts from the fill estimate (islands, cut-off pads); KiCad's from its DRC after refill (`isolated_copper`, unconnected).

| Sweep | Routed | KiCad unconnected | KiCad isolated copper | Estimate: islands | Estimate: cut-off pads | Routing time |
|---|---|---|---|---|---|---|
| Tier 1 (Mac, 12 boards) | as before on every board | 134 to 121 | 36 to 34 | 239 to 231 | 73 to 31 | 150 to 79 s (the base run was slow; 49 s in the older baseline) |
| Tier 2 (Windows, 13 boards) | fewer on three: kicad-cm5-minima 156 to 136, kicad-stickhub 97 to 89, kicad-openair 228 to 227 | 512 to 412 | 3 to 5 | 1393 to 1206 | 379 to 57 | 971 to 2120 s |

Also on tier 2: 4 annular-width errors on kicad-tinytapeout (vias), and one more open item on lumen-pi-interface. The pours stay whole as meant, and far fewer items are left open in KiCad, but on the dense multi-layer boards the skeletons take room traces needed (half of cm5-minima's skeleton joins gave way, and still 20 connections more were lost), and the routing takes twice as long. So it is off by default. Before an earlier version made skeletons give way at settling, tier 1 lost a connection each on fomu and lumen-blade13.

**Made the default (2026-11, branch `pours-default`).** What changed from the version above, and why.

- *Traces first, the skeletons after.* Negotiated with the traces, a skeleton won contests a trace needed, however it was priced: priced the same, cm5-minima lost 20 connections; with the skeletons yielding in rip-up (on an over-full gate they go first, an open skeleton rips up only skeletons, traces rerouted first) still 1 to 3 per dense board, and the time four to seven times. Now the skeletons are not in Phases 1 to 3 at all (no candidates, no demand, no airwires, `Context.later`): the traces are routed and negotiated exactly as without them, and the skeletons go in when the routing is settled (`ripup.settle`), where they fit. Then, for each that does not, the traces in its way are moved (`ripup.make_way`): its way with crossings allowed through gates wide enough for it on an empty board, the connections on it taken out, the skeleton put in and they put back with no more vias than they had; kept only if every trace goes back and no gate is over-full, else undone exactly. So no trace is lost to a skeleton, and a trace that crosses a pour where it could go round is moved round. During design-rule repair the skeletons are taken out and put back by the settling after it; a violation is never a skeleton's (it is not copper: what comes near it only narrows the fill), and a skeleton on a gate of a trace in a violation gives way before repair is tried (`router._crowding`). After refinement, `give_way` once more for what is still open. Skeletons are left out of refinement and via tidying, and their length out of the copper rip-up compares, so that rip-up stops when the traces stop improving.
- *One tree per net.* A pad through the board is in the pours of every layer: the skeleton is now one spanning tree over all the net's pads in its pours, each join through one pour (the shortest joins of each pour's own spanning tree, Kruskal over them), not a tree per pour. Fewer joins, the same pads joined.
- *Width.* What the fill needs, not 1.4 times its minimum: the zone's minimum width and 0.05 mm (`skeleton.SLACK`), and twice what the zone's clearance is above the board's, since traces keep the board's clearance from the skeleton and the fill the zone's from the traces. Never below the narrowest trace.
- *The zone's own settings* are read from the .kicad_pcb (`io.kicad._settings`, `board.Zone`): its clearance, thermal gap and spoke width, how it joins pads (thermal, solid, none, thermal for through-hole only) and its priority; and each pad's own zone connection and relief, or its part's. Used by the estimate, the skeleton's width and region (a zone of another net with higher priority is taken out of it, `skeleton.area`), and fan-out.
- *The estimate (`pours.estimate`)*: each zone's clearance; zones of higher priority filled first, a lower one keeping its clearance from their fill; a pad joined by a thermal relief is kept the gap clear of, and joined only by its spokes (four, along its sides, at 45 degrees for a round pad) whose end, just past the gap, is in the fill; a pad the zone does not join (connection "none") is not counted as cut off.
- *Fan-out* treats a pad the zone does not join as one the pour does not reach, and a pad cut off by the estimate (traces or starved spokes) likewise, with the option on or off; such a pad may be joined by a short trace into the fill of its own layer's pour (the main piece), not only by a via to another layer. Copper drawn on a part is of no net to fan-out (KiCad sees it so: a stub ran into one). Two fan-out vias of a net keep the hole-to-hole distance.
- Found on the way: a pad edge holding one net's run of wires (a skeleton through a pad of its net leaves it twice) was counted over-full; it is not, as one wire there is not.

*Measured* on the Mac against `main` (6e3f9d7), same DSNs, one run each. Cut-off pads are all counted by the new estimate on the boards as KiCad imported them (so the runs are counted alike; the old estimate, with the board's clearance and no spokes, counted fewer); KiCad's open items from its DRC after refill.

| Sweep | Routed | KiCad unconnected | Routing DRC | Cut-off pads (estimate) | Routing time |
|---|---|---|---|---|---|
| Tier 1, 12 boards: main / old skeleton / now | 562 / 562 / 562 | 134 / 119 / 104 | 9 / 9 / 9 | 80 / 40 / 51 | 29 / 45 / 36 s |
| Tier 2, 16 boards: main / now | 2357 / 2358 (openair 229 to 230; none fewer) | 515 / 364 | 276 / 275 | 357 / 125 | 738 / 818 s |
| Tier 2, old skeleton (`--skeleton` on main) | 2346 (cm5-minima 150, stickhub 94, tinytapeout 290) | 382 | 279 | 114 | not comparable (shared Mac) |

The times are of a run with the Mac to itself (the final run of the same counts shared it with another program: 1306 s, not comparable). kicad-cm5-minima, the slowest relative to main, took 106 s against 55; castor-mainboard 88 against 65; the solo boards less than before. Worse than main on one count each: hackrf-operacake leaves 8 more items open in KiCad (33 against 25; fan-out finds no room for a via beside 25 pads of its two inner planes, against 16: with the skeletons in, the traces lie differently, and with them off it is 17); sweep-half-swept one more island. Tried for operacake: `make_way` moving no connection through a via (moved, its vias move into the room fan-out needs). It gave operacake 19 open items, fewer than main, but lumen-blade13 8 more, openair one connection fewer and a clearance error, stickhub a dangling via; not kept. Skeleton joins that still give way to traces (of those planned, on the boards where any does): tier 1 38 of 144, tier 2 258 of 1011 (openair 83 of 175, cm5-minima 57 of 179). Against the old, negotiated skeleton, fewer pads stay joined on a few boards (fomu 22 to 28, sweep-half-swept 6 to 10): traces placed first close ways that negotiation kept open, and moving them is tried only where at most 12 connections are in the way and each goes back with no more vias.

Not done: the skeleton does not yet arrive at a pad along one of its spokes (starved thermals are seen by the estimate and joined by fan-out where there is room, not prevented); a trace moved for a skeleton may not take a via more.

**The estimate checked against KiCad's fill (2026-11, branch `pour-kicad`).** On a user's board, routed, imported, refilled and checked in KiCad 10, and against KiCad's `zone_filler.cpp`, the estimate now fills as KiCad does in these ways (ROADMAP, "Pour estimate against KiCad"):
- *Order.* The outline within the board's own copper-edge clearance (`Board.pour_edge`, from the .kicad_pro and custom rules, not the traces' edge clearance) and away from Margin drawings; less the thermal reliefs; less other nets' copper and higher zones with the clearance; that pruned to the least width is what spoke ends are tested against. The kept spokes are added, other nets' clearances taken out again, and the whole pruned: shrunk by half the least width less 0.001 mm, pieces then under the least width across either way dropped, grown back (`pours._prune`). Solder-mask openings are not taken out (`Obstacle.fill`).
- *Spokes* (`pours.spokes`, KiCad's `buildThermalSpokes`): four, from the middle of the pad's box in its own frame (its angle from the .kicad_pcb, `Pad.turn`) to that box grown by the gap, KiCad's error (0.005 mm) and half the least width, at the pad's spoke angle (`Pad.spoke_angle`; KiCad's default 45 degrees for a round pad, else 90); as wide as the relief asks but no wider than the pad, none at all if that is under the least width. A spoke is kept if its end is in the fill, or in a spoke of another pad whose end is in it. (The earlier rule, a spoke ending in another pad's relief joins it, was not KiCad's: it joined pads KiCad leaves cut off, by bent spokes.)
- *Joins.* The net's own pads, and its traces (fan-out's stubs), join what they touch, on the pour's layer and, through the pads they meet, across layers.
- *Islands.* Each zone's island removal is read (`Zone.islands`): islands KiCad removes are not in the estimate, unless every piece the zone fills is one (KiCad leaves such a zone whole); the ones left are what KiCad reports as isolated copper.
- The CLI prints what the estimate expects KiCad to find unconnected (connections not routed, pads in cut-off pieces, pads fan-out could not join), and the sweep compares it and the islands with KiCad's on each board.

## 11a. Where the copper goes, and undoing detours (`ripup.untangle`, branch `detour-repair`)

**Measured first.** Each routed connection's drawn length against two references: the straight line between its pads, and the same connection routed alone on the empty board (same search and geometry). The gap from the straight line to alone is what the parts and keepouts cost; the gap from alone to drawn is what sharing the board costs.

| Board | Alone, over straight | Drawn, over alone (both in % of straight) |
|---|---|---|
| blinkSP1 | +6.5 % | +63 % (224 of its 294 mm in the 18 connections through vias) |
| ALU | +1.9 % | +22 % |
| Word of RAM | +1.7 % | +5.6 % |

And it is concentrated: on ALU half of the 1,670 mm in 15 of 380 connections, each two to three times as long as alone; on blinkSP1 half in 5 of 35. Refinement (Phase 4) moves one connection at a time and keeps any shorter route, so with everything else in place these have none: the short way is crossed by one to three other wires, or runs through gates that are full.

**What is done.** Longest detour first (copper more than 1.25 times its length alone, and two pitches more): the relaxed search with crossings allowed and capacity ignored names the wires in its way, and those in the gates of that way with no room for it; at most twelve. If the short way is not two pitches shorter than the way it has, nothing is tried. Otherwise they are taken out with the connection, it goes in by its short way (hard capacity), they are put back where they fit (`fill`). Kept if nothing is open that was not, no gate is over-full, and the copper of the nets involved (vias at their worth, as in 13.2a) is less by a pitch; else the board is restored from a snapshot. Up to three sweeps, in every settle. On by default since it was made cheap (setting "Undo long detours"; it was off while it cost up to 170 % more time). The copper of a trace depends only on its own net's wires beside it, so only the nets involved are measured: measuring the whole board each try took 7 of ALU's 27 s.

**Measured** against the same program without it (perturbed runs, median and range; copper where completion is equal):

| Board | Routed, off / on | Vias | Copper | Time |
|---|---|---|---|---|
| Word of RAM (6) | 85 / 85 | 0 / 0 | -0.2 % (-1.9 to 0) | 1.55 to 1.65 s |
| Micromouse2026PCB (6) | 69 / 69 | 0.5 / 1 | -3.0 % (-8.6 to -0.4) | 1.1 to 1.25 s |
| ALU (3) | 409 / 409 | 10 / 12 | -1.6 % (-2.5 to -1.2) | 13.2 to 20.4 s |
| ulx3s (6) | 203 / 203 | 32 / 32.5 | -1.7 % (-7.2 to -0.6) | 9.2 to 14.6 s |
| blinkSP1 (18) | 55.5 / 55 (51 to 58 both) | 40.5 / 39 | -0.4 % (-11.7 to +6.8) | 6.4 to 8.1 s |
| RAM Selector Tree (1) | 400 / 400 | 119 / 117 | -0.5 % | 139 to 194 s |

No violation in any run. The time goes on tries that are undone: 20 to 100 per settle, of which 2 to 8 are kept, spread over the whole order (on RAM Selector Tree at tries 35 to 71 of 102), so capping the tries would lose gains. A first version without the pre-check and with the whole board measured kept twice as many moves on ALU and cost 2.7 times the time on RAM Selector Tree (377 s).

**Made cheap (2026-11).** Three changes, measured in turn:
- *An exact undo instead of a snapshot.* The connections of a try are lifted (`Context.lift`), and while it runs every `commit` records how to take itself out (`Context.record`): the same rollback commit uses when a via does not fit, in the reverse of the order things went in. A try that does not pay is undone by `Context.undo`, last first, and the lifted connections are put back, last first. A first attempt that ripped what the try had put in, instead of taking it out in order, was not exact and failed on boards with vias: rolling the log of the via sites back restored places on gates for wires that were gone. Checked with `WEAVEENGINE_CHECK_UNDO`, against a snapshot after every undone try, on nine runs of six boards: the same gate orders, corner counts, wires, vias and pieces (loads to 1e-9; a wire put back keeps its present place on each gate in its steps). The check found a bug outside this work: `TopoState.insert` did not count again a gate where a wire goes in between two wires of one net, which splits their run into two traces; the load was too low by the width of one. Fixed (on `main` too). It changes nothing on five boards and, on RAM Selector Tree without the detour repair, two of three perturbed runs: 119 to 123 vias and 22,894 to 23,698 mm, 119 to 123 and 24,946 to 25,165 mm (the third identical). Kept: the undo relies on loads that agree with the wires, and the under-count let a gate look roomier than it is; the check of the geometry had been catching what it cost.
- *The over-full test counts only what the try made over-full.* Repair after the check can leave gates over capacity for rip-up; with two such gates every try of a settle was refused.
- *Only the moved wires are put back,* each once, shortest first, and the first that does not fit ends the try (`fill` tried every open connection, and again while any was placed); and a try refused once is not made again in the same call (the same detour with the same wires in its way).

| Board | Copper | Time, off / on |
|---|---|---|
| Word of RAM (6) | -0.2 % (-1.9 to +1.0) | 1.45 / 1.5 s |
| Micromouse2026PCB (6) | -3.6 % (-8.6 to -0.4) | 1.05 / 1.15 s |
| ALU (3) | -1.6 % (-2.5 to -1.2) | 12.7 / 14.2 s |
| ulx3s (6) | -1.8 % (-7.2 to -0.6) | 8.7 / 10.4 s |
| blinkSP1 (18) | -0.4 % (-1.9 to +6.8) | 5.95 / 6.5 s |
| RAM Selector Tree (1) | -2.2 % | 152 / 172 s |

Completion as without it on every board (blinkSP1 55 (52 to 58) against 55.5 (51 to 58)), no violation in any run. The extra time is now 3 to 20 %, against 6 to 170 % for the first version.

**Vias left behind, and the order of a settle (2026-11).** A via on RAM Selector Tree (the user's settings, edge clearance 1.5 mm) took a trace to the other layer a millimetre before a through-hole pad it could have reached on its own layer, at a sharp angle. Planning every connection through vias again once more after a finished pass found 29 such vias (175 to 146). The cause was the order of `settle`: vias were planned again (`replan`) before the detour repair, which moves wires and puts them back where they fit, through a via if need be, and before the last `fill`, which places open connections the same way; nothing planned those again. Now the vias are planned last, after the detour repair and its refinement and filling, and again while that plans anything (three times at most), since what is filled after may go through vias again. With the detour repair off nothing else changes in a settle. Measured against `main` before it (perturbed runs; RAM Selector Tree one run each):

| Board | Routed | Vias | Copper |
|---|---|---|---|
| RAM Selector Tree, the user's settings | 394 to 398 of 400 | 184 to 144 | -7.8 % |
| RAM Selector Tree | 400 of 400 | 122 to 102 | -2.9 % |
| blinkSP1 (18) | 55 (52 to 58) to 55 (50 to 58) | 39 to 40 | -3.4 % (-17.3 to +2.4) |
| ALU (3) | 409 | 12 to 12 | -0.8 % (-2.4 to +1.4) |
| ulx3s (6) | 203 (201 to 203) to 202 (201 to 203) | 32.5 to 31.5 | -0.4 % (-1.7 to +5.3) |
| Word of RAM, Micromouse2026PCB | the same | the same | the same |

No violation in any run. Times were not usable (the machine was loaded throughout: one run of unchanged code took 2,008 s against 15 s alone). After the change, planning the vias once more after a pass still finds 4 on RAM Selector Tree with the user's settings (144 to 140), as many as before the detour repair existed.

## 12. Layers and vias

**Layers left out of the routing** (`read_dsn(layers=...)`, `--layers`): a board with its planes inside is routed on its outer layers only. The layers left out (`Board.unrouted`) get no planar map and no traces, but every via goes through them: their keepouts and fixed copper are kept, with the layer indices after the routed layers', and kept clear of by vias (`plan.path.Legal`, `plan.fanout`), and their planes are reached by fan-out vias. A via is written to the session on every copper layer, else KiCad's import takes it for a blind one. Until 2026-11 the layers left out were dropped whole: on a 4-layer board routed on F.Cu and B.Cu, the +3V3 and GND planes inside were gone, so their pads were left with no way to them.

**The difficulty:** a via is a new obstacle inside a triangle, which changes the planar map.

### 12.1 The first approach: vias between passes, and why it is not enough

Each layer has its own triangulation and `TopoState`. A pass routes on a fixed set of pads. Connections still open at the end of a pass are each split at one via (`plan/vias.py`), the via is added to the board as a pad, every map is rebuilt, and Phases 1 to 4 run again from nothing, up to `max_via_rounds` times.

That works when a handful of vias are needed (ALU: 0 to 18). It fails on `boards/RAM Selector Tree.dsn` (2 layers, 670 through-hole pads, 400 connections, three nets of 73, 35 and 19 pins), where the crossings cannot be removed without many vias. Measured with one variant on one worker, cut off after 400 s:

| Step of pass 1 | Time |
|---|---|
| Candidates, selection, commit | 23 s |
| Rip-up | 259 s |
| Refinement, geometry, repair | 6 s |
| Placing vias | over 110 s, not finished |

78 of the 400 connections were open after pass 1. The causes:
1. **Rip-up negotiates for something it cannot get.** It spends 259 s trading wires between connections that no planar arrangement on two layers can all satisfy. Only a via helps them, and vias are not on offer until the pass ends.
2. **Every via round starts again.** Maps are rebuilt and all 400 connections are routed again, including the 322 that were fine. With four rounds and up to eight raced variants this is where the hours go.
3. **One via per connection per round.** A connection that needs two takes two rounds.
4. **Proposing a via is slow.** Each proposal shrinks the whole free space of every layer again.
5. **A via never moves.** Its position is a guess made before the traces around it exist, and the traces are then routed around the guess.

### 12.2 Plan: via sites created inside the map, during the pass

A **site** is a point that exists at the same place on every layer. On each layer it is a tiny triangular hole in the map (radius 2 µm, inside the routing margin of section 4), whose three edges are terminal edges owned by the site. A site is either:
- **dormant:** owned by no net, keep-off zero. It takes no room; wires pass it on either side.
- **active:** a via of net N. Its three vertices carry a keep-off radius `rho = via_diameter / 2 + s + t / 2`. Wires of net N may start and end on its edges.

**Creating a site in triangle `T`** (done separately on each layer, in the triangle that contains the point). `T` is replaced by the six triangles between `T` and the small hole. Each corner of `T` gets two new gates ("spokes") to the hole. The site is declared to lie in the middle cell of `T`: the region left over after every wire through `T` has cut off its corner. Then:
- Every wire that cuts corner `k` of `T` crosses the two spokes at corner `k`, in its existing nesting order, and nothing else changes.
- Corner counts at the three outer corners are copied; all others are zero.
- No wire's relation to any other wire or obstacle changes, so the invariant of 7.2 holds by construction and no search is needed.

This is option 3 of the earlier draft of this section. It was called intricate there; restricting the site to the middle cell is what makes it simple.

**Then the edges round the site are flipped until they are Delaunay** (`legalise`, about six flips per site). This was planned for later and turned out to be required. The triangle a site lands in is usually long and thin, so its first spokes run to far corners and the via's keep-off disc sticks out through the triangle's sides, where no gate knows about it: in the first trial, traces in the neighbouring triangles ran straight through the via. After the flips the site has a spoke to every vertex that is really its neighbour, and each spoke is a gate with a capacity and a wire order. A flip turns the diagonal of a quadrilateral; every wire keeps the four outer edges it used, and the order on the new diagonal follows from the corner counts of the two triangles, so again no search is needed. A flip is refused if the quadrilateral is not convex or a wire would cross the new gate twice.

**Where a site may go.** A spoke of width `w` has capacity `(w - rho) / (t + s) + 1`, the rule of 6.3 with the keep-off of the via at either end taken off the width. The point is legal when no spoke is over capacity on any layer (`sites.fits`) and the via copper keeps its clearance from fixed copper on every layer. As everywhere, DRC after realisation is the final judge.

A site must also really lie in the middle cell it is declared to be in: if five wires cut a corner and the point is two pitches from that corner, the spokes there are over capacity even while the site is dormant. Choosing the point is the caller's job (M14). On the ALU and RAM Selector Tree maps, with about a hundred wires routed, sites put blindly at the incentre of random triangles left 5 and 1 gates over capacity out of 300 sites each.

**The map is mutable (M13, `topo/sites.py`).** A site appends 3 vertices, 9 edges and 5 triangles (the split triangle keeps its id); a flip reuses the edge and triangle ids it turns. The map's arrays grow by appending, the list mirrors and the per-half-edge transition rows are updated for the edges touched, `TopoState.grow` extends the state, and the compiled search's tables are built with spare room and updated in place. Every change to the map is logged with what is needed to undo it (`sites.log`, `rewind`, `replay`), and what a new site changes in the state is journalled (`sites.journal`, `undo`), so a site that does not fit is taken out without a trace and a snapshot restores the map it was taken on. Undoing works in reverse order; taking out a site that is no longer wanted, in any order, is `sites.delete` (12.5, step 2). Things cached per gate that pass through a changed triangle (airwire paths, the candidate corridors of Phase 2) are not updated; they only feed cost estimates, and the code tolerates them being stale.

**Removing a via.** The site is put to sleep and then deleted from the map (12.5, step 2). A sleeping site that could not be deleted keeps half a pitch clear around its hole, because wires pass it on both sides and must not meet at it. The last pull of the traces tries the deletion again (13.1).

**The 2 µm hole realises as a correct via: go (M13).** This was the hypothesis the spike was for. What it took in `realize/`:
- the keep-off is what a foreign wire keeps from the via at the least, whatever lies in between (13.1);
- for a wire that goes round it, a via is one disc at its centre, not the three corners of its hole. Three discs of half a millimetre whose centres are 3 µm apart have tangents between them that point anywhere (13.1, "what does not carry over");
- the via's own trace: which of the three hole edges it leaves through is the search's accident, so its heading is judged a trace width away from the hole, a trace heading away from its edge goes round the nearer way, and it may take the innermost place inside foreign traces that wrap the via (`terminals.hop`). All three apply to via sites only; ordinary pads behave as before.

Results (details in the M13 row of section 18): 10,000 random operations with the invariant intact; hand-placed vias clean at 55 of 55 random positions, the smallest gap from a foreign trace to the via copper 0.307 mm against a rule of 0.300.

Known limits:
- A point within 20 µm of a triangle's edge cannot hold a site (5 of 60 random positions). The caller nudges the point.
- A via's own trace counted as load on the spokes it crossed, although it needs no keep-off from its own via. Since section 11 ("Its net's copper") the wires of a via's net that lie against it are not counted there.
- Capacity lowered by DRC feedback (`_penalise`) is overwritten when a site at that gate changes state or the gate is flipped.
- `free_space` does not know about vias; only the end-straightening shortcut in relaxation reads it, and DRC checks its result.

### 12.3 Routes across layers (M14c, built)

**The route** (`plan/path.py` `find`). A chain of searches, one per layer the route runs on. The first starts at the connection's pad, on each layer the pad is on. Each later one starts from seeds (8.6): every point the search before reached on another layer where a via may legally be and has room among the wires, at its cost so far plus `via_cost`. The cheapest arrival over all chains of up to `max_vias` vias wins; more vias are tried even after a way is found, because the way with fewest vias is often a long way round. The pieces are read back from the searches' own records, so the plan is exact: there is no second search to turn it into wires.

**Where a via may be.** Inside every layer's free space shrunk by the via's copper and clearance (worked out once, with a coarse grid for testing many points); clear of other vias; and really in the middle cell of its triangle on both layers, far enough from each corner for the keep-off and the wires cutting that corner. Candidate points are five per triangle reached.

**Committing it** (`Context.commit`). For each via in order: its site is made on every layer (12.2) without settling its edges, so that the rest of the plan is still good; the piece ending on it is extended across the site's nearest spoke to the hole and inserted. Then the sites' edges are settled (`legalise`), which carries all wires. If a via turns out not to fit (a gate beside it would be over-full, a later piece passes a triangle an earlier via went into, two pieces on one layer share a gate), everything is taken out again in reverse order and the route is planned once more without that point (`path.place`). A trace may over-fill a gate for rip-up to sort out; a via may not.

**The connection** then consists of pieces, each an ordinary connection from pad or via to via or pad, with the original as parent (`Connection.pieces`, `.sites`). `Context.rip` on any of them removes them all and deletes the vias from the maps (12.5 step 2), except a via that a trace of another connection ends on.

**Copper the net already has** (section 11, "Its net's copper"). A piece may also end on a pad or via of its net that is there already, and the next start from it, on the same layer or on another. Such a joint costs nothing and nothing is made for it.

**Racing variants** hand their sites back through the map log (12.2); the parent replays it.

**What a profile of RAM Selector Tree shows** (162 s, one variant). 69 s are inside the search, nearly all of it in rip-up and for nets of two pads: a search that finds no way on its layer floods the layer (about 25,000 nodes), and a route that changes layer is a chain of such searches. The flips of edges when a via is made or taken out are 12 % (81,873 of them); geometry is 10 %; refinement and the tidying of vias 15 %, most of it the making and unmaking of vias, not searching.

- *A plan refused because its via did not fit is made again from the same searches from the pad* (`Path.first`): a refused plan leaves no trace, so they stand. Exact: the routings are the same to the last digit with and without it on blinkSP1 (189 plans made again), ulx3s and ALU.
- **Tried and taken out: dropping a via point where an earlier search on that layer had been for no more.** The thought was that whatever leads on from there the earlier search had found for less. That is true of costs and not of routes: the cheapest path of a search may cross a gate twice and so be no route (8.4), and then the dearer way through the via is the only one there is. Measured by planning everything both ways on blinkSP1: 65 of 1725 plans were lost or came out dearer, and a run that gave 56 of 58 gave 45. It had looked like a gain (the searches of `find` in rip-up on RAM Selector Tree from 34.7 s to 21.0 s) because part of what it saved was plans that should have been found. RAM Selector Tree went from 400 of 400 to 399 with it and is back at 400 without.
- The same holds, far more weakly, for a rule that was already there: a search from a pad of the net on another layer is not made if a search has been at that pad on that layer for less (section 11). Measured the same way it changes 3 plans in 6,400 over blinkSP1, ulx3s and ALU. It stays, for now, because without it ALU takes 15 % longer; it is not exact.
- **Tried and taken out: taking the next cheapest way when the cheapest cannot be read back.** `find` keeps only its cheapest arrival at the goal and returns nothing if an earlier piece of that way crosses a gate twice, although dearer arrivals were found. Taking those in turn is plainly more correct, and ulx3s went from 201 to 203 of 203 and blinkSP1 from 55 (47 to 58) to 56 (47 to 58). RAM Selector Tree went from 400 of 400 with 119 vias to 390 with 173, in three times the time: the plans it now finds are ways through more vias, and on that board more vias leave less room. One run each; but a change that can cost ten connections on one board is not one to keep on the strength of two on another. What it points at is the root: a search whose cheapest path can be no route. Everything that reasons from "cheapest" (the bound, the reach, these prunings) is only as sound as that.
- Tried and not kept: *not planning a connection again in refinement when nothing its last searches looked at has changed* (the gates they touched and those beside them, by the stamps of 13.1). It is exact: the routings came out identical on four boards. But one connection in four moves in a refinement pass, and each move changes gates that many others' searches looked at: 655 of 3252 attempts were skipped on ALU (15.2 s to 14.6 s), 474 of 1540 on ulx3s and 15 of 191 on blinkSP1 with no gain in time. Forty lines for 4 % on one board.
- **The rule this leaves for speed work:** a change that is meant only to save time has to give the same routing to the last digit, on every board, with and without it. The three kept here do (what relaxation keeps, 13.1; the searches from the pad used again; the map's changes stamped per edge).

### 12.4 What was measured

Measured with one variant on one worker. All results have zero design-rule violations (violators are dropped at the end). The table and findings 1, 2, 4 and 5 are from the prototype that 12.3 replaced; "After M14c" below is the present code.

| Board | No vias | Vias during the pass | Vias between passes (old) |
|---|---|---|---|
| RAM Selector Tree (400 connections) | 322 | 383, 206 vias, about 5 min | did not finish in hours |
| blinkSP1 (59 connections) | 26 | 37, 33 vias, 7 to 23 s | 32, 92 vias, 135 s |

Reference for blinkSP1: the routing saved in the file itself (Freerouting) makes at least 51 of the 59 connections with 33 vias at a length ratio of 1.23. We spend the same 33 vias to gain 11 connections.

Findings, from `blinkSP1` unless stated:

1. **Vias are decided last, one connection at a time, after the single-layer topology is fixed.** 56 of the 59 connections can use one layer only (surface-mount pads on the top). Phases 1 to 3 therefore try to route the whole board on one layer: after rip-up the top layer holds 27 to 36 wires and the bottom layer 1. Each leftover connection then has to hop over walls the others built, which is where three vias per connection come from. This is the forward-only routing section 2 argues against.
2. **Rip-up cannot settle because the problem it is given has no solution.** Over-full gates swing between 21 and 166 from round to round, all of them between two pads. Of the 8 wires over-filling a gate when it stops, 6 have a legal route only by crossing other wires. Over-filling is how the search expresses a crossing it is not allowed to make; no price on capacity removes it. (RAM Selector Tree: 300 to 1,500 over-full gates every round, with or without the prototype.)
3. **The geometry failures on this board are not about capacity.** All 16 clearance violations without vias (22 of 30 with) are the same thing: the straight stub from a pad's centre to where the trace leaves the pad's keep-off ring passes too close to the neighbouring pad of a fine-pitch part (0.185 mm where 0.235 is needed, pads 0.2 mm apart). Section 13.1 step 4 assumes "the inflation ring guarantees no foreign copper is in between"; that is false once the rings of neighbouring pads merge. Most of these violations sit at gates with half a pitch or more to spare.
4. **RAM Selector Tree is different**: there the failures are spacing between traces (25 of 311 without vias, 92 of 540 with), and widening the spacing of the offenders makes it much worse (38 violations on a layer become 131, then 207), because the gates have nothing to spare. That board has not been diagnosed further, by decision: work continues on `blinkSP1` only until it routes.
5. **About 60 % of via attempts are built and undone**, because whether a via fits among the wires is only known after its edges are flipped.

**After M14c** (`blinkSP1`, one variant, 9 s): 52 of 58 connections, 63 vias, no violations, the invariant intact after every commit and rip-up. The top layer carries about 70 wires and the bottom about 30, where before M14c the bottom carried 1.
- **Completion** is past the target (51) and past both earlier attempts.
- **Vias are far too many**: 63, against 33 in the file's own routing. Changing `via_cost` from 2 to 40 mm moves the result between 44 and 48 connections and 33 and 44 vias with no trend: the outcome is governed by how rip-up happens to go, not by the price.
- **Rip-up still does not settle**: 23 to 36 rounds, over-full gates swinging between 0 and 65, then the over-fillers are removed. So finding 2 was only part of the story; vias inside rip-up made a solution possible but did not make rip-up find it steadily.
- Part of the via count is structural: 30 connections cannot be routed on the top layer as the net decomposition stands, and each then needs two vias. The file's routing connects more on the top layer; it is not tied to a fixed set of pad pairs per net (section 11).
- `Word of RAM`, which needs no via, now takes 6 (85 of 85, length ratio 1.08 against 1.12 without): a via is used wherever it is the cheaper way, which is not the same as wherever it is needed.

### 12.5 Plan for the rework

In this order. Each step is to be measured on `blinkSP1` before the next.

1. **Fix the pad-exit stub (finding 3). Done (M14a, 13.1 step 4).** It is independent of vias and affects every fine-pitch part. A trace may leave a pad only where the stub from the pad centre keeps its clearance: that gives each pad edge a legal window (possibly empty), computed once per map. An edge with an empty window is not a way out of the pad at all, so the search never uses it; relaxation keeps trace ends inside the window.
2. **Delete a via site from the map, anywhere, at any time. Done (M14b, `sites.delete`).** The triangles round the hole form a polygon. It is filled with triangles again, without the hole, by cutting ears off it; the new inner edges are then flipped until they are Delaunay, as the map's edges were before the site came. Every wire that crossed the polygon keeps the two edges of it that it came in and went out by, and crosses whatever new edges lie between them; their order on a new edge is their order along the polygon's boundary. With the hole gone, which way round it a wire went no longer matters. No search is needed, and the invariant of 7.2 holds by construction.
   - **Slots.** A site takes 3 vertices, 9 edges and 5 triangles of the map's tables. Deleting one frees exactly that many whatever the polygon's size, so they are kept as a slot and the next site takes it. No id of anything else ever changes, and the tables stop growing once as many slots exist as sites were ever alive at once. An unused slot's rows are parked far from the board, joined to nothing.
   - **Logged like everything else**: a deletion rewinds and replays with creations and flips, so snapshots and raced variants are unaffected.
   - A wire that passes the polygon twice, once on each side of the hole (a U-turn round something of another net beside the site), must not cross one new edge twice. Until 2026-11 the polygon was filled with the first ears found, and such a deletion was refused (10 of 400 in a trial with wires; 0 of 400 without), leaving the site asleep and the U-turn round it. Now, where it is asked to (`both_sides`), an ear is cut only where its new edge is crossed by no wire twice (and by fewer than 15): every deletion the first ears refused in the random-operations test goes through, the invariant holding. The fill is logged, so a replay makes the same one. It is still refused, changing nothing, if no ear can be cut so. Only the last pull of the traces asks (13.1), on a map it puts back afterwards. While routing the first ears are used and such a site stays asleep, as before: a site taken out where it used to stay leaves another map, the routing goes another way from there on, and it was no better for it (the sweep of tiers 1 to 3, 2026-11: solo-a routed 132 to 127, solo-c 121 to 120, lumen-mobo 332 to 331, olimex-poe-c 201 to 206, upduino 122 to 123; KiCad open items 880 to 900 in all). A sleeping site is not used again, so nothing is gained by having it out sooner; what it costs a trace is taken off at the end.
   - A first version, which flipped the site's spokes away until six were left (the reverse of creation), got stuck on 1 site in 40 for geometric reasons and was dropped.
3. **One search. Done (M14c, 12.3 and 8.6).** The multi-layer search is the router's only search; `complete`, the displacement rule and the between-passes code are removed.
4. **Open, in the order I would take them:**
   - why rip-up does not settle now that a solution exists (12.4, after M14c);
   - what a via should cost, and whether that cost belongs in Phase 1's global selection so that layers are assigned with the whole board in view;
   - nets as trees grown from their own copper instead of fixed pad pairs (section 11), which decides how many connections need a via at all;
   - capacity with a margin, from what is measured then.


### 12.6 Which layer is a question about the whole board (experiment, to be built as M14d)

**The thought.** A board's topology has two levels, and the router has been solving both with the tools of the second.
- *Level 1, between connections:* which of them would cross, and so which must be on different layers. This is a property of the whole set at once, a colouring of a graph of crossings. Nothing about it is local.
- *Level 2, within a layer:* which way each wire passes each obstacle and each other wire. This is what the triangulation, the gate orders and the search are for, and they do it well.

Today level 1 is never posed. A crossing is not something the state can hold (7.2), so the search meets other wires only as walls. Which connection yields is decided by the order they happen to be routed in, one at a time, and rip-up revisits those decisions one at a time. Its currency is over-full gates, which is not what the conflict is about (12.4, finding 2). That is why it does not settle, and why the price of a via has no steady effect: the price is paid inside a process that is not choosing.

Phase 1 already has the right shape for level 1: several ways per connection on the empty board, a table of which ways cross (`cross_count`), and one choice per connection made for all of them together. It works for through-hole boards, where a connection has ways on every layer (ALU: 236 and 173 wires on its two layers, no vias). It has nothing to choose on a surface-mount board, because every way it is given is on the one layer the pads are on.

**The experiment** (`blinkSP1`, 2 seconds, nothing committed to the maps). For every connection: its single-layer ways as Phase 1 makes them (4 on average), plus one way "underneath" per other layer, found by the same search with that layer's gates favoured: down a via near one end, along the other layer, up a via near the other end. Then one way chosen per connection to minimise crossings, with a price per via.

| | Crossing pairs | Vias |
|---|---|---|
| Every connection by its shortest single-layer way | 238 | 0 |
| Best choice among single-layer ways only | 152 | 3 |
| Best choice with the ways underneath as well | 5 | 53 |

With the ways underneath, 31 connections go underneath, 9 connections are still in a crossing, and the total length is 787 mm (the file's own routing: 831 mm, 33 vias). Raising the price of a via from 20 to 60 against 100 per crossing gives 9 crossings and 45 vias: here the price does act, because it is paid where the choice is made.

So 233 of the 238 crossings can be taken out before any wire is laid, by a choice that sees all connections. What is left for rip-up is a handful.

The via count (53) is still above the file's routing (33). That routing puts more connections on the top layer at the cost of longer traces. The single-layer ways tried here are all within 50 % of the shortest (`alpha`), so the choice could not make that trade. A via beside a pad shared by all connections of its net was also tried as an explanation and is not one: it would save about 5 vias.

**Built and measured; it does not hold up as it stands.** The plan was: Phase 1 offers ways underneath, the selection chooses, the rest carries the choice out. Three ways of carrying it out were tried on `blinkSP1` (one variant). The code was taken out again; what was learned is below.

| How the choice was carried out | Routed | Vias |
|---|---|---|
| No layer choice; vias found during commit and rip-up (12.3, the code as it is) | 52 of 58 | 63 |
| Layers chosen in the selection; each chosen way planned again when committed | 42 to 45 | about 40 |
| Layers chosen in the selection; all chosen vias made first, then the pieces routed as ordinary connections | 30 | 64 |

1. **The selection does choose** (24 connections on one layer, 34 through vias, 7 to 25 crossings left depending on the detours allowed), but **what follows does not keep to it**. Of 24 ways chosen on one layer, 4 were committed on the gates chosen. A way through vias cannot be replayed at all: it was found on the map before any via existed, and every via changes the map under the others.
2. **Making the chosen vias first chokes the board.** 58 vias leave more than 300 gates over-full after the commit phase, most of them beside a via, and rip-up never brings that down. Each way underneath goes down at the nearest legal point to its pad, so the vias crowd round the fine-pitch part and take the room the other pins need to get out.
3. **What the file's own routing actually does** (measured from the file, correcting a guess made above): it makes 31 connections with top-layer copper alone, which is what this router manages on the top layer too (28 to 32). It does not route more on top. It makes 20 more connections with 33 vias, 1.65 each: multi-pin nets reach the other layer once and branch there (GND: two more pins for one via), and through-hole pads serve as layer changes.
4. Growing each net as a tree from whichever of its pads are already joined, instead of fixed pad pairs, was tried on the top layer alone, greedily: 22 to 32 connections against 31 for fixed pairs. Not the lever either.

**Where that leaves the question.** The planar part is not what is behind: on one layer the router matches the reference. What is behind is everything about vias, and on this board a via is mostly a matter of *room*: its keep-off is 1.5 mm across beside pins 0.5 mm apart. The search prices a via as a constant and puts it wherever is nearest. Nothing says what it costs the others to have it there, and nothing lets a net reuse a layer change it already has.

**Hypothesis 1 tested before building it, and dropped.** "Vias crowd the fine-pitch part" is true of the made-first variant above, not of the router as it is. Measured on its result (63 vias) against the file's routing (33): median distance from a via to the nearest pad of its own net 3.8 mm against 2.6; to the nearest foreign pad 1.7 against 1.0; vias within 2 mm of the chip's pins 4 against 7. The file's routing packs its vias *tighter* than this router does. Placement is not the problem.

**What is different is how many connections stay on one layer.** Of this router's 52 routed connections 18 are on one layer and 34 go through vias (25 of them through two). The file's routing has 31 on the top layer alone and 20 through vias. With vias switched off this router also puts 28 on the top layer. So with vias on offer from the start, about ten connections that could stay on one layer take vias instead, and that is most of the extra thirty vias.

**The price of a via, swept properly** (one variant; "one layer" = routed connections with no via):

| `via_cost` (mm) | Routed | Vias | One layer |
|---|---|---|---|
| 3.4 | 41 | 44 | 17 |
| 3.5 | 45 | 50 | 19 |
| 3.56 (the default) and 3.6 | 52 | 63 | 18 |
| 3.7 | 45 | 45 | 20 |
| 10, 20, 40, 60, 80, 100 | 45 to 49 | 41 to 46 | 20 to 23 |
| 130 to 170 | 48 | 32 | 27 |
| 200, 300, 600 | 44, 43, 42 | 33, 36, 36 | 24, 21, 20 |
| 2000 | 46 | 46 | 20 |
| vias only after single-layer routing has settled (not a price: rip-up without vias, then legal placement with them) | 42 | 28 | 26 |

- **The default's 52 of 58 was one draw.** A change of 4 % in the price moves the result between 41 and 52 connections. Results quoted from a single run of this router on this board cannot be trusted to within ten connections; from here on a result is quoted as a range over small changes.
- **There is a plateau** from 130 to 170 mm, more than twice the board's diagonal (58 mm), where the result is steady and close to the file's routing: 48 connections, 32 vias, 27 on one layer, in 8 s. A price that high means "a via only where the connection cannot otherwise be made". Outside the plateau it is worse again, in both directions, so this is an observation about this board, not yet a setting.
- **The seed changes nothing** (three seeds, identical results), and neither does racing variants on this board: the variants differ only in seed and search weighting.

The common factor in every one of these is that rip-up does not settle, so whatever is being varied, the outcome is mostly where rip-up happened to stop.

### 12.7 What rip-up does on a board that needs vias (investigation)

Traced round by round on `blinkSP1` (default settings, no DRC repair):

| Before round | Open | Over-full gates | Ripped, then placed again (through vias) | Vias | On one layer |
|---|---|---|---|---|---|
| 0 (after the commit phase) | 18 | 51 | | 30 | 22 |
| 1 | 5 | 38 | 24, 37 (22) | 53 | 24 |
| 3 | 11 | 19 | 16, 16 (7) | 49 | 20 |
| 5 | 12 | 6 | 12, 9 (6) | 52 | 17 |
| 7 | 11 | 0 | 11, 9 (4) | 50 | 19 |
| 9 | 12 | 3 | 6, 6 (2) | 49 | 19 |
| 12 | 11 | 0 | 5, 4 (2) | 53 | 18 |
| 14 | 10 | 2 | 7, 6 (3) | 55 | 18 |

1. **The over-full gates do get cleared**, in about seven rounds. "Rip-up does not settle" was the wrong description: what does not go away is a core of about ten open connections. From round 2 on, each round rips the wires in the way of the open ones (the relaxed search names them), places most of them again, and leaves about as many open as before, often different ones. The best state seen is kept, so this does no harm, but it does no good either, and which state is "best" by a connection or two is where the run-to-run differences come from.
2. **That core may be near what the board allows.** The file's own routing, after thousands of passes of another router, makes 51 of these connections. This router's level is 48 to 50.
3. **Vias are taken from the start, not as a last resort.** Already after the commit phase there are 30 vias and only 22 connections on one layer; with vias switched off the same phases put 28 on one layer. About half of everything rip-up places goes through vias. A connection takes vias as soon as that is cheaper than its single-layer route with congestion priced in, and at the default price that is almost at once.

**As a spread** (18 runs each over small changes that should not matter: congestion growth 1.45 to 1.55, via price within 5 %, two search weightings). A single run of this router means little; from here on results are quoted like this.

| | Routed (of 58) | Vias | On one layer |
|---|---|---|---|
| As it is (`via_cost` 3.6 mm) | 48 (41 to 52) | 54 (39 to 63) | 18 (16 to 25) |
| `via_cost` 133 mm (1.5 x the cap on an over-full gate's price) | 48 (42 to 52) | 37 (28 to 57) | 23 (20 to 27) |
| `via_cost` 150 mm (1.7 x the cap; **the default since**) | 48 (45 to 50) | 37 (32 to 48) | 24 (18 to 27) |
| `via_cost` 200 mm (2.25 x the cap) | 48 (44 to 51) | 44 (32 to 61) | 21 (14 to 27) |
| `via_cost` 89 mm (the cap itself) | 47 (42 to 50) | 44 (22 to 57) | 21 (16 to 29) |
| A via only if no single-layer route exists at all, however over-full | 46 (43 to 48) | 44 (34 to 57) | 21 (17 to 27) |
| The file's own routing | 51 | 33 | 31 |

A price somewhat above the cap keeps the same number of connections, with a third fewer vias and half the spread. The good range is not wide: at the cap itself and at 2.25 times it the vias are back to 44. `via_cost` is now 1.7 times the cap (`CostParams.pres_cap`), on the strength of this one board. Why it works, from the trace: the price of an over-full gate grows each round up to a cap (about 89 mm on this board). With a via dearer than that, a connection stays on one layer while rip-up sorts out the layer, and only changes layer when its single-layer route is still over-full in several places at the highest price. With a cheap via it changes layer in the first round, before rip-up has sorted anything out. The rule "never, while a single-layer route exists" goes too far the other way: connections that have no legal single-layer route then sit on over-full gates for good (the same six ripped and put back every round), until they are removed at the end.

**Next (M14d).**
1. Rip-up: find out why it does not settle with a solution available, measuring over a spread of small changes instead of one run. Everything else is unreadable until this is steady.
2. Then the price of a via as a principle: more than any detour on the board, so that single-layer routes are kept (the plateau above suggests it).
3. Then: a net changes layer once and branches (a connection may start from any via or through-hole pad its net already has), which is where the file's routing gets 1.65 vias per connection instead of 2.

**Stopping the cycling.** Once nothing is over-full, rip-up stops after three rounds without improvement instead of eight. Same spread of results on `blinkSP1` (48, 45 to 50; 37 vias), 15 s a run instead of 18.

**Still open.**
1. A net changes layer once and branches: a connection may start from any via or through-hole pad its net already has. This is where the file's routing gets 1.65 vias per connection through vias; this router pays 2. **Done** (section 11, "Its net's copper").
2. Run-to-run spread is still 45 to 50 connections and 32 to 48 vias.
3. The repair after the design-rule check on `RAM Selector Tree` (below).

**The other boards on this code** (command line; `main` is the code without any of section 12.2 onwards):

| Board | Now | On `main` |
|---|---|---|
| Word of RAM | 85 of 85, 0 vias, 3 s | the same |
| ALU | 409 of 409, 9 vias, 55 s | 409 of 409, 0 vias, 99 s |
| ulx3s (one variant) | 200 of 203, 39 vias, 60 s | 200 of 203, 37 vias, 239 s |
| blinkSP1 | 50 of 58, 41 vias, 68 s | 35 of 58, 47 vias, 58 s; with more passes the vias only grow (26, 50, 63, ... 99 after nine) and the open pieces never fall |
| RAM Selector Tree (one variant) | 360 of 400, 216 vias, 18 min | does not finish in hours |

All measured clean from the written file.
- `ALU` takes 9 vias it does not need.
- `RAM Selector Tree` is worse than the prototype of 12.4 was (383 of 400, 206 vias, about 5 minutes). The main rip-up takes 9 minutes, and then each of the four repair rounds after the design-rule check takes over two minutes more. The geometry failures with vias on this board (12.4, finding 4) have never been diagnosed, and the repair for them is where half the time and probably the connections go.
- Racing variants on a board no variant will finish runs all eight for nothing: the variants differ only in seed and search weighting, and the seed has no effect. `RAM Selector Tree` raced was stopped after 13 minutes with its first heat of four still running.

### 12.7a Where rip-up's time goes, round by round (measured; nothing changed)

Rip-up is 106 of 156 s on RAM Selector Tree (35 rounds), 3.3 of 5.8 s on blinkSP1, 2.7 of 10 s on ulx3s, 2.1 of 16 s on ALU. On RAM Selector Tree: planning a route for each connection taken out 76 s (3,600 plans, 21 ms each), putting plans in 18 s, taking connections out 5.5 s, the searches that name what blocks an open connection 3.6 s, everything else 1 s.

**What the searches inside rip-up are** (58 s inside the kernel on that board):

| Search | Share | Nodes each |
|---|---|---|
| from the pad: the cheapest way crosses a gate twice, so there is no route (8.4) | 33 % (7,650 of 10,500 such searches) | 28,000 |
| from via points, a way found | 32 % | 28,000 |
| from via points, the way crosses a gate twice | 12 % | 35,000 |
| from via points, no way | 6 % | 9,000 |
| everything else (a way found from the pad, candidates, blocking) | 17 % | |

So the floods are not, as was supposed, a matter of the price of an over-full gate dwarfing distances and blunting the estimate. Three searches from the pad in four end in a way that goes along one side of a wire, round its end and back along the other, which cannot be put in. Nothing is offered on that layer, and the connection goes to the searches from via points, which flood again. This is one cause for both the time and the vias made during rip-up (44 rise to 173; 119 are left after tidying).

**Tried: a search that does not cross a gate its own way has crossed.** Before a step onto a gate the search has been at already, the way to the present node is walked back to see whether it crossed that gate (the walk stops at a pad of the net it came through, where a new piece begins). Not exact (a dearer way to the same node that had not crossed the gate is not kept), but every way found can be put in. Rip-up then behaves as negotiation should: on RAM Selector Tree it ends in 20 rounds with nothing open and nothing over-full, where it now runs 35 and ends with 5 open; ALU ends with 1 via instead of 8. But the walk makes a search ten times slower with the same number of nodes (ALU: 0.74 ms to 8.7 ms each): RAM Selector Tree 156 s to 424 s, ALU 15 s to 28 s. Quality was mixed besides: ALU 6 % more copper, RAM Selector Tree 150 vias for 119, blinkSP1 56 (47 to 58) for 55 (47 to 58). Taken out. A test in a few steps instead of a walk (a pointer per node that jumps back, the depth of each node) would by my estimate leave a search about twice as slow, and the run on RAM Selector Tree perhaps a sixth faster: not enough for a change that alters every board's routing.

**Tried: ending rip-up as soon as no gate is over-full**, and leaving what is open to `fill` (once nothing is over-full, a round takes out what blocks the open connections, and those are open next: on RAM Selector Tree the open count goes 20, 13, 10, 17, 8, 12, 13, 10, 2, 5, 9, 6, 6, 5, 7, 2, 5, 3 over the last 17 rounds). Word of RAM, ALU and ulx3s come out the same to the millimetre. RAM Selector Tree: 400 of 400 still, 105 vias for 119, 4 % more copper, 148 s for about 160. blinkSP1: the same median, 55, but the worst of 18 runs falls from 47 to 42. Eight per cent on one board against a worse tail on another: not kept.

**Other things seen and not pursued.** Every wire on an over-full gate is taken out, though the gate may be over by one: 133 to 169 of 400 connections a round for the first eleven rounds. Rounds that repeat exactly (ulx3s rounds 10 to 15: the same four wires out and back) until the stall limit ends them; they cost hundredths of a second each. Each plan in rip-up makes two scored candidates per layer before the route across layers is tried; how often the second or the scoring changes the choice has not been counted.

**Where that leaves it.** The lever is the search: a way from the pad that is a route, and one search over all layers instead of a chain that floods each layer again. Both are the same piece of work, and it changes what every board routes to, so it is to be judged as a change of the router and not as a speed-up.

**Measured again (2026-11), and would a better first guess help? (A question about a learned guess: no.)** One variant, one process. Rip-up is 123 of 207 s on RAM Selector Tree with default settings and 263 of 420 s with the app's settings; the other boards take 1 to 15 s in all. Each round costs 3 to 8 s, 70 % of it planning routes and 20 % putting them in. Two stretches: the first 14 rounds bring the over-full count from about 3,400 to about 100 (58 s, 85 s); the rest (67 s, 168 s) bounce between 10 and 60 violations, and on the app's settings never reach zero (the best, 24 open and 9 over-full, at round 36 of 43).

*Tried: ending the bouncing sooner.* Stall limit 5 or 3 instead of 8, or at most 20 rounds, two margins each, both settings: every cut that saves time leaves 6 to 11 more connections open, or 30 to 60 % more vias, or design-rule violations (app settings, 20 rounds: 389 of 400 and 7 violations, for 398 and none). Settle does not finish what rip-up leaves. Not kept.

*Tried: the perfect guess (an oracle).* Route once, keep the history prices rip-up ended with (keyed by the gate's two corners, which survive vias flipping the map), route again from nothing with those prices in place, either before the candidates or when rip-up starts. If a guess of where the fight is could save time, this one would. Rounds of the main rip-up, recorded / seeded before everything / seeded at rip-up:

| | Default, margin 0 | Default, margin 1 | App, margin 0 | App, margin 1 |
|---|---|---|---|---|
| Rounds | 47 / 24 / 42 | 26 / 31 / 41 | 43 / 44 / 45 | 27 / 38 / 30 |
| Routed (of 400) | 400 / 400 / 400 | 400 / 399 / 398 | 398 / 400 / 398 | 399 / 400 / 391 |
| Vias | 102 / 96 / 128 | 113 / 120 / 139 | 144 / 149 / 176 | 111 / 119 / 172 |

One run in four is shorter; three are longer. Seeded before everything is about as good as without (within the spread margins make); seeded at rip-up is worse. Knowing *where* the fight is does not end it: every connection now avoids the same gates, and the fight moves. What rip-up works out over its rounds is *which* connections give way, and a map of prices does not hold that. So a learned guess of congestion (a network trained on the router's own results, or a hand-made estimate like RUDY) cannot do better than this oracle, and this oracle does nothing. Not pursued. The lever stays the one above: the search.

**Tried again (2026-11, branch `simple-search`): the search keeps to ways that cross each gate once.** First, whether a short look back would do: on RAM Selector Tree 12,579 of 38,461 ways found (33 %) cross a gate twice, and the two crossings are never fewer than 11 steps apart, mostly 36 to 40, 15 % 64 or more (ulx3s: mostly 36). These are long ways along a wire, round its far end and back, not turns round a pad; only the whole walk catches them. So the whole walk, made cheaper than before by walking only for a step that would make a node cheaper (`kernel.crossed`). No way found crosses a gate twice any more. Judged on results first, the same spreads as always (one process; RAM Selector Tree against `main` runs of the same day):

| | `main` | Walk |
|---|---|---|
| RAM Selector Tree, margin 0 / 1 | 400, 102 vias, 22,493 / 400, 113 vias, 23,846 | 400, 123 vias, 23,527 / 400, **231** vias, **28,326** |
| The same, app settings | 398, 144 vias, 23,368 / 399 (1 violation), 111 vias, 22,489 | **400**, 121 vias, 22,081 / **400**, **280** vias, **26,301** |
| ALU (3) | 12 vias, 10,625 | 6 vias, 10,844 |
| blinkSP1 (18) | 55 (50 to 58), 40 vias, 1,057 | 55.5 (**47** to 58), 35 vias, 1,117 |
| ulx3s, Word of RAM, Micromouse (6) | | within the spread |

Every board 1.3 to 3 times slower (RAM Selector Tree 1.5 to 2.5). The app settings route all 400 for the first time, but one run in two on RAM Selector Tree doubles its vias and puts on 17 to 19 % copper, blinkSP1's worst run falls from 50 to 47, ALU has 2 % more copper. Not kept: the spread it opens is worse than what it closes. Kept on the branch.

### 12.8 Why traces end up too close on RAM Selector Tree (diagnosis)

Taken from the routing saved at its first geometry step: 32 of 400 connections open, 215 vias, 583 traces, 123 violations.

- 121 of the 123 are spacing between two neighbouring traces, away from their ends (10 are within 1.5 mm of an end). 15 go away when corners are not rounded into arcs; without arcs at all there are 462, so the arcs are not the cause.
- **They are predicted by how full the fullest gate the two traces share is**, by the capacity count of 6.3:

| That gate is | Pairs of neighbouring traces too close |
|---|---|
| under 70 % full | 7 of 379 (2 %) |
| 70 to 80 % | 10 of 91 (11 %) |
| 80 to 90 % | 37 of 123 (30 %) |
| 90 to 100 % | 66 of 121 (55 %) |

- 100 of the 123 are at gates carrying four wires or more.

So the count of how many wires fit through a gate is right as arithmetic and too tight as geometry, for bundles. Pulled taut, a bundle sits at exactly the spacing it owes, and wires that cross gates at a slant and arcs drawn as polygons cannot keep "exactly". It is not about vias: the same board had 25 violations in 311 traces before there were any. Vias add traces, and so bundles.

**Tried: count a bundle less tightly** (from the fourth wire on, 1.25 pitches each). Not kept. `RAM Selector Tree`: 364 of 400 with 273 vias, against 360 with 216. `blinkSP1`, which has no geometry problem: 45 (38 to 49) routed against 48 (45 to 50). It takes room from every board to spare one, and by the table it would have to be nearer 1.5 to work.

**Kept for a while, then removed with the rewrite below: the geometry uses the room that is there.** On each gate, whatever room is left over once every wire has its window is put between the wires, up to 0.15 of a pitch per pair of different nets, instead of leaving the bundle packed at the minimum. It costs no routing capacity. On the saved state: 123 violations to 82. More than about 0.2 of a pitch makes it worse again. `Word of RAM`, `ALU` and `ulx3s` route as before and their smallest trace-to-trace gaps go up (0.210 to 0.269, 0.213 and 0.230 mm against a rule of 0.2); `blinkSP1` 48 (45 to 52) routed, 42 vias.

**The other two thirds: the way the geometry is built cannot hold a bundle that crosses a gate at a slant.** Measured on the same saved state, after the change above (82 violations left):

- 68 of 79 spacing violations are between two traces in the same triangle, both passing through it. (An earlier guess, traces either side of an edge neither crosses, with nothing to keep them apart, accounts for 1.)
- For 51 of them, the two traces are as far apart *along* the nearest gate they share as they owe, but they cross it at a slant (median 40 degrees off square), so measured square to the traces they are closer than they owe.
- Relaxation knows this and asks for the spacing divided by the sine of the angle along the gate. Of the 270 gates where a pair ends up too close square to the wires, 225 have no room along the gate for what it asks.
- It is not the sweep stopping early (it never reaches its tolerance, with 120, 600 or 3000 sweeps, but the count is 82, 82 and 78), nor the slant below which it stops asking (0.2 to 0.7: 76 to 102), nor the fineness of the arcs (15, 7.5, 3.8 degrees: 82, 89, 85).

Those wires do fit. Wire number k from one end of a gate has to stay k pitches from that end's vertex and the rest of the pitches from the other end's: two discs, which do not overlap whenever the capacity count says the wires fit. A bundle crossing at a slant bends round the one disc and then the other, as concentric arcs, and keeps its spacing. Relaxation as built (13.1) gives every wire one point on each gate and joins the points with straight chords, adding an arc afterwards round at most one vertex per triangle; it has no way to draw that double bend, so it asks for room along the gate instead, more than is there.

So the capacity count of 6.3 is right, and the fix is in realisation: the taut path of a wire against *discs* at the vertices it passes (radius: what lies between it and the vertex), which is the rubber-band construction this design started from. Tangent lines between discs and arcs round them are spaced correctly by construction, for any number of wires at any slant.

**Done (M14e): realisation rebuilt that way** (13.1). The sweep, the spacing along gates and the spare-room spreading above are gone. On the same saved state:

| Realisation | Violations |
|---|---|
| as it was (one point per gate) | 123 |
| with spare room spread between the wires | 82 |
| rubber band, first check | 1 |
| rubber band, after repair (13.2) | 0 |

End to end, one variant, every result measured from the written file and clean:

| Board | Before | Rubber band |
|---|---|---|
| RAM Selector Tree | 360 of 400, 216 vias, 18 min (349, 218 vias, 20 min with the spreading) | 368 of 400, 215 vias, 9 min; **382, 255 vias, 12 min** with the fill step (see below; 393 in 16 min when repair rounds ran) |
| blinkSP1 (18 runs) | 48 (45 to 52) of 58, 14 s | 48 (47 to 51), 9 s |
| ALU | 409 of 409, 5 vias | 409 of 409, 3 vias, 27 s |
| ulx3s | 200 of 203, 39 vias, 50 s | 200 of 203, 37 vias, 36 s |
| Word of RAM | 85 of 85, 0 vias | 85 of 85, 0 vias, 1.5 s |

**Read the RAM Selector Tree row with care.** With the final code its first geometry check finds nothing, so no design-rule repair round runs and the result is the routing as rip-up left it: 368, in half the time. Two runs made earlier the same day, when the geometry still left one or two violations, went through the repair rounds, and each repair round also runs up to five more rounds of rip-up; those runs ended at 393 of 400 with 253 vias. So 25 connections are there to be had by negotiating longer, and the rule that stops rip-up once nothing is over-full and the open count has stalled for three rounds (12.7) is stopping too soon on this board. That is a search question, not a geometry one, and it is open.

**Followed up: it is not the stop rule.** From the saved routing (32 open, rip-up just stopped), with the stall limit at 3, 8 or 20 rounds the outcome is the same. Each further round of negotiation has fewer open (15 to 29) and 80 to 330 gates over-full, never fewer of both, and the loop rightly falls back to where it was. What had routed the extra connections in those earlier runs was the last step of every repair round: placing what is open with every gate held to its capacity. Done again after refinement has shortened the routes, that step alone takes 32 open to 18 in 13 seconds. Negotiation cannot do it, because there an over-full gate is cheaper than a via (so that it settles who shares a layer), and an open connection that could go round by vias keeps asking for the gate.

So that step is now its own (`ripup.fill`: place what fits, shortest first, until a pass places none) and runs after refinement, alternating with it while it places anything. End to end: **382 of 400**, 255 vias, 12 minutes, no repair round, clean. The other four boards are unchanged.

**Tried and not kept: reserving room for the open connections.** Negotiation only weighs wires that are routed; an open connection loads no gate, so nothing makes way for it. The trial: make the full gates on each open connection's cheapest route one wire tighter, negotiate five rounds, settle with the gates still tight, give the room back, settle again; keep the cycle if fewer are open. (Raising those gates' price instead does nothing, and neither does giving the room back before settling: 18 open stays 18.) On the saved routing it went 18, 14, 10, 8, 5, 3, 1 open in 14 minutes. End to end it gave 386 of 400 in 15 minutes against 382 in 12, and on blinkSP1 49 (45 to 52) against 48 (47 to 51) at 18 s against 14. Four connections for a quarter more time on one board and nothing elsewhere is not enough to carry a mechanism, and why the saved routing responds so much better than the live one is not understood. The observation stands and is worth more than the trial: **open connections have no standing in negotiation.** That, and the price of a via against an over-full gate (9), are the two open questions on the search side; routing a connection to a trace of its own net rather than to a pad (11) would take demand off the gates before either arises.

What the geometry itself changed: nothing that fits is ripped up or dropped any more because its trace could not be drawn (360 to 368 with no repair rounds at all, against 18 minutes half spent in them). blinkSP1 does not change, as expected: it had no geometry problem, and its open connections are the search's (12.7).

### 12.9 Not in scope
- Blind and buried vias: a site is on every layer.
- Vias in pads (`via_at_smd`).

---

## 13. Geometry realisation (topology to coordinates)

Input: per wire, the gate sequence and the per-gate ordering `gate_order`.

### 13.1 Relaxation: the rubber band with thickness (`realize/relax.py`, `realize/kernel.py`)

**The construction.** A wire with other wires between it and a vertex has to stay their combined spacing away from that vertex. That is a disc round the vertex, and its radius comes from the topology alone: on any gate ending at the vertex, add up the spacings of the wires nearer to the vertex (wires of one net owe each other nothing). The wire's trace is the shortest line in its route that stays outside the discs of the vertices it passes: straight runs tangent to the discs it touches, and arcs round them. Each wire is pulled on its own; no wire's trace depends on another's.

Two wires pulled against the same two vertices get radii that differ by the spacing they owe, so their straight runs are parallel and their arcs concentric, that spacing apart, whatever the angle at which they cross the gates. Nothing has to be spaced along a gate. (If the wires fit through every gap by the capacity count, the discs either side of each gap do not overlap for any of them.)

**The discs of one wire** are the vertices of the triangles it crosses, in order, each on a known side: when the wire moves on to its next gate, one end of the gate is new. Details:
- a wire is not held away from the corners of its own two pads, and a via's keep-off (12.2) is the least any foreign wire's radius can be there;
- a via is one disc at its centre;
- the two ends of the window on each pad edge (see Terminals) are points the trace has to pass between.

**Pulling taut (`kernel.pull`, compiled).** This is the funnel algorithm with discs for points: the line between two points becomes the tangent between two discs. Two things do not carry over from points, and both were found by the traces they broke.
1. *The two sides of the funnel do not only meet at its apex.* A disc reaches into the other side's string anywhere along it: a via beside a pad makes a slot the string has to thread, touching a disc on its left, then one on its right, then one on its left. So the funnel is kept as what it stands for, two taut strings from the start, one to the last disc met on each side, each free to touch discs of either side. A new disc is reached by its own side's string, less the discs that string lifts off; if a disc of the other string is in the way, it is reached by that string instead; and if the new disc is in the way of the other string, that string goes by it from there on.
2. *Taut round the far side of a disc is taut too.* Between points, a string that turns the wrong way at a vertex has lifted off it. Round a disc the string can turn by more than half a turn (a trace that makes a U-turn round a via), and a turn of 350 degrees looks like a turn of 10 the other way. What tells them apart is the route: the angles of the wire's triangles at the vertex add up to how far its route goes round it. A wire that passes a point straight has half a turn of triangles there; what they sweep beyond that is how far the trace can turn round the point (a quarter turn more is allowed round a disc). A string that would have to turn further has lifted off.

**Only copper holds a trace off.** Two things in the state are not copper on the board, and do not bend a trace (they made traces that were not taut, ROADMAP 1.1):
- *A sleeping via site*, left in the map where a via was taken out and `sites.delete` refused, or not tried again since. A wire passing a site crosses a fan of its spokes, and wires on one side of it share a spoke, nested on it. A wire gets no disc at a sleeping site, as if the site were not there, unless a wire of another net lies nearer the site on a spoke they share and does go round it, or a wire of another net passes it across (no spoke in common): the site keeps wires on its two sides half a pitch from it each (`sites.asleep_keep`). Whether a wire goes round a site depends on the other wires there, so a wire's kept discs also wait on the site's spokes. That last case made a U-turn round nothing on a user's board (0.47 mm from any copper): a wire passed the site on both sides, round a wire of another net between, and the deletion was refused for it (12.5 step 2). Once the routing is done, each layer's traces are pulled once more with every sleeping site that can be deleted out of the map, there also one a wire passes on both sides (`router._taut`, `sites.delete`'s `both_sides`).
- *A wire that is not drawn* because its net joins those pads already (`router._redundant`). In that last pull those wires are out of the state too. The layer keeps the traces if the check finds no more there than before; the state and the map are then put back as they were (`sites.undo`, a snapshot).
- *A via of the wire's own net that it passes without ending on it* (lumen-blade13: corners 2.3 mm from any foreign copper). While routing a wire goes round it, as the routing was planned with it. The last pull is made a second time with such vias loose, as sleeping sites are (`relax`'s `own_vias`), but not a pour net's vias: its traces lie in its own fill. A straightened trace is kept if the check finds nothing on it, and only where the pours keep their room (`router._over_own_vias`): the pour estimate cuts off no pad it did not, and fan-out still finds a place for every via it did. Where either fails, the straightened traces within a via and four pitches of the trouble get their bends back, up to three times, and then all do; so do those the check then finds more on (beside others that keep their bends). Drawing every such trace straight cost the pours room (lumen-blade13 KiCad open 18 to 20, fomu 2 to 14 mask and thermal problems); so guarded, lumen-blade13 has one bend left of seven (4.5 mm of 8.7 mm of slack), its pour as before. A via may be left with traces of one layer only ending on it (what else ended on it was ripped up or is not drawn; on solo-c a trace of its net made a hairpin out to such a via and back, and pulled straight it no longer hid that). It joins nothing a point does not, so it is not drawn and is no copper from then on, unless its net has a pour, which it may join (`router._drawn`).
The pours' skeletons (11.3) are not drawn either, but they hold traces off on purpose: that is the fill's room. `bench/slack.py` finds slack left in a board's traces, by cause (against the trace as drawn, so what the last pull took out does not show) and by bends with no copper within their clearance (the corners of one arc are one bend).

A disc inside another on the same side is never touched and is dropped. Where discs of opposite sides overlap, the gate between them is over-full; the trace passes square to the line of centres and DRC reports it.

**Drawing.** An arc is drawn as a polyline round the outside of its circle, in steps of 6 degrees or less, fine enough that its corners stand at most 0.4 µm out (the check allows a micron).

**Terminals.** The wire's first and last points are the pad centres; it must pass through the window on its pad edge. The ring guarantees only that a point on it is clear of foreign copper. Where the rings of neighbouring pads have merged (fine-pitch parts), a stub from the part of the ring over the gap cuts across towards the neighbour. So each pad edge has a window, the part of it from which the stub keeps its clearance, found once per map by trying 17 points along the edge (`geom/exits.py`). An edge with no such part is made a wall, so the search never leaves a pad through it. A pad with no legal edge at all cannot be reached on that layer and is reported with the other unreachable pads. Within the window, pulling taut chooses the point. An end that presses against a corner of its pad edge is moved to the next edge round the pad (`terminals.straighten`, M8).

**What a wire's own triangles do not tell it.** One thing is added before pulling, because it is common and the repair does not converge on it where pad exits crowd each other (13.2):
- *Where a foreign wire leaves its pad.* A wire passing that pad keeps its distance from the point, a disc there. The wires near such points are pulled a second time once the points are known, and a third time with every such point held where it is.
Anything else of the kind is found by the check and repaired (13.2). (For a short while a second case was built in as well, the next vertex along an obstacle that the bundle inside a wire goes round. The repair of 13.2 does the same work: without the built-in case the saved RAM Selector Tree routing has 5 violations at the first check instead of 1 and none after repair either way, and the five boards route the same. It was removed.)

**Cost.** RAM Selector Tree, 583 traces on two layers: 0.3 s per layer for relaxation, 0.1 s for the check.

**Kept from one run to the next.** A layer is relaxed many times in which little has changed: once per round while ends are hopped round their pads (up to eight), once per repair (up to four), and again after every settling. A wire's discs follow from the wires on its own gates and from the map, and its first pull from its discs; so both are kept per wire while no wire has come or gone on any of its gates (`TopoState.stamp`, the state's count of changes written to each gate a wire comes to or leaves) and the map has not changed at any of them (`PlanarMap.edge_changed`, likewise; `relax._kept`). At first the whole layer's were dropped whenever a via was made, moved or taken out anywhere; now only those of the wires that cross an edge at that via. The results are the same to the last digit. On ALU, of 54,000 computations of a wire's discs 4,700 are left, and the run takes 15.3 s instead of 19.2.

### 13.2 DRC and repair

- Check with `shapely`: buffer every trace by `t/2`, query an `STRtree` of foreign-net copper for distance `< s`; check against the outline; check wire-to-wire distances.
- **Repair: a trace is given the discs it did not know of.** A disc can reach into a wire's path from a vertex that none of the wire's triangles touch: a via in the next triangle, a bundle going round a vertex two triangles away. Where the check finds two traces too close, each takes on the discs the other is pulled against at that spot (those on the far side of the other trace), a spacing wider; a trace too close to a via takes on the via's disc. The new disc goes into the wire's order between the two discs its trace touched either side of the spot. Then the layer is pulled again, up to four times, and the best result is kept. This is the same rule as 13.1 (keep clear of a vertex by what lies between), applied where the triangulation did not show it.
- What is still wrong after that is a routing problem: mark the gates involved, add history cost, and send the affected wires back to Phase 3.
- Tried: letting this repair do all the work of 13.1's additions. For the next vertex along an obstacle it does, and that addition is gone. For pad exits it does not: the fine-pitch test, clean at the first check with the exit discs, is left with violations without them. Those stay.
- Via copper is checked as a pad of its net on every layer. Via to via and via to fixed copper are kept apart when the via is placed, not checked afterwards.
- The stub from a pad centre to the ring used to be assumed clear (12.4, finding 3). Fixed in 13.1 step 4.
- (Until M14e, repair widened the spacing of the wires involved. With traces drawn as in 13.1 that helped in 1 of 14 realisations that had a violation, and it is gone.)

### 13.2a Vias where a trace would do (`ripup.replan`, `router._route_once`)

**What was seen.** A via between two pads that a plain trace could join; a via just before a pad; two vias taking a trace to the other layer and back with nothing crossing in between. Counted on the finished boards by taking each connection through vias out and planning it with fewer: about 5 of 25 on blinkSP1, none on ulx3s, 31 on RAM Selector Tree at the same length or shorter.

**Why.** (1) Nothing looked at the vias again after design-rule repair, which moves things. (2) A via went only if the route came out strictly shorter, and a route through vias measures short, because the way to and from the via inside its triangle was not counted. (3) The search's cheapest plan is not always the one that does best once it is in, so a plain trace lost to a plan through vias that was then refused.

**What is done.**
- *A via has a worth* when a routing is tidied: the track it takes away, which is the width it keeps clear on every layer (`ripup.via_worth`; about 3 mm on blinkSP1). A route is better without a via if that makes it no more than this much longer. This is not the price of a via during negotiation (9), which would trade a via for a detour of many times its size, as was measured when that price was first used here.
- *Each connection through vias is planned again with the fewest vias first*: none (its net may have a change of layer it can use, section 11), then one more, up to as many as it goes through. Routes are compared by the copper they add, not by length, and a via counts at its worth only if no other connection has a trace on it. A plan is measured as it will be once in (to and from each via counted), must be better by a pitch, and is skipped altogether if the connection is already within a pitch of the straight line with the vias its pads force. Repeated while any connection comes out better (three times at most): one that moves leaves room for another.
- *After repair the routing is settled once more.* If the check then objects, it is done again from where it stood, leaving alone the connections whose traces the check names (three times at most, then left as repair had it).
- *No copy of the board per attempt.* The connection is lifted (`Context.lift`: its pieces with their places on every gate, its vias, the marks of the maps' logs) and put back exactly (`Context.put_back`) if nothing better is found. A plan that is put in can be refused from inside `commit` (`within`), which then undoes itself as it does for a via that does not fit. Before this, every attempt took a snapshot of the whole board and most restored it: 48 of 73 s on RAM Selector Tree.
- Found by the test of the exact put-back: deleting a site emptied the journal of what it had done to the wires whenever nobody else had written in it, so that it could never be undone. The journal is now emptied once a round by the planner. Also seen there and not yet traced: the stored capacity of a gate between two vias can be one wire too low (it errs on the safe side).

**Measured** against the program before (medians over perturbed runs; RAM Selector Tree one run):

| Board | Vias | Copper | Time |
|---|---|---|---|
| RAM Selector Tree | 207 to 130 | 24,967 to 23,497 mm | 129 to 123 s |
| blinkSP1 | 44 to 42 | 962 to 973 mm | 5.9 to 7.2 s |
| ulx3s | 42 to 40 | 1557 to 1592 mm | 8.0 to 8.3 s |
| ALU | 19 to 16 | 11,436 to 11,361 mm | 13.1 to 14.2 s |

No violation in any run; the same connections routed; the check from the written file clean on the four boards with vias. Copper rises a little where a via was given up for a slightly longer trace. Left: three connections on RAM Selector Tree that could still lose a via at no cost in length (not traced), and the small boards pay 1 to 2 s for settling again after repair.

Tried on the way and dropped: planning the vias again only in the last settle (RAM Selector Tree came out worse than before, 211 vias, because the check refused the whole tidy and nothing of it was kept); requiring only "shorter" (leaves the vias the complaint was about).

### 13.3 Sliding vias (planned, M15)

**Where bad vias come from (measured before building anything).** On blinkSP1 the two traces at a via meet at under 90 degrees at 40 % of the vias (30 to 45 over perturbed runs) and at under 45 degrees at 14 %; on ulx3s 43 % and 22 %. Causes, each checked:
- *Phase 4 left connections through vias as they were routed.* Planned again alone on the finished board, 18 of 26 (blinkSP1) and 36 of 40 (ulx3s) came out shorter. **Built** (`ripup.replan`): once the others are in their final places, each is taken out and planned again with no more vias than it had, a via priced at one pitch, and the plan kept only if the connection is shorter once it is in; otherwise everything is put back from a snapshot. Result: ulx3s 43 to 35 % under 90 degrees, the length a via costs 1.35 to 0.89 mm, copper 2.5 % less; blinkSP1 40 to 36 %, copper 2 % less; RAM Selector Tree 7.5 % less copper (one run), still 400 of 400; no violation anywhere. Pricing the via at its negotiation price here was tried first and is wrong: it traded vias for detours (ulx3s copper up 6 %).
- *A via beyond the pad it serves* (the trace passes the pad, changes layer and comes back) is mostly forced: of 11 on ulx3s, 5 have no room for a via anywhere on the near side, and most of the rest have a pad that its own layer only lets be reached from the far side.
- *Too few places offered* (five points per triangle) is not the cause: wherever a via would fit nearer, one of the five points offers it. A lattice of points in large triangles was tried: slightly better on blinkSP1, no better on ulx3s, a fifth slower. Not kept.
- *The way from where a triangle is entered to the via is not charged* by the search. Charging it (from the middle of the gate crossed) was tried and made nothing better and ulx3s 14 % longer: in a large triangle the middle of a gate is far from where the trace really crosses it. Not kept. It shows the real limit: the search measures through the middles of gates, so it cannot place a via well inside a large triangle. That is a matter for the geometry, below.
- *A via cannot be straightened in the finished geometry alone.* Tried on the final polylines with everything else held still: 8 of 52 moved, because the traces beside a via are taut against it (25 of 52 stopped by a neighbouring trace at once).

**Trial of sliding in the map** (a script, not in the program: coordinates of the site's three vertices moved on every layer, tables of the edges and triangles at the site recomputed, relaxation run again, the check run, vias next to anything it finds put back). Each via steps towards the straight line between the corners either side of it, as far as its triangles stay the right way up, no gate at it is over-full, and it stays legal and clear of other vias. blinkSP1, four rounds: 45 of 58 vias moved in the first (0.41 mm on average), copper 975 to 939 mm, median angle 113 to 128 degrees, vias under 135 degrees 40 to 32, **no violation left** (one per round appeared and went when one via was put back). The sharpest are not helped: they are short stubs from a pad, whose best place is the pad itself, and they would have to travel round it further than their triangles allow (M16). Not then built: the move as an operation of `sites` with its log entry (the raced variants pass their sites to the parent by log), and the force of the traces that go round the via.

**Built (M15, on the branch `sliding-vias`).**
- `sites.move(pmap, state, site, point)`: the hole's three vertices go to the new place together; the lengths, middles, widths and capacities of the edges at the hole and the middles of the triangles round it are recomputed; no wire's place on any gate changes. Refused, with nothing changed, if a triangle at the site would turn over or keep less than 0.3 of its area (a move is a step), or a gate at the site could no longer hold its wires. It writes a log entry (the old coordinates and widths, so that undoing is exact, not a move back), and `rewind`, `replay` and `undo` know it: snapshots and the hand-back from raced variants work as for any other change. Tested among the 10,000 random operations (moves of sites awake and asleep with wires passing: the invariant, the map's tables and every gate order after each), for exact undo and replay on a second copy of the map, and for a moved via realising clean with foreign traces round it.
- `Context.move_via(pad, point)`: the same on every layer, after the tests a new via gets (legal on every layer, spacing to other vias); all layers or none.
- `plan/slide.py`, last in a pass: up to three rounds. Each via steps towards the nearest point of the line between the corners either side of it (the whole way, or a half, a quarter, an eighth: the furthest that is allowed). Then every trace is pulled taut again and the check run. A violation is traced to the vias its traces go past (their own two and those crossing a gate at the via); those go back and stay where they were for good. A round that leaves the check worse or the copper no shorter is undone. An option (`slide`, "Straighten vias" in the settings).
- Not built: the force of the traces that go round a via (13.3 step 1, second part); travelling further than the triangles at the site allow (M16), which is what the sharpest vias need.

**Measured**, over perturbed runs, against the same program without sliding (and, in brackets, before any of this section):

| Board | Vias under 90 degrees | Under 45 | Length a via costs | Copper | Time |
|---|---|---|---|---|---|
| blinkSP1 | 36 to 32 % (40) | 15 to 11 % (14) | 1.52 to 0.94 mm (1.48) | 994 to 962 mm | 5.8 to 5.9 s |
| ulx3s | 35 to 35 % (43) | 19 to 16 % (22) | 0.89 to 0.64 mm (1.35) | 1569 to 1557 mm | 8.3 to 8.0 s |
| RAM Selector Tree (one run) | | | | 25,063 to 24,967 mm | 118 to 129 s |

No violation in any run (18 of blinkSP1, 6 each of ulx3s and ALU, one of RAM Selector Tree), and the check made from the written file is clean on all five boards. On RAM Selector Tree one via of 207 is sent back by the check (a trace it pushes comes 45 µm too near a pad); the first version undid the whole round over it, because it looked for the via near the violation instead of along the trace.

What it does not do: the share of vias under 90 degrees hardly moves on ulx3s. Those are short stubs from a pad to a via beside it, and a step within the via's own triangles does not take it round the pad.

A via's position is part of the geometry, not of the topology: moving a site inside the ring of triangles around it changes no gate order. So the position is optimised here, after relaxation, against every trace it affects.

1. **Force on a via.** Total trace length changes with the via's position at a rate given by unit vectors read off the relaxed polylines:
   - each trace that ends on the via pulls it along the trace's first segment (on both layers);
   - each foreign trace bent around the via pushes it away, along the sum of the two directions in which the trace leaves the bend.

   The sum is the direction that shortens the board fastest. A via in the way of five traces is pushed by all five, which is the point: the traces going round it get shorter, not only its own.
2. **Step.** Move along the force with backtracking, keeping: every triangle around the site right way up on every layer; the via's clearance to fixed copper on every layer; no spoke over capacity; via-to-via spacing.
3. **Update** only what moved: vertex coordinates, lengths and midpoints of the edges at the site, spoke capacities.
4. **Relax again** only the wires that cross the triangles around moved sites. Relaxation windows depend only on the topology, so the other wires are unaffected.
5. Repeat until the largest move is below 1 µm or a round limit is reached. Keep the result only if total realised length went down and the check is clean, as corner smoothing does.

**Travelling further (M16).** A via pressed against the edge of its ring with force left over needs that edge flipped. The flip exists (`sites.flip`, 12.2); what remains is to use it while sliding. Decide from how often vias end pinned in M15.

### 13.3a Sharp vias placed again (tried 2026-11, shelved; branch `reseat-vias`)

**What the sharp vias are** (drawn and looked at, the sharpest dozen on two boards):
- *blinkSP1*: fan-outs from fine-pitch pins. The stub from the pin goes out past the row to the via, and the trace on the other layer comes straight back under it. Round the pad, the only place a via is legal and makes the way shorter is where it already stands (U1-14, U1-28): between the row and the large pad beside it there is no room. Forced, and the way a person fans out such a part.
- *RAM Selector Tree, app settings*: V's, not stubs (no short leg under 2 mm). One trace runs along the edge of a bundle of others to the via, the other comes back along the bundle on the other layer. The worst are on connections with two vias, and the board is full: from the pad of one of them the search with gates held to capacity reaches 504 of 13,902 triangles. The places that would straighten the V are walled off by full gates. Together the 15 under 90 degrees cost 61 mm against vias their traces would run straight through: 0.25 % of the copper. Mostly a matter of looks.

**Tried: a pass before sliding** (`slide.reseat`, setting "Re-place sharp vias"). Each via whose traces meet at under 100 degrees: up to four places, the best expected first (the straight line between the traces' corners, back along either trace, or, for a stub from a pad under 4 mm, round the pad where a via is legal); the connection lifted and planned again with that via held to the triangles within 0.75 mm of the place (`path.find(within=...)`), its other vias to theirs, no running through pads of its net (else it escapes by its net's copper, up to 155 mm longer drawn); the board drawn, the place kept if the copper (vias at their worth) is shorter and the check no worse, else `Context.restore`. On a saved state it looked worthwhile: blinkSP1 2.2 % less copper, RAM Selector Tree 3 of 17 sharp vias straightened (one from 48 to 153 degrees, 30 mm shorter).

**Measured in the router, against the same code with the pass off** (usual spreads): sharp vias blinkSP1 9.5 to 10, ulx3s 7 to 7, RAM Selector Tree 7.5 to 6 (default) and 15/10 to 17/8 (app settings); copper within 0.2 %; 3 to 20 % slower; ALU, Word of RAM and Micromouse unchanged. No gain. The likely cause, not confirmed: the copper it judged by was the plain sum of the drawn traces, which counts twice a trunk two wires of a net share, so it could "save" by splitting a shared trunk, and the moves it kept made sharp vias elsewhere. Judged by the copper as drawn (`copper_length`) and refusing a new sharp via, it might do somewhat better, but most sharp vias are forced, so at best about one in seven. Not pursued.

**Measured again (2026-11, branch `reseat2`), after the board review flagged sharp vias as the most visible fault.** The pass judged by the copper as drawn (`copper_length`, shared trunks once) and kept a move only if it left fewer sharp vias. Measured as a person sees a via: on the finished board, the straightest pair of its traces on different layers, read 0.5 mm along each (so a rounded corner just past it does not count), under 90 degrees is sharp. Seven boards (mixer, blade13, stickhub, bling, openair, castor-mainboard, lily58-pro), two seeds each: 122 of 647 vias sharp without the pass, 120 with it; copper 0.1 % less, 11 more vias, 49 % slower. Not kept. Why, from the reviewed cases (mixer, openair): the sharp vias a person would move are mostly where several connections of a net meet, or a fan-out stub pressed against its pad. The pass sees only a via where one connection changes layer, and re-planning one connection cannot move a via others share. A pass for those would have to move a junction with all its connections; 1.1 is to do that (13.3b), and until then the 1.2 editor is to let them be moved by hand (`ROADMAP.md`).

One finding kept from it: on the test board of `test_connections_of_a_net_share_a_via`, the shared via costs one trace a way round 4.5 mm longer than a second via is worth, and `replan` does not see it through the middles of gates.

### 13.3b Acute angles where copper of one net meets (1.1; branch `acute`)

Sharp junctions and acid traps are one fault: copper of one net meeting itself at an acute angle. The wedge between two traces of a net, or between a trace and its pad, traps etchant when the board is made, and looks wrong. (Copper of different nets is kept apart by the clearance, so it makes no trap.) The places it happens, and what each needs:
- *A trace into its pad.* The trace may bend away over its last stretch so that it meets the pad's edge at no less than the set angle. Teardrops (on by default) already fill most of these wedges; the bend is for a pad without one.
- *A trace joining a trace of its net* (a branch of a net's tree off a trunk it shares). The branch point moves along the trunk, or the branch bends, until the angle is no longer acute.
- *Three or more traces at a junction, a via or a fan-out stub.* The case 13.3a measured: re-planning one connection cannot move a via others share. The junction moves with all its connections: lifted together, planned again with the junction held near a new place, kept if no angle is acute and the copper and checks are no worse.

One setting gives the least angle (90 degrees by default) for all three, and the check counts every meeting under it, as 13.3a's measure did for vias.

**The measure (`realize/acute.py`, `meetings`).** The copper of a net on a layer, as drawn: its traces at their drawn width (tapered ones in their pieces), its pads and vias (round ones as true circles), its teardrops and wedges, as one shape. Where the outline of that shape turns into the copper, the empty wedge outside the corner is where etchant would be trapped; every such corner whose wedge is under the least angle (less half a degree: a branch square to its trunk meets at 90) is a meeting. The angle is read 20 microns along the outline either side, so the facets of a drawn arc and notches of no depth do not count; two corners within 40 microns are one. Corners where only pads meet (a board's own overlapping pads) are not counted. Each is named by what meets there: `pad` (a trace and its pad, or traces at a pad), `via`, `tee` (traces of the net away from any pad: a branch off a shared trunk, or two crossing), `fanout` (a fan-out via or trace is one of the two), `bend` (one trace on both sides: in practice the waist of a pair of teardrops). Apart from those, `through` counts 13.3a's sharp vias, where even the straightest pair of a via's traces on different layers, read 0.5 mm out, meets under the angle: no etchant is trapped there, but it is how the board looks. The counts are in the router's stats, the CLI output, the sweep records (`acute`) and `bench/acute.py`, which lists every meeting and draws a picture of each.

**Measured first** (tier 1, main d6d7b1e, one run each, teardrops on): lumen-blade13 pad 9, via 16, tee 17, fan-out 8, through 3 (59 vias); sweep-half-swept pad 54 (a reversible footprint whose pads of one net are stacked several deep), through 1; hackrf-lna915 pad 8, fan-out 5; kicad-sonde pad 7, tee 1, fan-out 2; kicad-complex-hierarchy pad 8, tee 1, bend 2; kicad-ecc83 pad 4; castor-expander pad 6, fan-out 1; sweep-mini via 1, tee 3, bend 2; kicad-pic-programmer pad 3, tee 1; fomu fan-out 4, bend 2; lumen-ringlight pad 1, fan-out 7; lumen-ftp none. Looked at: the via meetings were mostly teardrops, not traces: two teardrops at one via leave a notch between them, and the pair of teardrops on a trace too short for two (the hourglass, section 14) has a waist of 84 to 88 degrees where one pad is wide. The tees are traces of a bus net (RS-485 on blade13) that run together and part at 4 to 30 degrees. The pads are traces leaving a rectangular pad near a corner, at a slant through a long side.

**Built.**
- *Blunting* (`acute.blunt`), after the corners are rounded and before the teardrops, on the traces as drawn. For each meeting of kind pad, via or tee, sharpest first, the ways tried, the one that adds least length first: a trace into its pad leaves it square to one of its edges (from a point half its width inside the pad, out to a clearance and a half-width beyond the edge) and goes on from there to the point where it last comes within that of the pad (the pad with the pads of its net it overlaps, for stacked pads); a trace that leaves a pad or via, or a trunk, with another of its net runs along the other to where it would be a clearance off it and squares off to its own way there, so the branch point moves out along the trunk (at a via the two share a stub). A way is kept only if the new stretch keeps every clearance (foreign copper, keepouts, edge, foreign traces, as smoothing's arcs do) and nothing foreign lies between it and the trace it replaces, and if the net then meets itself fewer times on that layer, counted with the teardrops the traces would get. Those traces then go in where the pours keep their room and the check finds no more, as the traces straightened over their own vias do (`router._over_own_vias`). Each trace still joins the same two pads; the topology is not touched (as with rounding corners, it is drawing).
- *Teardrop pairs with no acute waist* (`teardrop.WAIST`): the pair on a short trace is made narrower, as far as need be, so that its waist is no sharper than 90 degrees; failing that, as before.
- The setting "Least angle where a net meets itself" (`Options.least_angle`, `--least-angle`, 0 to 90, 0 = no blunting; the counts are then taken at 90).

**Measured** (local sweep of tier 1, against taut2 95d67f5, which is main d6d7b1e's code): every board routes as many, with the same KiCad open items (104), cut-off pads (47), DRC the routing causes (9), islands (37) and mask/thermal (12); routing time 152 to 156 s. Meetings of all kinds, before (main) and after: lumen-blade13 53 to 36, sweep-half-swept 55 to 34, kicad-complex-hierarchy 11 to 3, kicad-sonde 10 to 4, hackrf-lna915 13 to 9, kicad-ecc83 4 to 0, kicad-pic-programmer 4 to 1, sweep-mini 6 to 2, fomu 6 to 4, lumen-ringlight 8 to 7, castor-expander 7 to 7, lumen-ftp 0. What is left is mostly fan-out (not blunted), traces in the stacked pads of sweep-half-swept, and pads with no room to be left square.

**Tried: no slack beyond the clearance** for the new stretch (the routing's own traces stand on it; the clearance holds the routing margin already). Most of the tees of a bus that runs beside others then pass (blade13 tees 17 to 2), but sweep-half-swept got one more island of pour in KiCad (37 to 38), which the pour estimate does not see (it is 30 islands off on that board), even with the check below. Not kept; the stretch keeps 0.1 microns beyond, as rounded corners do.

**A sliver in a via.** The GitHub sweep of the first version (tiers 1 to 3) found every board routing as many, with the same open items, cut-off pads, islands and mask/thermal, and DRC 339 to 335, but for one track KiCad finds dangling on kicad-cm5-minima: a trace from a pad to a via was squared out of the pad and rejoined its old way 0.19 mm from the via's centre, inside the via, and what was left of it there was written as an arc of 0.06 mm. A new corner is now refused within half the trace's width of the pad or via at the trace's other end (cm5-minima: 0 dangling again).

**Islands.** The traces redrawn here, and those straightened over their own vias, now also go back where the estimate shows an island of pour that overlaps none it showed before (`router._over_own_vias`).

Not done: a junction moved with all its connections (the third case above) and 13.3a's `through` vias. Re-planning was tried twice (13.3a); the measure shows those meetings are not acid traps, and the same-layer ones at vias are fixed in the drawing. Fan-out stubs (`fanout`) are made after blunting, by `plan.fanout`, and are not blunted.

---

## 14. Post-optimisation (M8)

- Remove collinear points and shorten by re-running relaxation with a tighter tolerance.
- Optional: round corners to arcs where clearance allows (fillets).
- Optional: via minimisation once vias exist.
- **Straight pad exits.** A trace must leave a pad in a straight line from the pad centre. No sharp kink is allowed between the short stub inside the pad's keep-off ring and the taut (elastic) part of the trace. This means the pad edge a wire leaves through is chosen at realisation time, not fixed by the search.
- **Teardrops** on circular pads and vias: the trace widens smoothly into the pad (tangent lines from a point on the trace to the pad circle). A teardrop wants breathing room (1.5 clearances from foreign copper). Where it has none it is shortened, then held to the plain clearance every trace keeps, then narrowed, and only then left out. Until then it was left out as soon as the shortened one had no breathing room, and the room was measured from the whole shape, the part over the pad's own copper included. Vias stand at the least clearance from their neighbours, so they lost most: of the trace ends on vias, 12 of 122 on blinkSP1 and 6 of 70 on ulx3s had no teardrop while the other end on the same via had one; all but three of those would have kept 1.0 to 1.5 clearances. Now 1 of 122 and none of 70; the written files measure clean. On a trace shorter than two teardrops (a via beside the pad it serves) each used to reach past the other, with its point inside the other's pad, which drew as crossed triangles (22 of 253 teardrops on blinkSP1). Cutting each to half of the trace between the pads cured that and left a notch either side of where they met: a teardrop made by itself ends in sides tangent to the trace's round end, and two such ends cross. On such a trace the two are now made together (20 traces on blinkSP1): each takes half of the trace, and both end on one line across it, as wide as the trace, so the pair is an hourglass whose halves touch along that line and nowhere else. (One straight-sided piece from pad to pad was tried first and looked wrong: a trapezium, not teardrops.) The session file's fill (`ses.teardrop_tracks`) runs its traces from that line, which for an ordinary teardrop is one point, as before. If the pair has no room, each end is tried by itself. SES wiring has no filled shapes, so in the session file a teardrop is written as a few short traces that widen towards the pad (`--teardrops tracks`, the default). `--teardrops zones` writes them instead as KiCad teardrop zones in a routed copy of the `.kicad_pcb` (`io/kicad_board.py`), and `--teardrops none` leaves them out (`--no-teardrops` and `--no-ses-teardrops` say the same). Teardrops are made after everything else, so the app makes them again, and only them, when their settings change after routing (`router.redo_teardrops`).

---

## 15. Code layout

```
weaveengine/
  io/        dsn.py  ses.py
  geom/      inflate.py  triangulate.py  capacity.py
  topo/      planar_map.py  state.py  kernel.py  search.py  sites.py  costs.py  barrier.py  runs.py
  plan/      context.py  path.py  candidates.py  select.py  commit.py  ripup.py
  realize/   relax.py  drc.py
  cli.py
tests/       unit + property tests
bench/       board generators and the benchmark runner
```

Key module contracts:
- `planar_map.build(board) -> PlanarMap` (all arrays and tables, immutable after build)
- `TopoState(planar_map)` with `insert(wire_id, steps)`, `remove(wire_id)`, `check_invariants()`
- `search.route(pmap, state, src, dst, ...) -> Route | None` on one layer, where `mode` is `normal`, `relaxed` or `corridor`; `path.find(ctx, conn) -> Path | None` across layers; `Context.commit(conn, route or path) -> bool` and `Context.rip(wire)`
- `runs.cross_count(pathA, pathB) -> int` (pure function over gate sequences)
- `realize.relax(state) -> dict[wire_id, polyline]`

---

## 16. Performance plan

Targets to validate with measurements (these are guesses, not results):
- Triangulation and table build for about 5,000 pads: a few seconds.
- A single `search.route` on a mid-size board: low milliseconds average.
- Phase 3 should converge in tens of rounds, not hundreds, on benchmark boards.

**Status (M9, measured by `python -m bench.perf`, results in `bench/results.md`):** all three targets are met. What got it there, in order of effect:
1. The search loop and the string pulling of relaxation are compiled with `numba` (`topo/kernel.py`, `realize/kernel.py`). `numba` is optional: without it the same loops run in Python and give the same routing, only slower. Profiling had shown more than 95 % of the time in these two loops.
2. Relaxation is exact in one pass per wire since M14e (13.1); there is no iteration to converge.
3. (Until M14e: the clearance solve along a wall was cached per map. There is no such solve now.)

**Speed pass (M17).** Measured first, per phase and per caller of the search, on all five boards (one variant, one process). Each step below either leaves the routing identical (checked by a fingerprint of every trace) or is a change of behaviour measured as a spread.

| Board | Before | After | Routed before | Routed after |
|---|---|---|---|---|
| RAM Selector Tree | 1041 s | 104 s | 398 of 400 | 399 (397 to 400) |
| ALU | 29 s | 11 s | 409 of 409 | 409 (408 to 409) |
| ulx3s | 41 s | 6 s | 199 to 202 of 203 | 201 (196 to 202) |
| blinkSP1 | 10 s | 5.3 s | 46 (40 to 52) of 58 | 49 (46 to 53) |
| Word of RAM | 1.9 s | 1.6 s | 85 of 85 | 85 of 85 |

What did it, largest first:
1. **No second search when the cheapest path crosses a gate twice** (8.4). The only change of behaviour. It was 80 % of the search work on ALU and most of RAM Selector Tree's (54,000 searches of 35,000 nodes became 22,600 of 16,600).
2. **Via points once per layer and hop.** The points where a search may change layer were worked out again for every layer it might change to. On ulx3s (four layers) that was 25 s of 36.
3. **One plain search per layer** serves both the candidates on that layer and the start of the route through vias; they were two identical searches.
4. **Via points in one compiled pass** (`kernel.via_points`): the five points per triangle, the legality grid, the bound, the room and the spacing from other vias, with no arrays in between.
5. **Points located in a grid of the triangles** (`kernel.cells`, `kernel.locate`), about two cells to a triangle: exact, with no walk. The walk from a hint scanned every triangle whenever it ran into a wall, which is what a point inside a pad on the other layer does.
6. **Transitions brought up to date when a search needs them** (`PlanarMap.catch_up`), not at each of the dozen flips a via site causes; the search reads the map's edge tables instead of a copy made at every change.
7. Crossing counts walk the gates two paths share, not the whole of one path; a reach reads its paths through an index array instead of a dictionary of every node; the selection loop of Phase 1 works on one number per gate and plain lists (half its time).

Tried and not kept, with the reason:
- **Filling the location grid once** and letting hints go stale: locating went from 13 s to 49 s on RAM Selector Tree (stale hints walk into walls). I had committed it on the strength of small boards; a clean run of the large one showed it.
- **Heap entries side by side in one array**, **a plain square root in place of `hypot`**, **not queueing nodes over the bound**: no measurable change. The search costs 30 to 60 ns a node on an empty map and about 110 ns in a real run, where its workspace (16 places per half-edge) does not stay in the cache.
- **Heuristic weight** 1.1, 1.25, 1.5: the mean number of nodes falls by a third at most, because it is set by the searches that flood: those that find no route, and those that start from thousands of via points.

Measured for the rip-up question (12.8), nothing changed: wires ripped up rarely come back by the route they had (blinkSP1 7 of 581, ALU 32 of 253), so ripping is not wasted in that sense. The exception is the tail of a run that has stalled (ulx3s: the last rounds rip three wires and get the same three routes back, until the stall limit ends it).

Where the time is now (RAM Selector Tree, 104 s): the search kernel 41 s, flips of via sites 15 s, candidate selection 7 s, realisation 6.5 s. Not done: the reading of routes back and the rest of the selection loop are plain Python (about 8 s together); flips make two shapely points per edge written.

Techniques: per-half-edge precomputed successor tuples; integer node ids; `dict` visited sets; a stamp-free design (clearing small dicts per search); inverted indexes for `cross_count`; parallel candidate generation; optional A* landmark heuristic. Re-profile after every milestone and record numbers in `bench/results.md`.

---

## 17. Testing and benchmarking

### 17.1 Correctness
- **Triangulation checks:** every obstacle boundary segment appears as an edge; no triangle overlaps a hole; areas sum to free-space area.
- **Invariant property test:** randomly insert and remove wires with the search; after every operation call `check_invariants()`.
- **Independent crossing oracle:** realise geometry (section 13) and test all wire pairs for segment intersection with `shapely`. The topology must never produce a crossing; the oracle must report zero.
- **`cross_count` vs geometry:** on random path pairs, compare the topological count with geometric crossings after realisation.
- **Union-find closure vs brute force:** compare closure detection against a flood-fill connectivity check on small cases.
- **Tiny-board oracle:** brute-force enumerate all routings on very small boards to confirm the search finds a solution whenever one exists.
- **Unit tests:** capacity formula, orientation/end logic, DSN round trip.

### 17.2 Benchmarks
Synthetic generators plus a few real boards:
1. Random pad grids with random 2-pin nets (including a "solvable by construction" variant).
2. Escape-routing case (dense pad array to periphery).
3. Channel-routing case.
4. A few real boards exported from your CAD tool. First one: `boards/Word of RAM.dsn` (KiCad, 2 layers, relay-based 6-bit word of RAM; through-hole parts with custom footprints, a 0.5 mm power class next to the 0.2 mm default, hyphenated references).
5. Boards that need vias: `boards/blinkSP1.dsn` (59 connections, mostly surface-mount on one layer, 1.0 mm vias; runs in seconds, so it is the board to iterate on; its own saved routing is the reference: at least 51 connections, 33 vias) and `boards/RAM Selector Tree.dsn` (400 connections, through-hole; minutes per run, so only for confirmation).
Compare against a baseline (Freerouting or your CAD tool's router) on completion, length ratio and runtime.

### 17.3 Experiments that decide what stays in the design
- **E1:** greedy shortest-first (baseline) vs regret order.
- **E2:** with vs without Phase 1 global selection.
- **E3:** with vs without airwire crossing and closure costs (the core hypothesis of this document).
- **E4:** with vs without demand map.
- **E5:** value of Phase 3 and of each of K = 1, 3, 6, 10.
Keep a feature only if it improves completion or length ratio without disproportionate runtime.

---

## 18. Milestones and acceptance criteria

| # | Milestone | Acceptance |
|---|---|---|
| M0 | Repo, benchmark generators, runner | Boards generate; runner records metrics |
| M1 | DSN reader, inflation, triangulation, capacity | All triangulation checks pass on all benchmark boards |
| M2 | **Spike:** `TopoState` + slot-aware search, single layer, 2-pin nets, length cost only | Invariant test passes on 10,000 random operations; tiny-board oracle agrees; **go/no-go decision** |
| M3 | Geometry relaxation + DRC + SES writer | Routed output imports into the CAD tool; zero crossings by oracle |
| M4 | Congestion, history, regret order, Phase 3 rip-up | Completion better than greedy on E1 |
| M5 | Candidates, global selection, airwire and closure costs | E2/E3 results recorded; keep or cut features by evidence |
| M6 | Multi-pin nets (MST) | Real boards with multi-pin nets route |
| M7 | Multi-layer with via sites (section 12 option 2) | 2-layer benchmark boards complete |
| M8 | Post-optimisation: straight pad exits, teardrops on circular pads, optional arcs | Length ratio improves vs M7; no kink at pad exits; teardrops pass DRC |
| M9 | Performance pass | Meets the section 16 targets or targets are revised with data (done: see section 16) |
| M10 | Parallel execution (section 22) | Wall-clock time on the ALU board drops substantially on a multi-core machine; results independent of the worker count (done: see section 22) |
| M11 | Desktop application (section 23) | `python main.py` opens the app; a DSN can be imported, routed with live progress, and exported as SES; a single PyInstaller command produces a working standalone build (done on macOS: see section 23) |
| M12 | Two fixes: rule defaults for a DSN without them (section 4); nothing hidden can be selected (section 23) | Each of the rule-less variants of a real board (no `rule`, width only, clearance only, no `via`) loads, reports what was defaulted, and routes; with outlines or a layer switched off, clicking where they were selects nothing; tests for both (done; the rule-less variants of RAM Selector Tree load, and a small rule-less board routes clean in the tests) |
| M13 | **Spike:** mutable map and site creation (12.2), one layer pair, no search changes | Invariant test passes on 10,000 random insert, remove, create-site and wake/sleep operations; a hand-placed via realises with zero DRC violations and zero crossings; **go/no-go decision** on the 2 µm hole. **Done: go.** 10,000 operations (120 sites, about 800 flips) keep the invariant and the map's tables consistent, with no crossing in the realised result; a hand-placed via is clean on both layers at 55 of 55 random positions with 2 or 3 foreign traces going round it per layer. On ALU and RAM Selector Tree a site costs 0.8 to 0.9 ms to create, flips included; 300 dormant sites (11 % more triangles) slow a search by 9 to 14 %, and the search finds the same pairs in 400/400 and 396/400 cases. Edge flips, planned for M16, were needed here and are in |
| M14 | Vias during the pass: prototype (12.3) | RAM Selector Tree connects everything in one pass with zero violations; other boards no worse. **Not met.** Built and measured (12.4): 383 of 400 with 206 vias on RAM Selector Tree, 37 of 59 with 33 vias on blinkSP1; ALU, Word of RAM and ulx3s not re-measured. To be reworked as M14a to M14d |
| M14a | Pad-exit stub (12.5 step 1) | No clearance violation on blinkSP1 comes from a stub; connections routed without vias on blinkSP1 goes up from 26. **Done.** blinkSP1, one variant: violations at the first check 16 to 0 without vias and 30 to 5 with the prototype's vias; routed 26 to 28 without vias and 37 to 46 (42 vias) with. 78 of 965 pad edges on the top layer are restricted; one pad (U1-33, the pad under the chip) has no legal way out on its layer, so the board now counts 58 connections, not 59. Other boards not re-run, by decision |
| M14b | Deleting a via site (12.5 step 2) | 10,000 random operations including deletions keep the invariant and the map's tables; a map with every site deleted equals the map before any was made, up to edge flips. **Done.** 10,000 operations with 220 deletions, none refused; with every site deleted the vertex, edge and triangle counts, the pad edges and the walls are those of the original map. 0.8 ms per deletion on the test grid. blinkSP1 routes as before (46 of 58, one variant) and no sleeping site is left in its maps |
| M14c | One search across layers for every phase (12.5 step 3) | blinkSP1: at least 51 of 59 with no more than about 40 vias and zero violations; `complete` and the between-passes code deleted. **Partly met.** 52 of 58 with zero violations, and the old code is gone (the tree is about 740 lines shorter); but 63 vias, and the result swings with small changes (12.4). Other boards not re-measured except Word of RAM (85 of 85, now with 6 vias) |
| M14d | The open questions of 12.5 step 4, on blinkSP1; then RAM Selector Tree, ALU, Word of RAM, ulx3s | blinkSP1 within reach of the file's own routing (51 or more connections, about 40 vias or fewer), steadily; the M14 criterion above; boards that need no via take none or nearly none |
| M14e | Realisation as the rubber band with thickness (12.8, 13.1, 13.2) | The saved RAM Selector Tree routing realises with no violation; RAM Selector Tree end to end improves; other boards no worse. **Done.** Saved state: 82 violations to 0. End to end: 360 to 368 of 400 in 9 minutes instead of 18, with no repair round needed (393 when rip-up is given more rounds: 12.8); blinkSP1, ALU, ulx3s and Word of RAM as before or slightly better, all clean (table in 12.8). `realize/` lost about 100 lines over it (relax, kernel and funnel: 800 lines; relax and kernel now: 700). Not done: the connections RAM Selector Tree still leaves open |
| M17 | Speed pass (16) | Every board faster with routing identical or better over perturbed runs; each step measured. **Done**: table in 16. |
| M15 | Sliding vias (13.3) | Total length on RAM Selector Tree and ALU drops against M14 with the check still clean; added geometry time recorded. **Done on the branch `sliding-vias`**: copper down 3 % on blinkSP1, 1 % on ulx3s, 0.4 % on RAM Selector Tree (11 s more), the length a via costs down by a third, no violation; figures in 13.3. |
| M16 | Via reduction in Phase 4; flips while sliding (the flip itself exists since M13) if M15 shows vias pinned | Fewer vias at equal or shorter length on the benchmark boards |

---

## 19. Risks and open questions

1. **Slot-aware search complexity.** The state space grows with wires per gate. If M2 shows it is too slow or too fragile, fallbacks are: plain capacity-only search plus a separate planarisation step (compute pairwise orders with `cross_count`, resolve conflicts by rip-up), or dynamic triangulation where wires become constrained edges.
2. **Capacity is an estimate.** Slanted and skinny gates can mislead. The realisation and DRC feedback loop (13.2) is the safety net.
3. **Length estimate in search is crude** (midpoint to midpoint). It overestimates taut length. Phase 4 compares realised lengths for this reason; consider a funnel-based correction if the gap is large.
4. **Vias** (section 12). Settled: a via site in the map realises as a correct via; the map can change under the compiled search; a flood across layers is cheap (about a millisecond each on RAM Selector Tree). Not settled: where vias should go. The prototype decides too late and too locally (12.4). The rework (12.5) rests on one unproven operation, deleting a site from the map, and on rip-up behaving once vias are inside it; rip-up's failure to settle today is explained by the missing vias on `blinkSP1` (12.4, finding 2), but that explanation has not been tested by seeing it settle.
5. **Closure and airwire costs are unproven** (E3). Their weights are hard to set; expect tuning per board family.
6. **Net classes** (different widths and clearances) will need per-class inflation and either per-class maps or conservative merging.
7. **Python speed.** If targets are missed after algorithmic fixes, move the search kernel to `numba`, keeping the code layout.
8. **Open decision:** DSN/SES versus direct KiCad file support. Confirm what your CAD tool can exchange.

---

## 20. Parameter defaults (starting guesses, all to be tuned)

| Parameter | Default | Meaning |
|---|---|---|
| `t`, `s` | from board rules (for example 0.15 mm each) | trace width, spacing |
| `K` | 6 | candidates per net |
| `alpha` | 0.5 | candidate cost slack over shortest |
| `lambda_x` | 2 x median pad pitch | cost per airwire crossing |
| `lambda_sever` | 50 x median pad pitch | cost per severed net |
| `lambda_conf` | 5 x median pad pitch | cost per candidate-pair crossing |
| `w_d` | 0.5 | demand weight |
| `pres_fac` | 0.5, x1.5 per round | present congestion weight |
| `hist` increment | 1.0 per overflow unit | history growth |
| `cross_penalty` (relaxed search) | 10 x median pad pitch | per violated crossing |
| `via_cost` | 1.7 x the cap on an over-full gate's price (the cap is 10 x `cross_penalty`) | what a via costs in the search. Above the cap, so that a connection changes layer only when rip-up has failed to sort out its own layer (12.7). Measured on one board |
| `max_vias` | 4 | vias one connection may take |
| Site hole radius | 2 µm | size of a site in the map (12.2). Internal: `sites.SITE_RADIUS`. Not the via's drill, which is a rule (0.3 mm unless the DSN or the settings say otherwise) |
| Via slide rounds / tolerance | 8 / 1 µm | stop condition of 13.3 |
| Slot cap per gate | 15 | search state limit |
| Arc drawing | 6 degrees per segment at most, corners at most 0.4 µm outside the circle | `ARC_STEP`, `ARC_OUT` in `realize/relax.py` |
| Repair rounds | 4 | times a layer is pulled again with added discs (13.2) |
| Phase 3 iteration limit | 100 | stop condition |

---

## 21. Glossary

- **Gate:** an interior triangulation edge a wire can cross. Capacity is how many wires fit.
- **Corner (of a wire in a triangle):** the vertex shared by the two edges the wire uses in that triangle; the wire cuts that corner off.
- **Slot:** the position of a wire within a gate's ordered list.
- **Airwire:** the straight line between two pads that must be connected.
- **Homotopy class (topology):** two routes are equivalent if one can be slid into the other without crossing an obstacle. Here, equivalent to having the same gate sequence after pulling taut.
- **Regret:** how much worse a net's second-best option is than its best.
- **Negotiated congestion:** repeatedly rip up and reroute, raising the price of overused resources until nothing is overused.

---

## 22. Parallel execution (M10)

Use more than one core wherever it is applicable, practical, and gives a real, substantial speed-up. Do not parallelise for its own sake: measure first (section 3 rule 8) and keep only what pays.

**What is in place** (`weaveengine/parallel.py`):
1. **Racing variants of a pass** (the big one). How a pass ends depends strongly on small early differences: on the ALU board single runs varied between one pass with no vias and five passes with 18 vias. So the spare cores each run the whole pass with a different seed and search weighting (`Options.portfolio`, default one per worker, at most 8). Variants are taken in a fixed order: the first that connects everything is used, otherwise the best one.
2. **Phase 1 candidate generation**: one batch over all connections, in worker processes.
3. **Layers**: maps are built, and geometry (straightening, relaxation, DRC, teardrops) is done, one layer per worker. The two layers of a connection are searched at the same time in threads; the compiled kernel releases the interpreter lock.
4. **Via proposals** for all open connections, and **benchmarks**.

Workers are forked, so they inherit the maps and the state without copying; only small task data is pickled (a batch costs about 15 ms to start). Without fork (Windows) the batches of items 2 to 4 run in one process, with the same results. **The race (item 1) runs in parallel there too** (2026-11): its workers are started afresh, each given a copy of the context once (`parallel.first_accepted`, `_given`); a variant is a whole pass, so the copy costs little beside it. Live events still reach the app, whose events go through a queue that can be handed to a worker (`parallel.travels`); the command line's progress bar shows only the parent's. Tested: the same routing, to the byte, as the forked race (`WEAVEENGINE_START=spawn` forces it where fork exists).

**What was tried and not kept as the default:** rerouting several connections at once against one snapshot of the state in Phases 2 and 3 (`CostParams.batch`). A compiled search takes about a millisecond, less than the cost of handing it to another process, and routes computed blind to each other conflict more. It stays available as a parameter; the default is 1.

**Rules**
- For a given portfolio size the result does not depend on the number of workers or on which finishes first (tested), nor on the process: nothing depends on the order of a set or dict that changes from one run of the program to the next (tested with three string-hash seeds). It does depend on its input to the micrometre (section 4.1, the board sweep), and on the time limit, if one is set (`--time-limit`, 22a).
- The portfolio size itself does change the result (more variants, better chance of a clean pass), so it is a setting, not an implementation detail.
- Everything must still work in a frozen (PyInstaller) build: use `multiprocessing.freeze_support()`; where fork is unavailable, batches run in one process and the race in processes started afresh.

**With vias (12.2):** a variant's result also carries the log of what it did to its maps; the parent replays it, so the kept variant's snapshot restores exactly. Tested on a small board; not yet confirmed on a large one.

**The variants were not all different (measured, then put right).** Each run alone on blinkSP1 and ulx3s: the seed by itself changes nothing (three seeds, the same routing to the millimetre on both), and the eight variants held only five settings of the search weighting, so variant 4 repeated variant 0 on both boards and, on blinkSP1, 6 and 7 repeated 1 and 2. A second heat was therefore mostly the first heat again. What does change the outcome: the search weighting (blinkSP1 53 to 58 of 58 over the five), the growth of the price of an over-full gate (55 to 56, other vias), the number of candidates. The four variants of the second heat now differ from the first in price growth (1.8 for 1.5); all eight give different routings on both boards.

**Still open:** Phase 3 is sequential inside a variant; progress is reported only by the plain variant; a persistent worker pool with the state in shared memory would let single searches be spread across cores.

## 22a. A time limit, and the best routing kept as it goes (`--time-limit`, `Context.late`)

**Why.** A user's command-line run on a large two-layer board went three hours and saved nothing. `--max-rounds` bounds the rounds of rip-up, but rounds vary greatly in length, and phases 1 and 2, realisation and repair are not bounded by it.

**The deadline.** `route_board(deadline=...)` takes a time on the clock (`time.time()`), not a duration, and puts it on the context; `Context.late()` says whether it has passed. Raced variants get it with the context: inherited by a fork, pickled with the copy each spawned worker is given on Windows (`parallel.first_accepted`, `_given`). Being a time on the clock, it means the same in a worker started late as in its parent. The command line counts it from its own start (`--time-limit MINUTES`), the app from the start of the routing process (setting "Time limit", 0 = none). Once it has passed:
- rip-up stops (`negotiate`: beside the round limit, and also between connections of a round, leaving it half done) and the best routing it has seen is put back, as at any stop;
- `fill` makes one pass over the open connections, not as many as place anything;
- `_route_portfolio` starts no further heat; the best of the variants that have run is kept, by the usual rule;
- no further repair after the check: what still violates is dropped, as after the last repair (a repair under way skips its rip-up); the settling after repair and the sliding of vias are left out;
- `settle` makes the routing legal and places what fits (`legalise`, `fill`), leaving out refinement, untangling and replanning.

It does not use `ctx.stop` (`parallel.Stopped`): that throws a variant's work away. Phase 1 (candidates and the global selection), commit (phase 2), the realisation of what there is, its check, and the drawing after the race (rounded corners, teardrops, vias to the planes) still run, so a run ends some time after its limit: on lily58-pro (a minute's routing on three workers) the SES was written 5 s after a 12 s limit, all 311 connections routed with 52 vias instead of 90; on hackrf-one (six minutes on ten cores) 36 s after a one-minute limit, 540 of 545 routed, measured clean; there the race ended 10 s after the limit and the drawing after it took the rest, most of it the vias down to the planes. Commit was stopped by the limit at first: on Windows, where hackrf-one's variants were still committing at the limit, `fill` then placed what was left in repeated passes, 208 s past the limit, and the drawing (on one core there) took 75 s more. With commit left to finish, one pass of `fill` and no repair past the limit, the race there ends 51 s past the limit and the SES is written 134 s past it (500 of 545 routed, measured clean): most of what is left is the drawing on one core. A routing that stopped on time depends on how fast the machine is: the result is deterministic only without a limit.

**The best so far, kept as it goes.** `route_board(checkpoint=...)` is called in the routing process each time a pass or a raced variant finishes, with how good its routing is (`_key`) and a `Result` of it that `write_ses` can write (`_draft`: traces and vias as the pass left them, before corners are rounded, teardrops added or vias put down to the planes). A variant computes its draft itself and returns it with its outcome. `first_accepted` now takes results as they finish (`imap_unordered`), so a variant that finishes before an earlier one is seen at once; it still decides in variant order, so what is kept does not change. The command line (`cli.Checkpoint`) writes the output file whenever a routing is better than the one written last, and says so; Ctrl+C then ends with "the best routing found is in ...". `write_ses` always writes to a file beside the target and renames it over the target (`os.replace`), so the file is the old session or the new one, never half of one. The final result is written the same way at the end.

**Tested:** a deadline already past, forked and spawned: no round of rip-up, no second heat, a clean result; checkpoints and a deadline never reached change nothing; lily58-pro with three seconds ends within an allowance of 30 s and its SES holds every trace and via of the routing; a session whose writing fails part way leaves the one before; the command line writes a better routing and not a worse.

## 23. Desktop application (M11)

A simple GUI started from `main.py` at the repository root (`weaveengine/app.py`, PySide6).

    python main.py [board.dsn]              # run it
    pyinstaller --noconfirm WeaveEngine.spec # build dist/WeaveEngine.app (macOS) or dist/WeaveEngine/

**What it does**
- Import a DSN, route it, export the SES (measured against the rules as it is written) or an SVG.
- **Live view of routing as it happens:** the board filling with traces from the best of the raced variants, wires just placed or rerouted highlighted, both layers with per-layer toggles, vias, unrouted connections as airwires; then the real geometry with teardrops and DRC markers when a pass ends.
- Live numbers: pass, phase, connections routed and open, vias, rip-up rounds, over-full gates, trace length, wires per layer, which variant is shown, and how each raced variant stands.
- A progress bar with elapsed time and time left in the pass. Stop cancels the run and its worker processes.
- A settings editor over `settings.json` (`weaveengine/settings.py`): speed, routing phases and costs, rule overrides, teardrops. **Under each rule it shows what the open board gives** (`settings.detected_rules`: the value and where it came from, the DSN or the KiCad project beside it) **and warns, in orange and on the Rules tab, about a rule the board does not give**: one missing from the DSN (a built-in default stands in), and above all the edge clearance, which no DSN carries (the trace clearance stands in unless a `.kicad_pro` of the same name is beside the DSN). Opening a board logs the same warnings for rules the settings do not set either, and routing reports them as warnings. One function decides all three. (Added 2026-10-16 after a board exported to the desktop, away from its project, was routed with 0.2 mm to its cutouts against KiCad's 0.5 mm, with nothing but a quiet note to say so.) Beside `main.py` when run from source; in the user's configuration folder in a packaged app.
- **Compiled-kernel status** is checked at start-up, off the GUI thread, and shown as a banner; if the kernels are unavailable the reason is shown and confirmed before routing starts. Routing then uses the Python fallback; it never crashes on this (`weaveengine/accel.py`).

**Selection follows what is drawn (M12, done).** Clicking selects a pad, else a trace, else a part. A thing can be selected only if its switch is on (`BoardView.pick`).
- A part: only while footprint outlines are switched on.
- A trace or teardrop: only while its layer is shown (as now).
- A pad that is on hidden layers only: neither drawn nor selectable. Through-hole pads and vias stay while any layer is shown.
- Switching something off clears the selection if it was the thing selected.
- Detail that is merely too small to draw at the current zoom stays selectable; that is a drawing shortcut, not the user hiding it.

**How it is put together**
- The window never routes. `weaveengine/session.py` runs the router in its own (spawned) process and passes events back through a pipe: status, progress, snapshots of the routing from every variant, pass results, the final result. The router forks its workers inside that process, away from the GUI.
- Abandoned variants are stopped by a flag and given time to return, not killed, so an event half-written to the shared pipe cannot leave it locked.
- In a packaged app numba's on-disk cache goes to the user's cache folder (the bundle is read-only).

**Tested** by driving the window with no person present (`main.py --self-test board.dsn --ses out.ses --screenshot out.png`), from source and from the PyInstaller build, on both real two-layer boards.

**Drawing speed.** While routing, the picture is sent with the points left out that move a line by less than half a pitch (RAM Selector Tree: 12,000 points instead of 49,000) and drawn as hairlines; final traces are thinned to 4 µm for drawing only. One item per trace: one path per layer was tried and is four times slower. *Draw with the graphics card* (View settings, off by default) puts the view on OpenGL, with the card's own edge smoothing. It is built not to be able to lock anyone out: a context is asked for before the view is trusted to one, and a refusal (no driver, a remote desktop) leaves the ordinary drawing with a note; a marker file beside the settings is there while it is being tried, so a driver that takes the program down gets the setting switched off at the next start. The app's own screenshots draw the view the ordinary way, since a grab of an OpenGL view is empty. Checked on one Mac only; `--self-test ... --gpu` is there to check a packaged build elsewhere.

**Open**: the window has only been exercised through the self-test, not by hand; Windows and Linux builds are untried (without fork the router runs single-process there).

---

## 23a. Going on from a routing (`router.route_board(resume=...)`)

**What it is for.** Routing a board that is routed already used to throw the routing away and start from nothing. Now a result carries the routing itself (`Result.kept`: the states, the log of what was done to the maps, the capacities and histories, the prices, the traces as drawn), and `route_board` given that goes on from it.

**What going on does.** Phases 1 and 2 are not run. The maps are built afresh and the kept routing put on them exactly (`router._adopt`, which is how the parent of raced variants has always taken up the variant it keeps: the via sites are made again from the log, 12.2). Then everything from Phase 3 on runs as in any pass: every connection may be tried again (the marks of earlier failure are wiped), rip-up may run as many rounds again, the routing is settled, drawn, checked, repaired and its vias slid. Raced variants each start from the kept routing, with their different settings.

**Never worse.** A routing is judged by connections open, then violations, then copper (a shared trunk once: `router._key`, which is also how raced variants are now told apart; it was the length of all traces added up, which counts a shared trunk twice and called a routing with more copper the better one on Word of RAM). If going on does not give a better one, the earlier routing is put back and returned as it was (`stats["improved"]` is False).

**Only on the same maps.** A kept routing names every gate by number. It is refused, with a message, if the maps built now are not the maps it was made on (a hash of each map as built): another board, or other rules.

**In the application.** Once a board is routed, "Route" reads "Improve" and goes on from the routing shown; "Restart" takes everything out and routes from nothing. Stopping a run that is going on from a routing, or one that fails, leaves the routing that was there. The time such a run takes is not put on the record of how long the board takes (24).

**Measured** (one variant unless said):

| Board | Routed from nothing | Gone on from once | Twice | Three times |
|---|---|---|---|---|
| blinkSP1 | 56 of 58, 43 vias, 5.2 s | 58 of 58, 41 vias, 3.4 s | no better, 1.9 s | no better |
| blinkSP1, raced | 58 of 58, 50 vias, 1248 mm, 14.8 s | 48 vias, 1236 mm, 2.8 s | 1225 mm | 1224 mm |
| ulx3s | 201 of 203, 31 vias, 8.3 s | 202, 4.7 s | 203 of 203, 4.1 s | less copper |
| Word of RAM | 85 of 85, 1971 mm | no better, 0.6 s | | |

The kept routing is 4 to 5 MB for blinkSP1 and ulx3s as handed between processes.

Not done: it is not saved with the board, so it lasts while the application is open; and it is all of the routing or none (no going on from part of it).

---

## 24. The time left (`weaveengine/progress.py`)

**What was wrong.** The old figure took fixed shares per phase, measured once on ALU, and extrapolated the time so far. It was nearly always too small, for three reasons found by recording the events of real runs: (1) rip-up's "progress" was the share of violations cleared, which reaches 90 % in the first few rounds and then crawls for most of the run; (2) repairs after the check were not counted; (3) when no raced variant connects everything, a second heat of four variants runs, about as long as the first, and the timer followed only variant 0, which is not in it.

**What can be known.** A run's length is open in three places, each a stop rule: rip-up ends some rounds after the last improvement; repair repeats while the check finds something (four times at most); another heat follows if no variant of this one connected everything. Tested on the recordings (28 variants on five boards) whether anything the run reports predicts the rip-up that is left: the rounds the stop rule still allows times the recent round time is the best single predictor, and the truth lies between 0.4 and 3.5 times it in eight cases of ten. Extrapolating the decay of violations was tried and is no better (0.15 to 1.9). So a first run cannot be timed closely, and the estimate says so.

**What is shown.**
- **A middle figure with its range**: "about 1:30 left (0:36 to 3:50)". The low end is the run if nothing more happens (no round improves, one repair); the high end adds the improvements still likely (nine runs in ten stay under 24 extra rounds at the start, halving every 8 rounds), up to three repairs, and rounds a third longer. Every duration is one the run has measured itself as soon as there is one (a round, the geometry, a repair, a heat); before that, shares of the time its variant took to reach rip-up.
- **The next heat in words**, not folded into the range: "and about 2:10 more if no variant connects everything". It is dropped the moment a variant does.
- **Nothing** before commit is under way: there is nothing to go on, and a figure then was the worst of the old ones.
- **The time the same run took before**, when there is one: the board file's contents and the settings name a run, and its duration is kept in `timings.json` beside the settings. Routing is deterministic, so this is the one close estimate there is (within about a tenth); it is used until the run outlasts it.
- The bar is the time gone over the time gone plus the middle figure (with half of a possible heat), and never moves back.

The router's events carry what this needs and nothing else was added to the router: each round of rip-up says which round it is, how many are still to run if none improves, and how many at most; a variant says when it has finished and whether it connected everything. The console bar reads the same events.

**Measured** by playing the recordings back through the estimator (judged on the heat that is running, which is what the range claims):

| Board | Run | Truth inside the range | Truth / middle figure (10 %, median, 90 %) |
|---|---|---|---|
| RAM Selector Tree | 148 s | 98 % of the time | 0.56, 0.86, 1.29 |
| blinkSP1 | 24 s | 93 % | 1.24, 1.66, 3.39 |
| ulx3s | 17 s | 100 % | 0.49, 0.84, 1.55 |
| ALU | 13 s | 98 % | 0.11, 0.41, 1.27 |
| Word of RAM | 1.7 s | 100 % | 0.15, 0.66, 0.66 |

Over all of them the middle figure is right in the median (0.96) and within a factor of about two in eight cases of ten outside the short runs. It is still low on blinkSP1, whose rip-up goes on improving a little for longer than most, and high on ALU, whose rounds get ten times shorter as it goes. The constants are fitted to these five boards and were not checked on a board outside them, except that RAM Selector Tree was recorded after they were chosen. With a run on record: "about 0:21 left" from the second second of a 21 s run.

**With a time limit** (22a) neither end of the range, nor the next heat, goes past the limit; what is done after it (the geometry of the best so far, its check, the drawing) is not counted, and the status reads "time is up: geometry" and so on.

**The report of "40 minutes left" with 2 h 54 min to go** (a large two-layer board, 2026-11) was not traced to one cause without the board. What is known to be short on such a board: the middle figure hardly moves while rip-up keeps improving (below), so a rip-up of a hundred long rounds that improves now and then reads as a few rounds left; after rip-up, repair (up to four times five rounds of rip-up and a geometry), the settling after it and the sliding of vias are timed by shares fitted on five boards of seconds to minutes; and the console follows only the plain variant. None of this is cheap to put right; the time limit gives a bound instead.

**Open.** The middle figure hardly moves while rip-up keeps improving (on RAM Selector Tree it stays near 1:35 for a minute while the truth falls from 1:55 to 0:55): each improvement puts back the rounds the stop rule allows. A second heat doubles a run and is only known at the end of the first.

