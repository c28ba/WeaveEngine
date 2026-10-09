# WeaveEngine: Design Document (v0.1)

**WeaveEngine** (Python package `weaveengine`) is a TopoR-inspired topological PCB autorouter that converts a board into a planar topological map, solves routing as a choice of *topology* (which way each wire passes each obstacle), and then realises that topology as the shortest legal geometry.

---

## 0. Status and honesty notes

- This is a design, not a tested result. Anything marked **[HYPOTHESIS]** is a design bet that must be validated by the experiments in section 17 before it is relied on.
- TopoR's internal algorithms are not public in anything I could verify. This design is built from general topological-routing ideas (homotopic routing, rubber-band sketches, negotiated congestion), not from TopoR's source. Do not describe it as a reimplementation.
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
- A DSN does not carry the copper-to-board-edge clearance or drill sizes. The edge clearance is taken from `--edge-clearance`, or from the KiCad project file (`--kicad-pro`, or a `.kicad_pro` next to the DSN); otherwise it defaults to the trace clearance. Hole clearance cannot be checked from a DSN.
- Routing uses a small extra margin (`--margin`, default 0.01 mm) so that nothing sits exactly on a limit; the written SES is read back and measured against the unpadded rules.
- SES writer emits each wire as a polyline `path` with width. Arcs are approximated by polylines until M8.

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
| Length | every step | midpoint-to-midpoint distance between entry and exit gate in the triangle |
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

Not built: places counted between runs rather than wires (a gate holds 15 wires in the search, riders included); riding through the trunk's via; merging coincident segments in the output.

---

## 12. Layers and vias

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

**Removing a via.** The site is put to sleep and then deleted from the map (12.5, step 2). A sleeping site that could not be deleted keeps half a pitch clear around its hole, because wires pass it on both sides and must not meet at it.

**The 2 µm hole realises as a correct via: go (M13).** This was the hypothesis the spike was for. What it took in `realize/`:
- the keep-off is what a foreign wire keeps from the via at the least, whatever lies in between (13.1);
- for a wire that goes round it, a via is one disc at its centre, not the three corners of its hole. Three discs of half a millimetre whose centres are 3 µm apart have tangents between them that point anywhere (13.1, "what does not carry over");
- the via's own trace: which of the three hole edges it leaves through is the search's accident, so its heading is judged a trace width away from the hole, a trace heading away from its edge goes round the nearer way, and it may take the innermost place inside foreign traces that wrap the via (`terminals.hop`). All three apply to via sites only; ordinary pads behave as before.

Results (details in the M13 row of section 18): 10,000 random operations with the invariant intact; hand-placed vias clean at 55 of 55 random positions, the smallest gap from a foreign trace to the via copper 0.307 mm against a rule of 0.300.

Known limits:
- A point within 20 µm of a triangle's edge cannot hold a site (5 of 60 random positions). The caller nudges the point.
- A via's own trace counts as load on the spokes it crosses, although it needs no keep-off from its own via. On a spoke shorter than the keep-off this under-states capacity.
- Capacity lowered by DRC feedback (`_penalise`) is overwritten when a site at that gate changes state or the gate is flipped.
- `free_space` does not know about vias; only the end-straightening shortcut in relaxation reads it, and DRC checks its result.

### 12.3 Routes across layers (M14c, built)

**The route** (`plan/path.py` `find`). A chain of searches, one per layer the route runs on. The first starts at the connection's pad, on each layer the pad is on. Each later one starts from seeds (8.6): every point the search before reached on another layer where a via may legally be and has room among the wires, at its cost so far plus `via_cost`. The cheapest arrival over all chains of up to `max_vias` vias wins; more vias are tried even after a way is found, because the way with fewest vias is often a long way round. The pieces are read back from the searches' own records, so the plan is exact: there is no second search to turn it into wires.

**Where a via may be.** Inside every layer's free space shrunk by the via's copper and clearance (worked out once, with a coarse grid for testing many points); clear of other vias; and really in the middle cell of its triangle on both layers, far enough from each corner for the keep-off and the wires cutting that corner. Candidate points are five per triangle reached.

**Committing it** (`Context.commit`). For each via in order: its site is made on every layer (12.2) without settling its edges, so that the rest of the plan is still good; the piece ending on it is extended across the site's nearest spoke to the hole and inserted. Then the sites' edges are settled (`legalise`), which carries all wires. If a via turns out not to fit (a gate beside it would be over-full, a later piece passes a triangle an earlier via went into, two pieces on one layer share a gate), everything is taken out again in reverse order and the route is planned once more without that point (`path.place`). A trace may over-fill a gate for rip-up to sort out; a via may not.

**The connection** then consists of pieces, each an ordinary connection from pad or via to via or pad, with the original as parent (`Connection.pieces`, `.sites`). `Context.rip` on any of them removes them all and deletes the vias from the maps (12.5 step 2).

**Racing variants** hand their sites back through the map log (12.2); the parent replays it.

What an earlier prototype had and this does not: a separate completion step after rip-up with its own displacement rule; a second, legality-only flood; spare capacity that grew with each repair round; capacity lifted while searching for a via's own trace; the older code that added vias between passes. About 740 lines fewer.

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
   - It is refused, changing nothing, when a wire that passes the polygon twice would have to cross one new edge twice (10 of 400 deletions in a trial with wires; 0 of 400 without). The site then stays asleep.
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
1. A net changes layer once and branches: a connection may start from any via or through-hole pad its net already has. This is where the file's routing gets 1.65 vias per connection through vias; this router pays 2.
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

A disc inside another on the same side is never touched and is dropped. Where discs of opposite sides overlap, the gate between them is over-full; the trace passes square to the line of centres and DRC reports it.

**Drawing.** An arc is drawn as a polyline round the outside of its circle, in steps of 6 degrees or less, fine enough that its corners stand at most 0.4 µm out (the check allows a micron).

**Terminals.** The wire's first and last points are the pad centres; it must pass through the window on its pad edge. The ring guarantees only that a point on it is clear of foreign copper. Where the rings of neighbouring pads have merged (fine-pitch parts), a stub from the part of the ring over the gap cuts across towards the neighbour. So each pad edge has a window, the part of it from which the stub keeps its clearance, found once per map by trying 17 points along the edge (`geom/exits.py`). An edge with no such part is made a wall, so the search never leaves a pad through it. A pad with no legal edge at all cannot be reached on that layer and is reported with the other unreachable pads. Within the window, pulling taut chooses the point. An end that presses against a corner of its pad edge is moved to the next edge round the pad (`terminals.straighten`, M8).

**What a wire's own triangles do not tell it.** One thing is added before pulling, because it is common and the repair does not converge on it where pad exits crowd each other (13.2):
- *Where a foreign wire leaves its pad.* A wire passing that pad keeps its distance from the point, a disc there. The wires near such points are pulled a second time once the points are known, and a third time with every such point held where it is.
Anything else of the kind is found by the check and repaired (13.2). (For a short while a second case was built in as well, the next vertex along an obstacle that the bundle inside a wire goes round. The repair of 13.2 does the same work: without the built-in case the saved RAM Selector Tree routing has 5 violations at the first check instead of 1 and none after repair either way, and the five boards route the same. It was removed.)

**Cost.** RAM Selector Tree, 583 traces on two layers: 0.3 s per layer for relaxation, 0.1 s for the check.

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
- *Each connection through vias is planned again with the fewest vias first*: as many as its pads force, then one more, up to what it has. A plan is measured as it will be once in (to and from each via counted), must be better by a pitch, and is skipped altogether if the connection is already within a pitch of the straight line with the vias its pads force. Repeated while any connection comes out better (three times at most): one that moves leaves room for another.
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

---

## 14. Post-optimisation (M8)

- Remove collinear points and shorten by re-running relaxation with a tighter tolerance.
- Optional: round corners to arcs where clearance allows (fillets).
- Optional: via minimisation once vias exist.
- **Straight pad exits.** A trace must leave a pad in a straight line from the pad centre. No sharp kink is allowed between the short stub inside the pad's keep-off ring and the taut (elastic) part of the trace. This means the pad edge a wire leaves through is chosen at realisation time, not fixed by the search.
- **Teardrops** on circular pads and vias: the trace widens smoothly into the pad (tangent lines from a point on the trace to the pad circle). A teardrop is shortened or dropped where it would break clearance. SES wiring has no filled shapes, so in the session file a teardrop is written as a few short traces that widen towards the pad (`--no-ses-teardrops` leaves them out).

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

Workers are forked, so they inherit the maps and the state without copying; only small task data is pickled (a batch costs about 15 ms to start). Without fork (Windows) everything runs in one process with the same results.

**What was tried and not kept as the default:** rerouting several connections at once against one snapshot of the state in Phases 2 and 3 (`CostParams.batch`). A compiled search takes about a millisecond, less than the cost of handing it to another process, and routes computed blind to each other conflict more. It stays available as a parameter; the default is 1.

**Rules**
- For a given portfolio size the result does not depend on the number of workers or on which finishes first (tested).
- The portfolio size itself does change the result (more variants, better chance of a clean pass), so it is a setting, not an implementation detail.
- Everything must still work in a frozen (PyInstaller) build: use `multiprocessing.freeze_support()`; where fork is unavailable the single-process path is used.

**With vias (12.2):** a variant's result also carries the log of what it did to its maps; the parent replays it, so the kept variant's snapshot restores exactly. Tested on a small board; not yet confirmed on a large one.

**Still open:** Phase 3 is sequential inside a variant; progress is reported only by the plain variant; a persistent worker pool with the state in shared memory would let single searches be spread across cores.

## 23. Desktop application (M11)

A simple GUI started from `main.py` at the repository root (`weaveengine/app.py`, PySide6).

    python main.py [board.dsn]              # run it
    pyinstaller --noconfirm WeaveEngine.spec # build dist/WeaveEngine.app (macOS) or dist/WeaveEngine/

**What it does**
- Import a DSN, route it, export the SES (measured against the rules as it is written) or an SVG.
- **Live view of routing as it happens:** the board filling with traces from the best of the raced variants, wires just placed or rerouted highlighted, both layers with per-layer toggles, vias, unrouted connections as airwires; then the real geometry with teardrops and DRC markers when a pass ends.
- Live numbers: pass, phase, connections routed and open, vias, rip-up rounds, over-full gates, trace length, wires per layer, which variant is shown, and how each raced variant stands.
- A progress bar with elapsed time and time left in the pass. Stop cancels the run and its worker processes.
- A settings editor over `settings.json` (`weaveengine/settings.py`): speed, routing phases and costs, rule overrides, teardrops. Beside `main.py` when run from source; in the user's configuration folder in a packaged app.
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

**Open.** The middle figure hardly moves while rip-up keeps improving (on RAM Selector Tree it stays near 1:35 for a minute while the truth falls from 1:55 to 0:55): each improvement puts back the rounds the stop rule allows. A second heat doubles a run and is only known at the end of the first.

