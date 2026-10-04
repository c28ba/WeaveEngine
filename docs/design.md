# WeaveEngine: Design Document (v0.1)

**WeaveEngine** (Python package `weaveengine`) is a TopoR-inspired topological PCB autorouter that converts a board into a planar topological map, solves routing as a choice of *topology* (which way each wire passes each obstacle), and then realises that topology as the shortest legal geometry.

---

## 0. Status and honesty notes

- This is a design, not a tested result. Anything marked **[HYPOTHESIS]** is a design bet that must be validated by the experiments in section 17 before it is relied on.
- TopoR's internal algorithms are not public in anything I could verify. This design is built from general topological-routing ideas (homotopic routing, rubber-band sketches, negotiated congestion), not from TopoR's source. Do not describe it as a reimplementation.
- The riskiest piece is the slot-aware topological search (section 8). Milestone M2 is a deliberate spike to prove or kill it early.
- Multi-layer routing with vias is the largest unsolved design question (section 12). The plan is to prove the core on a single layer first.
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
7. **Parallelism.** Candidate generation (Phase 1) is read-only per net: use `ProcessPoolExecutor` with `fork` so arrays are shared copy-on-write. Phase 2 and 3 are sequential in v1.
8. **Profile before optimising.** `cProfile` and `line_profiler` on the benchmark boards. Allowed accelerations in order: better algorithms, an A* landmark heuristic (via `scipy.sparse.csgraph`), `numba` on the relaxation kernel, only then anything else.

---

## 4. Input and output

**Format: Specctra DSN in, SES out** (the same interchange TopoR 4.0 added, which most CAD tools can produce). It keeps the router independent of any one CAD tool.
- A DSN parser is a small s-expression reader. Needed sections: `boundary`, `structure` (layers, rules), `placement`, `library` (images and padstacks), `network` (nets, classes), and optionally `wiring` for fixed existing copper.
- **Open item:** confirm which CAD versions you use can export DSN and import SES (KiCad has historically supported both; check your version).
- Pre-existing wires and vias in `wiring` are treated as fixed obstacles in v1.
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
5. **Same-net rule (v1):** a wire never passes over another pad of its own net; it may end on one. Multi-pin nets are handled in section 11.

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

- Heuristic: Euclidean distance from the current edge midpoint to the nearest target-pad point (use the target pad's centroid distance minus its radius, which is admissible; weight it 1.0).
- Priority queue: `heapq` with `(f, counter, node)`. Costs are floats; `counter` breaks ties.
- Reuse `dict`s per search; clear instead of reallocating.
- Return the gate/slot sequence, or `None`.

### 8.5 Relaxed search (used for rip-up decisions)

When no feasible path exists, run the same search with the feasibility check relaxed: if `r` exceeds `corner_cnt`, clamp `r` to the limit and add `cross_penalty * (r - limit)` to the cost, recording the wires of `a` lying in the violated range as the *blocking set*. The cheapest relaxed path tells you which wires to rip up (Phase 3, section 10).

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
| Via | multi-layer only | `via_cost` |

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

---

## 11. Multi-pin nets

v1: decompose each net into 2-pin connections by a minimum spanning tree over pad positions, then route the connections as independent 2-pin items (each a separate wire id). This loses Steiner-tree optimality, so it is a known limitation. Branches start at pads only. Later improvement: restart a net's routing as a tree grown from its existing copper, with start states on any gate slot adjacent to an existing branch. That requires splitting the wire at the branch point, so it is deferred.

---

## 12. Layers and vias (open design question)

**The difficulty:** a via is a new obstacle inside a triangle, which changes the planar map. Three options, in order of ambition:

1. **v1: single layer only.** Through-hole pads and fixed vias are obstacles or terminals. Proves everything above.
2. **v2: per-layer maps with fixed via sites.** Each layer has its own triangulation and `TopoState`. Candidate via sites are generated before triangulation as small pseudo-pad holes in *every* layer's map, placed sparsely in free space and near pad rows. A via is a wire ending at a site on one layer and a new wire starting at the same site on the other. Unused sites remain wasted obstacles; a pruning pass that deletes unused sites and re-embeds routes is possible but unspecified. **[HYPOTHESIS]** Sparse sites are good enough for 2-layer boards.
3. **v3: dynamic via insertion** by splitting a triangle (1 to 3 split) and updating the ordered gate lists of the new edges from the region the via sits in. Correct in principle but intricate; do only if v2 proves inadequate.

Layer assignment for v2: assign nets to layers by a cheap pre-pass (airwire crossing minimisation between layers), then route layer by layer, using sites for the nets that need to switch.

---

## 13. Geometry realisation (topology to coordinates)

Input: per wire, the gate sequence and the per-gate ordering `gate_order`.

### 13.1 Relaxation (taut string with ordering)

1. **Initial placement.** On each gate with `k` wires ordered `w_1..w_k`, place wire `w_i`'s crossing point at fraction `(i - 0.5) / k` along the gate (gate endpoints already include the clearance inflation).
2. **Gauss-Seidel sweeps.** For each wire, for each interior crossing point, move it to the point on its gate that minimises path length (intersect the line between its two neighbours' points with the gate, clamp to the gate). Then clamp it between its ordered neighbours on the same gate: at least `(t + s)` away from the previous wire's point and from the next wire's point. Repeat until the largest movement in a sweep is below `1e-4 mm` or a sweep limit is hit.
3. **Bends.** A point clamped to a gate endpoint means the wire bends around that obstacle vertex. Because obstacles are already inflated by `s + t/2`, a wire touching the inflated vertex is DRC-legal. Use a mitred/rounded inflation with enough resolution to avoid clearance loss at convex corners.
4. **Terminals.** The wire's first and last points lie on the inflated pad boundary. Add a short *stitch* segment from that point to the pad's anchor (centroid or nearest copper point). The inflation ring guarantees no foreign copper is in between.

Vectorise the sweep with NumPy (all wires processed per gate-index class), or use `numba` for this kernel only.

### 13.2 DRC and repair

- Check with `shapely`: buffer every trace by `t/2`, query an `STRtree` of foreign-net copper for distance `< s`; check against the outline; check wire-to-wire distances.
- If violations remain after relaxation: increase spacing locally and re-relax; if still violating, mark the gates involved, add history cost, and send the affected wires back to Phase 3. Capacity is only an estimate, so this feedback loop is expected.

---

## 14. Post-optimisation (M8)

- Remove collinear points and shorten by re-running relaxation with a tighter tolerance.
- Optional: round corners to arcs where clearance allows (fillets).
- Optional: via minimisation once vias exist.
- **Straight pad exits.** A trace must leave a pad in a straight line from the pad centre. No sharp kink is allowed between the short stub inside the pad's keep-off ring and the taut (elastic) part of the trace. This means the pad edge a wire leaves through is chosen at realisation time, not fixed by the search.
- **Teardrops** on circular pads and vias: the trace widens smoothly into the pad (tangent lines from a point on the trace to the pad circle). A teardrop is shortened or dropped where it would break clearance.

---

## 15. Code layout

```
weaveengine/
  io/        dsn.py  ses.py
  geom/      inflate.py  triangulate.py  capacity.py
  topo/      planar_map.py  state.py  search.py  costs.py  barrier.py  runs.py
  plan/      candidates.py  select.py  commit.py  ripup.py
  realize/   relax.py  drc.py
  cli.py
tests/       unit + property tests
bench/       board generators and the benchmark runner
```

Key module contracts:
- `planar_map.build(board) -> PlanarMap` (all arrays and tables, immutable after build)
- `TopoState(planar_map)` with `insert(wire_id, steps)`, `remove(wire_id)`, `check_invariants()`
- `search.route(state, net, costs, mode) -> steps | None` where `mode` is `normal`, `relaxed` or `corridor`
- `runs.cross_count(pathA, pathB) -> int` (pure function over gate sequences)
- `realize.relax(state) -> dict[wire_id, polyline]`

---

## 16. Performance plan

Targets to validate with measurements (these are guesses, not results):
- Triangulation and table build for about 5,000 pads: a few seconds.
- A single `search.route` on a mid-size board: low milliseconds average.
- Phase 3 should converge in tens of rounds, not hundreds, on benchmark boards.

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
| M9 | Performance pass | Meets the section 16 targets or targets are revised with data |

---

## 19. Risks and open questions

1. **Slot-aware search complexity.** The state space grows with wires per gate. If M2 shows it is too slow or too fragile, fallbacks are: plain capacity-only search plus a separate planarisation step (compute pairwise orders with `cross_count`, resolve conflicts by rip-up), or dynamic triangulation where wires become constrained edges.
2. **Capacity is an estimate.** Slanted and skinny gates can mislead. The realisation and DRC feedback loop (13.2) is the safety net.
3. **Length estimate in search is crude** (midpoint to midpoint). It overestimates taut length. Phase 4 compares realised lengths for this reason; consider a funnel-based correction if the gap is large.
4. **Vias** (section 12) are unsolved in the general case.
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
| `via_cost` | 10 x (t + s) | per via (v2) |
| Slot cap per gate | 15 | search state limit |
| Relaxation tolerance | 1e-4 mm | convergence |
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
