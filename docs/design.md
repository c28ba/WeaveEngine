# WeaveEngine: Design Document (v0.1)

**WeaveEngine** (Python package `weaveengine`) is a TopoR-inspired topological PCB autorouter that converts a board into a planar topological map, solves routing as a choice of *topology* (which way each wire passes each obstacle), and then realises that topology as the shortest legal geometry.

---

## 0. Status and honesty notes

- This is a design, not a tested result. Anything marked **[HYPOTHESIS]** is a design bet that must be validated by the experiments in section 17 before it is relied on.
- TopoR's internal algorithms are not public in anything I could verify. This design is built from general topological-routing ideas (homotopic routing, rubber-band sketches, negotiated congestion), not from TopoR's source. Do not describe it as a reimplementation.
- The riskiest piece is the slot-aware topological search (section 8). Milestone M2 is a deliberate spike to prove or kill it early.
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
Today (prototype, 12.3): Phases 1 to 3 route every connection on a single layer. Vias come afterwards, in a separate step (`complete`), and the repair after DRC goes through that step again. This ordering is the prototype's main fault (12.4, finding 1).

Planned (12.5): there is no separate step. The search itself may change layer, at a cost per via, and Phases 1 to 4 all use it.

---

## 11. Multi-pin nets

v1: decompose each net into 2-pin connections by a minimum spanning tree over pad positions, then route the connections as independent 2-pin items (each a separate wire id). This loses Steiner-tree optimality, so it is a known limitation. Branches start at pads only. Since realisation, wires of one net owe each other no spacing: where two of them leave a pad the same way they are drawn as one shared trace until they part, which gives the look and the copper of a branching trace without changing the topology (they are still two wires in the state, so capacity is counted conservatively). Later improvement: restart a net's routing as a tree grown from its existing copper, with start states on any gate slot adjacent to an existing branch. That requires splitting the wire at the branch point, so it is deferred.

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

**The map is mutable (M13, `topo/sites.py`).** A site appends 3 vertices, 9 edges and 5 triangles (the split triangle keeps its id); a flip reuses the edge and triangle ids it turns. The map's arrays grow by appending, the list mirrors and the per-half-edge transition rows are updated for the edges touched, `TopoState.grow` extends the state, and the compiled search's tables are built with spare room and updated in place. Every change to the map is logged with what is needed to undo it (`sites.log`, `rewind`, `replay`), and what a new site changes in the state is journalled (`sites.journal`, `undo`), so a site that does not fit is taken out without a trace and a snapshot restores the map it was taken on. Undoing works in reverse order of creation only; deleting an arbitrary site is not built (12.5, step 2). Things cached per gate that pass through a changed triangle (airwire paths, the candidate corridors of Phase 2) are not updated; they only feed cost estimates, and the code tolerates them being stale.

**Removing a via.** The site goes dormant: its keep-off returns to zero and the capacities are restored. The hole stays in the map, where it costs nothing, and can be made active again.

**The 2 µm hole realises as a correct via: go (M13).** This was the hypothesis the spike was for. What it took in `realize/`:
- the keep-off is one more term where relaxation sums the distance a wire keeps from a gate's end vertex (windows, and `radial` for the arcs);
- a wire crossing the gate *opposite* a via vertex is kept out of the keep-off disc on its own side of the via (`via_shadow`), for the edges that could not be flipped;
- the via's own trace: which of the three hole edges it leaves through is the search's accident, so its heading is judged a trace width away from the hole, a trace heading away from its edge goes round the nearer way, and it may take the innermost place inside foreign traces that wrap the via (`terminals.hop`). All three apply to via sites only; ordinary pads behave as before.

Results (details in the M13 row of section 18): 10,000 random operations with the invariant intact; hand-placed vias clean at 55 of 55 random positions, the smallest gap from a foreign trace to the via copper 0.307 mm against a rule of 0.300.

Known limits:
- A point within 20 µm of a triangle's edge cannot hold a site (5 of 60 random positions). The caller nudges the point.
- A via's own trace counts as load on the spokes it crosses, although it needs no keep-off from its own via. On a spoke shorter than the keep-off this under-states capacity.
- Capacity lowered by DRC feedback (`_penalise`) is overwritten when a site at that gate changes state or the gate is flipped.
- `free_space` does not know about vias; only the end-straightening shortcut in relaxation reads it, and DRC checks its result.

### 12.3 The prototype that decides where vias go (M14, built, to be reworked)

What is in the code today, on top of 12.2:
- **Split connections** (`plan/context.py`). A connection that takes vias is replaced by children, each an ordinary connection ending on a via site. `Context.snapshot` and `restore` carry the splits and take the maps back with them.
- **A search across layers** (`plan/vias.py` `_search`, `topo/kernel.py` `flood`). One flood per layer per via: each starts from every legal point the flood before could reach, at its cost so far plus `via_cost`. It returns the cheapest legal way through up to `max_vias` vias. On a test case its length equals the single-layer search's exactly. This part is sound.
- **All or nothing** (`vias.connect`). The sites are made, the pieces routed; if anything does not fit, everything is taken out again (journal and log, 12.2).
- **A separate step after rip-up** (`plan/ripup.py` `complete`). Rip-up runs without vias and stops early. `legalise` removes whatever over-fills a gate. Then each open connection gets a legal route, or vias, or room made by displacing the wires in its way; a displacement is kept only if fewer connections are open afterwards.
- **Racing variants** hand their sites back through the map log (12.2); the parent replays it.

Patches that accumulated around it, each added after one measurement on one board, and each a sign of a missing idea rather than a solution:

| Patch | What it stands in for |
|---|---|
| `spare`: a quarter pitch left free by late wires, growing each repair round | no margin in the capacity estimate |
| a via's own trace "added back" to its spokes' capacity; capacity raised while searching for it (`lift`) | capacity is one number per gate and cannot tell whose wire it is |
| three rules for how a trace leaves a via, half-pitch keep-off for a sleeping site, allowance for arc bulge | the via is a 2 µm hole standing in for a disc |
| `complete` with its displacement rule, blocker limit and memo | vias could not live inside rip-up |
| build a via, test it, undo it | the search does not know whether a via fits |
| the older between-passes via code, still present | two via systems |

### 12.4 What the prototype showed

Measured with one variant on one worker. All results have zero design-rule violations (violators are dropped at the end).

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

### 12.5 Plan for the rework

In this order. Each step is to be measured on `blinkSP1` before the next.

1. **Fix the pad-exit stub (finding 3). Done (M14a, 13.1 step 4).** It is independent of vias and affects every fine-pitch part. A trace may leave a pad only where the stub from the pad centre keeps its clearance: that gives each pad edge a legal window (possibly empty), computed once per map. An edge with an empty window is not a way out of the pad at all, so the search never uses it; relaxation keeps trace ends inside the window.
2. **Delete a via site from the map, anywhere, at any time.** Today a site can only be undone in reverse order of creation, which is the reason vias were kept out of rip-up. Deleting means flipping the site's spokes away until three are left and merging the triangles back, carrying the wires as `flip` already does. **[HYPOTHESIS]** This always succeeds or can be refused cleanly; to be shown by the same kind of random-operation test as M13.
3. **One search.** The multi-layer search becomes the router's only search: candidates (Phase 1), commit, rip-up and refinement all route connections that may change layer, with a via priced like any other cost. Layers and vias are then chosen with the whole board in view. Rip-up rips whole connections; their vias are deleted (step 2). `complete`, the displacement rule and the between-passes code are removed.
4. **Capacity with a margin**, decided from what is measured after steps 1 to 3, replacing `spare` and its growth.

What is kept as it is: `topo/sites.py` (12.2), the flood, the split-connection model, the map log for raced variants.

### 12.6 Not in scope
- Blind and buried vias: a site is on every layer.
- Vias in pads (`via_at_smd`).

---

## 13. Geometry realisation (topology to coordinates)

Input: per wire, the gate sequence and the per-gate ordering `gate_order`.

### 13.1 Relaxation (taut string with ordering)

1. **Initial placement.** On each gate with `k` wires ordered `w_1..w_k`, place wire `w_i`'s crossing point at fraction `(i - 0.5) / k` along the gate (gate endpoints already include the clearance inflation).
2. **Gauss-Seidel sweeps.** For each wire, for each interior crossing point, move it to the point on its gate that minimises path length (intersect the line between its two neighbours' points with the gate, clamp to the gate). Then clamp it between its ordered neighbours on the same gate: at least `(t + s)` away from the previous wire's point and from the next wire's point. Repeat until the largest movement in a sweep is below `1e-4 mm` or a sweep limit is hit.
3. **Bends.** A point clamped to a gate endpoint means the wire bends around that obstacle vertex. Because obstacles are already inflated by `s + t/2`, a wire touching the inflated vertex is DRC-legal. Use a mitred/rounded inflation with enough resolution to avoid clearance loss at convex corners.
4. **Terminals.** The wire's first and last points lie on the inflated pad boundary. A short straight stub joins that point to the pad centre. The ring guarantees only that the point itself is clear of foreign copper. Where the rings of neighbouring pads have merged (fine-pitch parts), a stub from the part of the ring over the gap cuts across towards the neighbour. So each pad edge has a window, the part of it from which the stub keeps its clearance, found once per map by trying 17 points along the edge (`geom/exits.py`). An edge with no such part is made a wall, so the search never leaves a pad through it; relaxation keeps a trace's end inside the window. A pad with no legal edge at all cannot be reached on that layer and is reported with the other unreachable pads.

Vectorise the sweep with NumPy (all wires processed per gate-index class), or use `numba` for this kernel only.

### 13.2 DRC and repair

- Check with `shapely`: buffer every trace by `t/2`, query an `STRtree` of foreign-net copper for distance `< s`; check against the outline; check wire-to-wire distances.
- If violations remain after relaxation: increase spacing locally and re-relax; if still violating, mark the gates involved, add history cost, and send the affected wires back to Phase 3. Capacity is only an estimate, so this feedback loop is expected.
- Via copper is checked as a pad of its net on every layer. Via to via and via to fixed copper are kept apart when the via is placed, not checked afterwards.
- The stub from a pad centre to the ring used to be assumed clear (12.4, finding 3). Fixed in 13.1 step 4.
- **Known limit (12.4, finding 4):** widening spacing only helps where the gate has room. The loop now stops as soon as a round is no better, and no longer gives up on a whole layer because one violation there is of another kind.

### 13.3 Sliding vias (planned, M15)

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

**Status (M9, measured by `python -m bench.perf`, results in `bench/results.md`):** all three targets are met. What got it there, in order of effect:
1. The search loop and the relaxation sweep are compiled with `numba` (`topo/kernel.py`, `realize/kernel.py`). `numba` is optional: without it the same loops run in Python and give the same routing, only slower. Profiling had shown more than 95 % of the time in these two loops.
2. Each wire starts relaxation from its exact taut path (string pulling), so the sweep only settles the places where wires press on each other.
3. The clearance solve along a wall is cached per map.

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
| M14b | Deleting a via site (12.5 step 2) | 10,000 random operations including deletions keep the invariant and the map's tables; a map with every site deleted equals the map before any was made, up to edge flips |
| M14c | One search across layers for every phase (12.5 step 3) | blinkSP1: at least 51 of 59 with no more than about 40 vias and zero violations; `complete` and the between-passes code deleted |
| M14d | Capacity margin (12.5 step 4); then RAM Selector Tree, ALU, Word of RAM, ulx3s | The M14 criterion above |
| M15 | Sliding vias (13.3) | Total length on RAM Selector Tree and ALU drops against M14 with the check still clean; added geometry time recorded |
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
| `via_cost` | 4 x median pad pitch | the detour, in mm, a via is worth. Prototype value: with 10 x (t + s) connections took four vias where one would do; still about three per connection (12.4) |
| `max_vias` | 4 | vias one connection may take |
| `spare` | 0.25 pitch, +0.25 per repair round | room late wires leave on a gate. A patch (12.3), to be replaced (12.5 step 4) |
| Site hole radius | 2 µm | size of a site in the map (12.2). Internal: `sites.SITE_RADIUS`. Not the via's drill, which is a rule (0.3 mm unless the DSN or the settings say otherwise) |
| Via slide rounds / tolerance | 8 / 1 µm | stop condition of 13.3 |
| `max_via_rounds` | 4 | only with vias between passes (`live_vias` off); to be removed with that code |
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

**Open**: the window has only been exercised through the self-test, not by hand; Windows and Linux builds are untried (without fork the router runs single-process there).
