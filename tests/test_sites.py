"""Via sites created inside the map (design section 12.2; milestone M13)."""
import copy
import math
import random

import numpy as np
import pytest
from shapely.geometry import LineString, Point

from tests.conftest import grid_board
from weaveengine.board import Board, Pad, Rules
from weaveengine.plan.path import locate
from weaveengine.realize.relax import realize
from weaveengine.realize.terminals import straighten
from weaveengine.topo import kernel, planar_map, sites
from weaveengine.topo.planar_map import GATE, TERMINAL
from weaveengine.topo.search import route
from weaveengine.topo.state import TopoState


def map_errors(pmap) -> list[str]:
    """Checks the map's tables against each other after it has been changed."""
    errors = []
    pmap.catch_up()
    idle_edges = {e for slot in sites.free_slots(pmap) for e in slot[1]}
    idle_tris = {t for slot in sites.free_slots(pmap) for t in slot[2]}
    for t in range(pmap.num_triangles):
        if t in idle_tris:
            continue
        verts, edges = pmap.tri_v_list[t], pmap.tri_e_list[t]
        (ax, ay), (bx, by), (cx, cy) = (pmap.vxy[v] for v in verts)
        if (bx - ax) * (cy - ay) - (by - ay) * (cx - ax) <= 0:
            errors.append(f"triangle {t} is not counter-clockwise")
        if tuple(pmap.tri_v[t]) != verts or tuple(pmap.tri_e[t]) != edges:
            errors.append(f"triangle {t}: array and list differ")
        for k in range(3):
            e, n = edges[k], int(pmap.tri_n[t][k])
            if set(pmap.edge_v_list[e]) != {verts[(k + 1) % 3], verts[(k + 2) % 3]}:
                errors.append(f"triangle {t}: edge {e} is not opposite its vertex {k}")
            if t not in pmap.edge_t_list[e] or n not in pmap.edge_t_list[e] or (n >= 0 and t not in pmap.tri_n[n]):
                errors.append(f"triangle {t}: edge {e} and neighbour {n} disagree")
    for e in range(pmap.num_edges):
        if e in idle_edges:
            if pmap.edge_kind_list[e] == GATE or pmap.trans[2 * e] or pmap.trans[2 * e + 1]:
                errors.append(f"edge {e} of an unused slot can be crossed")
            continue
        u, v = pmap.edge_v_list[e]
        if abs(pmap.edge_len_list[e] - math.dist(pmap.vxy[u], pmap.vxy[v])) > 1e-9 or u >= v:
            errors.append(f"edge {e}: stale length or endpoints out of order")
        if (pmap.edge_t_list[e][1] < 0) == (pmap.edge_kind_list[e] == GATE):
            errors.append(f"edge {e}: a gate has two triangles, anything else one")
    if pmap.trans != [pmap.transitions(h) for h in range(2 * pmap.num_edges)]:
        errors.append("stale transitions")
    tables = pmap.__dict__.get("_kernel_tables")
    if tables is not None:
        fresh = kernel.Tables(pmap)
        size = 2 * pmap.num_edges
        for name in ("tr", "length"):
            mine, theirs = getattr(tables, name)[:size], getattr(fresh, name)[:size]
            used = np.arange(2)[None, :] < fresh.n[:size, None]
            if not np.array_equal(tables.n[:size], fresh.n[:size]) or not np.array_equal(mine[used], theirs[used]):
                errors.append(f"stale kernel table {name}")
    return errors


def test_site_carries_the_wires_of_its_triangle(grid):
    board, pmap = grid
    state = TopoState(pmap)
    for wire, (a, b) in enumerate([(0, 24), (4, 20), (2, 22), (10, 14)], 1):
        state.insert(wire, route(pmap, state, a, b).steps)
    before = {w: [s[0] for s in steps] for w, steps in state.wire_path.items()}
    old_edges = pmap.num_edges
    t = max(range(pmap.num_triangles), key=lambda t: sum(state.corner_cnt[t]))  # the busiest triangle
    site = sites.create(pmap, state, t, sites.incentre(pmap, t), pad=99)
    assert state.invariant_errors() == [] and map_errors(pmap) == []
    assert not site.active and pmap.pad_net[99] == -1 and len(pmap.pad_edges[99]) == 3
    assert len(site.spokes) >= 6 and pmap.num_edges == old_edges + 9
    # Every wire still crosses the gates it crossed, in the same order, apart
    # from gates that were turned round the site; and it now crosses spokes.
    turned = {e for e in site.spokes if e < old_edges}
    for w, gates in before.items():
        now = [s[0] for s in state.wire_path[w]]
        assert [g for g in now if g < old_edges and g not in turned] == [g for g in gates if g not in turned]
    assert any(state.gate_order[e] for e in site.spokes)
    # The hole's edges are terminal edges of the site's own pad id, so they are
    # walls to every search that is not asked for that pad; asked for it, the search gets there.
    assert all(pmap.edge_owner_list[e] == 99 and pmap.edge_kind_list[e] == TERMINAL for e in site.hole)
    assert route(pmap, state, 0, 99) is not None
    with pytest.raises(ValueError):
        sites.create(pmap, state, site.tris[3], pmap.tri_cen[site.tris[3]])  # a sliver beside the hole has no room


def test_waking_a_site_takes_its_keep_off_from_the_spokes(grid):
    board, pmap = grid
    state = TopoState(pmap)
    t = max(range(pmap.num_triangles), key=lambda t: sites.room(pmap, t, sites.incentre(pmap, t)))
    site = sites.create(pmap, state, t, sites.incentre(pmap, t))
    asleep = [state.cap[e] for e in site.spokes]
    keep = sites.keep_off(board.rules)
    sites.set_net(pmap, state, site, 3, keep)
    assert site.active and pmap.pad_net[site.pad] == 3
    # Asleep it already keeps half a pitch (wires pass it on both sides); awake, the via's keep-off.
    assert all(abs(a - state.cap[e] - (keep - sites.asleep_keep(pmap)) / pmap.pitch) < 1e-9 for a, e in zip(asleep, site.spokes))
    assert sites.fits(pmap, state, site, keep) and not sites.fits(pmap, state, site, 1e3)
    r = route(pmap, state, 0, site.pad)
    state.insert(1, r.steps)
    with pytest.raises(ValueError):
        sites.set_net(pmap, state, site, -1)           # a via with a trace on it stays
    state.remove(1)
    sites.set_net(pmap, state, site, -1)
    assert [state.cap[e] for e in site.spokes] == asleep and sites.via_pads(pmap, board.rules) == []


def test_a_trace_against_a_via_of_its_net_takes_no_room_there(grid):
    """The via keeps its room clear for foreign traces; a trace of its own net
    that lies against it is the via's copper (design section 11)."""
    board, pmap = grid
    state = TopoState(pmap)
    t = max(range(pmap.num_triangles), key=lambda t: sites.room(pmap, t, sites.incentre(pmap, t)))
    site = sites.create(pmap, state, t, sites.incentre(pmap, t))
    sites.set_net(pmap, state, site, 3, sites.keep_off(board.rules))
    r = route(pmap, state, 0, site.pad, net=3)
    state.insert(1, r.steps, net=3)
    spokes = [e for e in r.gates if e in site.spokes]
    assert spokes and all(state.load[e] == 0.0 for e in spokes)
    assert all(state.load[e] == 1.0 for e in r.gates[1:-1] if e not in site.spokes)   # everywhere else it counts
    # The place against the via is one beside the net's copper, for the next wire of the net.
    beside = state.beside(3)
    assert all(beside[e] for e in site.spokes)
    # A second trace of the net comes to the via as well. Were they of another net, they would count.
    state.insert(2, route(pmap, state, 1, site.pad, net=3).steps, net=3)
    mine = [e for e in site.spokes if state.gate_order[e]]
    assert all(state.load[e] == 0.0 == state.tally(e) for e in mine)
    state.net[1] = state.net[2] = 5
    assert all(state.tally(e) == 1.0 for e in mine)
    state.net[1] = state.net[2] = 3
    state.remove(2), state.remove(1)
    sites.set_net(pmap, state, site, -1)
    assert all(state.load[e] == 0.0 for e in site.spokes) and state.check_invariants()


def test_random_operations_keep_the_invariants():
    """M13 acceptance: 10,000 random inserts, removes, new sites, and sites woken
    and put to sleep; the invariant of 7.2 after every one, the map's own tables
    checked regularly, and no crossing in the realised geometry at the end."""
    board = grid_board(6, 10.0, seed=3)
    pmap = planar_map.build(board)
    first_edges = pmap.num_edges
    state = TopoState(pmap)
    rng = random.Random(11)
    keep = sites.keep_off(board.rules)
    pads = sorted(pmap.pad_edges)
    live: dict[int, tuple[int, int]] = {}
    made: list = []
    wire = ops = flips = carried = 0
    count = {"insert": 0, "remove": 0, "site": 0, "wake": 0, "sleep": 0, "delete": 0, "delete refused": 0, "move": 0, "move refused": 0}
    peak_edges = 0
    while ops < 10_000:
        roll = rng.random()
        if roll < 0.40 or not live:
            ends = pads + [s.pad for s in made if s.active]
            a, b = rng.sample(ends, 2)
            # A trace that ends on a via is of the via's net: against the via it takes no room (``TopoState.tally``).
            nets = {pmap.pad_net[x] for x in (a, b) if x in pmap.sites}
            if len(nets) > 1:
                continue
            net = nets.pop() if nets else -1
            r = route(pmap, state, a, b, hard_cap=True, net=net)
            if r is None:
                continue
            wire += 1
            state.insert(wire, r.steps, net=net)
            live[wire] = (a, b)
            count["insert"] += 1
        elif roll < 0.78:
            state.remove(rng.choice(sorted(live)))
            live = {w: ends for w, ends in live.items() if w in state.wire_path}
            count["remove"] += 1
        elif roll < 0.80 and made:
            # Delete a sleeping site, whichever and whenever it was made (M14b).
            asleep = [s for s in made if not s.active]
            if not asleep:
                continue
            site = rng.choice(asleep)
            before = (fingerprint(pmap), state.snapshot()) if count["delete refused"] < 3 else None
            if sites.delete(pmap, state, site):
                made.remove(site)
                assert site.pad not in pmap.sites and site.pad not in pmap.pad_edges
                count["delete"] += 1
            else:
                count["delete refused"] += 1
                if before is not None:   # a refusal changes nothing
                    assert fingerprint(pmap) == before[0] and state.snapshot()[2] == before[1][2]
        elif roll < 0.84 and made:
            # Move a site a little, awake or asleep, with whatever wires pass it (13.3).
            site = rng.choice(made)
            reach = min(pmap.edge_len_list[e] for e in site.spokes)
            angle = rng.random() * 2 * math.pi
            step = reach * rng.choice((0.02, 0.1, 0.3, 0.8))
            point = (site.centre[0] + step * math.cos(angle), site.centre[1] + step * math.sin(angle))
            before = (fingerprint(pmap), state.snapshot(), state.cap.copy(), site.centre) if count["move refused"] < 40 else None
            orders = [list(row) for row in state.gate_order]
            if sites.move(pmap, state, site, point):
                assert site.centre == point and pmap.pad_centre[site.pad] == point
                assert not any(state.load[e] > state.cap[e] + 1e-9 for e in site.spokes)
                count["move"] += 1
            else:
                count["move refused"] += 1
                if before is not None:   # a refusal changes nothing
                    assert fingerprint(pmap) == before[0] and state.snapshot()[2] == before[1][2] and site.centre == before[3]
                    assert np.array_equal(state.cap, before[2])
            assert [list(row) for row in state.gate_order] == orders   # no wire's place on any gate changes
        elif roll < 0.88 and len(made) < 60:
            t = rng.randrange(pmap.num_triangles)
            (ax, ay), (bx, by), (cx, cy) = (pmap.vxy[v] for v in pmap.tri_v_list[t])
            u, v = sorted((rng.random(), rng.random()))
            point = (u * ax + (v - u) * bx + (1 - v) * cx, u * ay + (v - u) * by + (1 - v) * cy)
            edges = pmap.num_edges
            try:
                site = sites.create(pmap, state, t, point)
            except ValueError:
                continue
            made.append(site)
            flips += len(site.spokes) - 6
            carried += sum(len(state.gate_order[e]) for e in site.spokes)
            assert pmap.num_edges in (edges, edges + 9)      # a freed slot is used again before the tables grow
            peak_edges = max(peak_edges, pmap.num_edges)
            count["site"] += 1
        elif made:
            site = rng.choice(made)
            used = {pad for ends in live.values() for pad in ends}
            if site.active and site.pad not in used:
                sites.set_net(pmap, state, site, -1)
                count["sleep"] += 1
            elif not site.active and sites.fits(pmap, state, site, keep):
                sites.set_net(pmap, state, site, 1000 + site.pad, keep)
                count["wake"] += 1
            else:
                continue
        else:
            continue
        ops += 1
        assert state.invariant_errors() == [], f"after {ops} operations"
        if ops % 250 == 0:
            assert map_errors(pmap) == [], f"after {ops} operations"
    assert map_errors(pmap) == [] and not state.overflowed_gates()
    for site in made:  # every gate at a hole vertex is listed as a spoke of its site, and nothing else is
        at_hole = {e for e in range(pmap.num_edges) if pmap.edge_kind_list[e] == GATE and set(pmap.edge_v_list[e]) & set(site.verts)}
        assert set(site.spokes) == at_hole and len(site.spokes) == len(at_hole)
    assert min(v for k, v in count.items() if not k.endswith("refused")) > 50 and flips > 100 and carried > 50, (count, flips, carried)
    assert count["move refused"] > 20, count
    assert count["delete refused"] <= count["delete"] // 20, count
    assert pmap.num_edges == peak_edges <= first_edges + 9 * 62      # slots are reused: no growth without end
    print("random operations:", count, "flips at creation", flips)
    # Independent crossing oracle (17.1) on a well-filled board.
    ends = pads + [s.pad for s in made if s.active]
    for _ in range(400):
        if len(state.wire_path) >= 30:
            break
        a, b = rng.sample(ends, 2)
        r = route(pmap, state, a, b, hard_cap=True)
        if r is not None:
            wire += 1
            state.insert(wire, r.steps)
    assert state.invariant_errors() == []
    with_vias = copy.copy(board)
    with_vias.pads = board.pads + sites.via_pads(pmap, board.rules)
    lines, violations, _ = realize(state, with_vias)
    # Wires ending on one pad are one net on a real board, free to merge on
    # the way in. Here their nets are arbitrary, so such a pair is no witness.
    pad_of = {w: {pmap.edge_owner_list[p[0][0]], pmap.edge_owner_list[p[-1][0]]} for w, p in state.wire_path.items()}
    crossed = [v for v in violations if v.kind == "crossing" and not pad_of[v.wires[0]] & pad_of[v.wires[1]]]
    assert len(lines) >= 20 and not crossed


def two_layer_case(via_at: tuple[float, float], wraps: int = 2):
    """A net that must change layers at ``via_at``, and on each layer ``wraps``
    foreign traces whose straight path the via's own trace cuts, so they have
    to go round the via."""
    board = Board.rectangle(40.0, 30.0, Rules(), layers=["F.Cu", "B.Cu"])
    board.pads += [Pad.rect(0, 5, 15, 1.5, 1.5, 1, layers=frozenset({0})), Pad.rect(1, 35, 15, 1.5, 1.5, 1, layers=frozenset({1}))]
    pad_id, plan = 2, []
    for i in range(wraps):
        for layer, x in ((0, 19.0 - 1.5 * i), (1, 21.0 + 1.5 * i)):
            board.pads += [Pad.circle(pad_id, x, 4, 0.6, pad_id), Pad.circle(pad_id + 1, x, 26, 0.6, pad_id)]
            plan.append((layer, pad_id, pad_id + 1))
            pad_id += 2
    maps = [planar_map.build(board, i) for i in range(2)]
    states = [TopoState(m) for m in maps]
    via = 1000
    for pmap, state in zip(maps, states):
        t = int(locate(pmap, np.array([via_at]))[0])
        site = sites.create(pmap, state, t, via_at, pad=via)
        sites.set_net(pmap, state, site, 1, sites.keep_off(board.rules))
    for wire, (layer, a, b) in enumerate([(0, 0, via), (1, via, 1)] + plan, 1):
        r = route(maps[layer], states[layer], a, b, hard_cap=True)
        assert r is not None
        states[layer].insert(wire, r.steps)
    return board, maps, states


@pytest.mark.parametrize("via_at", [(20.3, 15.2), (20.1, 9.0), (19.3, 20.4), (18.2, 12.6), (21.7, 17.3)])
def test_hand_placed_via_realises_clean(via_at):
    """M13 acceptance: a via put in by hand realises with no design-rule
    violation and no crossing, on both layers, with foreign traces going round it."""
    board, maps, states = two_layer_case(via_at)
    rules = board.rules
    copper = Point(via_at).buffer(rules.via_diameter / 2.0, quad_segs=64)   # the true circle, not the checker's polygon
    for pmap, state in zip(maps, states):
        with_via = copy.copy(board)
        with_via.pads = board.pads + sites.via_pads(pmap, rules)
        assert [p.centre for p in with_via.pads if p.is_via] == [pytest.approx(via_at)]
        straighten(state, with_via)
        assert state.invariant_errors() == [] and map_errors(pmap) == []
        lines, violations, wire_net = realize(state, with_via)
        assert violations == []
        site = pmap.sites[1000]
        own = [w for w in lines if wire_net[w] == 1]
        foreign = [w for w in lines if wire_net[w] != 1]
        assert len(own) == 1 and min(math.dist(lines[own[0]][0], via_at), math.dist(lines[own[0]][-1], via_at)) < 1e-9
        # Foreign traces do pass the via's spokes, and keep their clearance from its copper.
        assert any(wire_net[w] != 1 for e in site.spokes for w in state.gate_order[e])
        gap = min(LineString(lines[w]).distance(copper) for w in foreign)
        assert gap >= rules.clearance + rules.trace_width / 2.0 - 1e-3
        assert gap < rules.clearance + rules.trace_width / 2.0 + 0.05   # and it is the via they are bending round


def fingerprint(pmap):
    """Everything in the map a site changes, in a form that can be compared.
    Slots that hold no site are left out (the tables never shrink)."""
    pmap.catch_up()
    idle_edges = {e for slot in sites.free_slots(pmap) for e in slot[1]}
    idle_tris = {t for slot in sites.free_slots(pmap) for t in slot[2]}
    tris = [t for t in range(pmap.num_triangles) if t not in idle_tris]
    edges = [e for e in range(pmap.num_edges) if e not in idle_edges]
    return ([(t, pmap.tri_v_list[t], pmap.tri_e_list[t], tuple(pmap.tri_n[t].tolist())) for t in tris],
            [(e, pmap.edge_v_list[e], pmap.edge_t_list[e], pmap.edge_kind_list[e], round(pmap.edge_len_list[e], 9),
              pmap.trans[2 * e], pmap.trans[2 * e + 1]) for e in edges],
            sorted(pmap.pad_edges), sorted(pmap.sites), {pad: sorted(site.spokes) for pad, site in pmap.sites.items()})


def busy_state(pmap, pairs):
    state = TopoState(pmap)
    for wire, (a, b) in enumerate(pairs, 1):
        r = route(pmap, state, a, b, hard_cap=True)
        if r is not None:
            state.insert(wire, r.steps)
    return state


PAIRS = [(0, 35), (5, 30), (2, 33), (12, 17), (6, 29), (1, 34), (18, 23), (8, 27)]


def roomy_triangles(pmap, count, seed=0):
    rng = random.Random(seed)
    order = list(range(pmap.num_triangles))
    rng.shuffle(order)
    return [t for t in order if sites.room(pmap, t, sites.incentre(pmap, t)) > 0.3][:count]


def test_a_site_can_be_taken_out_again():
    """A site that turns out not to fit is undone: the map and every wire exactly as before (12.4)."""
    board = grid_board(6, 10.0, seed=3)
    pmap = planar_map.build(board)
    state = busy_state(pmap, PAIRS)
    route(pmap, state, 0, 1)   # the compiled search's tables exist and must follow
    before_map, before_state = fingerprint(pmap), state.snapshot()
    caps = state.cap.copy()
    mark = (len(sites.log(pmap)), len(sites.journal(state)))
    made = []
    for t in roomy_triangles(pmap, 12):
        if t < pmap.num_triangles and sites.room(pmap, t, sites.incentre(pmap, t)) > 0.3:
            made.append(sites.create(pmap, state, t, sites.incentre(pmap, t)))
            sites.set_net(pmap, state, made[-1], 500 + len(made), sites.keep_off(board.rules) if sites.fits(pmap, state, made[-1], 0.5) else 0.0)
    assert len(made) >= 8 and sum(len(s.spokes) for s in made) > 6 * len(made)        # flips happened
    assert any(state.gate_order[e] for s in made for e in s.spokes)                    # and wires were carried
    assert fingerprint(pmap) != before_map
    moved = sites.undo(pmap, state, *mark)
    assert moved and fingerprint(pmap) == before_map and map_errors(pmap) == []
    after = state.snapshot()
    edges, tris = len(before_state[0]), len(before_state[1])     # the tables have kept the room they gained, empty
    assert after[0][:edges] == before_state[0] and not any(after[0][edges:])
    assert after[1][:tris] == before_state[1] and not any(any(c) for c in after[1][tris:])
    assert after[2] == before_state[2]
    assert np.allclose(after[3][:edges], before_state[3]) and np.allclose(state.cap[:edges], caps)
    assert state.invariant_errors() == []
    assert route(pmap, state, 3, 32) is not None                                       # and the search still works on it


def test_the_log_rebuilds_the_same_map_elsewhere():
    """A raced variant's sites are made again on the parent's maps from its log (12.5)."""
    board = grid_board(6, 10.0, seed=3)
    theirs, mine = planar_map.build(board), planar_map.build(board)
    state = busy_state(theirs, PAIRS)
    for t in roomy_triangles(theirs, 10, seed=4):
        if t < theirs.num_triangles and sites.room(theirs, t, sites.incentre(theirs, t)) > 0.3:
            sites.create(theirs, state, t, sites.incentre(theirs, t))
    snap = state.snapshot()
    log = sites.log(theirs)
    assert sum(1 for entry in log if entry["kind"] == "flip") > 10
    sites.replay(mine, log)
    assert fingerprint(mine) == fingerprint(theirs) and map_errors(mine) == []
    copy_state = TopoState(mine)
    copy_state.restore(snap)
    assert copy_state.invariant_errors() == []
    # Rewinding part of the way leaves the map as it was after that many entries.
    half = len(log) // 2
    third = planar_map.build(board)
    sites.replay(third, log[:half])
    sites.rewind(mine, half)
    assert fingerprint(mine) == fingerprint(third)


def test_a_search_from_a_via_point_matches_the_search_from_the_pad():
    """12.3: a search reports what it reached on the way, and another search can
    start from such a point, as from a via. The way back costs what the way there did."""
    board = grid_board(6, 10.0, seed=3)
    pmap = planar_map.build(board)
    state = busy_state(pmap, PAIRS)
    r, seen = route(pmap, state, 7, 28, hard_cap=True, reach=True)
    assert r is not None and r.seed == -1 and np.isfinite(seen.best).sum() > 20
    # The way to a triangle the search passed: an insertable route whose last step crosses into it.
    t = int(np.argmax(np.where(np.isfinite(seen.best), seen.best, -1.0)))
    there = seen.route_to(t)
    assert there is not None and there.cost == pytest.approx(seen.best[t])
    last_edge = there.steps[-1][0]
    assert last_edge in pmap.tri_e_list[t] and there.steps[-1][1] != t
    # From a seed in the middle of that triangle, back to the pad.
    x, y = pmap.tri_cen[t]
    back = route(pmap, state, None, 7, hard_cap=True, seeds=(np.array([t]), np.array([2.5]), np.array([x]), np.array([y])))
    assert back is not None and back.seed == 0
    assert back.cost == pytest.approx(seen.best[t] + 2.5, abs=0.75 * max(pmap.edge_len_list[e] for e in pmap.tri_e_list[t]))
    # ``bound`` cuts the search short: nothing dearer than it is reported.
    assert route(pmap, state, 7, 28, hard_cap=True, bound=r.cost - 1.0) is None
    assert route(pmap, state, 7, 28, hard_cap=True, bound=r.cost + 1.0).cost == pytest.approx(r.cost)


def test_deleting_every_site_gives_the_map_back():
    """M14b: sites are deleted in an order that has nothing to do with the order
    they were made in, with wires routed among them. What is left is the map
    there was before any site, up to which diagonals its quadrilaterals have."""
    board = grid_board(6, 10.0, seed=3)
    pmap = planar_map.build(board)
    before = (pmap.num_vertices, pmap.num_edges, pmap.num_triangles, sorted(pmap.pad_edges),
              sorted(e for e, k in enumerate(pmap.edge_kind_list) if k != GATE))
    state = busy_state(pmap, PAIRS)
    made = []
    for t in roomy_triangles(pmap, 14, seed=2):
        if sites.room(pmap, t, sites.incentre(pmap, t)) > 0.3 and t not in {x for s in sites.free_slots(pmap) for x in s[2]}:
            made.append(sites.create(pmap, state, t, sites.incentre(pmap, t)))
    wire = 100
    for a, b in [(7, 28), (3, 32), (10, 25), (14, 21), (9, 26), (4, 31)]:   # more wires, now passing the sites
        r = route(pmap, state, a, b, hard_cap=True)
        if r is not None:
            wire += 1
            state.insert(wire, r.steps)
    assert any(state.gate_order[e] for s in made for e in s.spokes)
    random.Random(5).shuffle(made)
    refused = [s for s in made if not sites.delete(pmap, state, s)]
    assert not refused
    assert state.invariant_errors() == [] and map_errors(pmap) == []
    idle = sites.free_slots(pmap)
    assert len(idle) == len(made) and not pmap.sites
    live = (pmap.num_vertices - 3 * len(idle), pmap.num_edges - 9 * len(idle), pmap.num_triangles - 5 * len(idle))
    assert live == before[:3] and sorted(pmap.pad_edges) == before[3]
    walls = sorted(e for e, k in enumerate(pmap.edge_kind_list) if k != GATE and e < before[1])
    assert walls == before[4]                                     # no wall or pad edge was touched
    # Every wire can be taken out and the board routed again on the map that is left.
    for w in list(state.wire_path):
        state.remove(w)
    assert all(not o for o in state.gate_order) and all(c == [0, 0, 0] for c in state.corner_cnt)
    again = busy_state(pmap, PAIRS)
    assert len(again.wire_path) == len(PAIRS) and again.invariant_errors() == []


def test_a_deletion_is_logged_like_everything_else():
    """Deletions rewind and replay with the rest of the map's log (12.2), so snapshots and raced variants still work."""
    board = grid_board(6, 10.0, seed=3)
    theirs, mine = planar_map.build(board), planar_map.build(board)
    state = busy_state(theirs, PAIRS)
    made = [sites.create(theirs, state, t, sites.incentre(theirs, t)) for t in roomy_triangles(theirs, 6, seed=9)]
    middle = (len(sites.log(theirs)), fingerprint(theirs), state.snapshot())
    assert sites.delete(theirs, state, made[1]) and sites.delete(theirs, state, made[4])
    t = next(t for t in roomy_triangles(theirs, 40, seed=1) if t not in {x for s in sites.free_slots(theirs) for x in s[2]})
    reused = sites.create(theirs, state, t, sites.incentre(theirs, t))
    assert set(reused.verts) in ({*made[1].verts}, {*made[4].verts})        # it took a freed slot
    assert state.invariant_errors() == [] and map_errors(theirs) == []
    sites.replay(mine, sites.log(theirs))
    assert fingerprint(mine) == fingerprint(theirs) and map_errors(mine) == []
    sites.rewind(theirs, middle[0])
    assert fingerprint(theirs) == middle[1] and map_errors(theirs) == []
    state.restore(middle[2])
    state.resize()
    sites.clear_free(theirs, state)
    assert state.invariant_errors() == []



def test_a_move_is_logged_like_everything_else():
    """A site moved (13.3) rewinds exactly and replays on another copy of the map, wires and capacities with it."""
    board = grid_board(6, 10.0, seed=3)
    theirs, mine = planar_map.build(board), planar_map.build(board)
    state = busy_state(theirs, PAIRS)
    route(theirs, state, 0, 1)   # the compiled search's tables exist and must follow
    made = [sites.create(theirs, state, t, sites.incentre(theirs, t)) for t in roomy_triangles(theirs, 8, seed=5)]
    for n, site in enumerate(made):
        if sites.fits(theirs, state, site, sites.keep_off(board.rules)):
            sites.set_net(theirs, state, site, 700 + n, sites.keep_off(board.rules))
    assert any(state.gate_order[e] for s in made for e in s.spokes)      # wires pass the sites that move
    middle = (len(sites.log(theirs)), len(sites.journal(state)), fingerprint(theirs), state.snapshot(), state.cap.copy(),
              theirs.edge_width.copy(), [s.centre for s in made])
    rng = random.Random(2)
    moved = 0
    for _ in range(6):
        for site in made:
            reach = min(theirs.edge_len_list[e] for e in site.spokes)
            angle = rng.random() * 2 * math.pi
            moved += sites.move(theirs, state, site, (site.centre[0] + 0.2 * reach * math.cos(angle), site.centre[1] + 0.2 * reach * math.sin(angle)))
    assert moved > 20 and [s.centre for s in made] != middle[6]
    assert state.invariant_errors() == [] and map_errors(theirs) == [] and not state.overflowed_gates()
    assert state.snapshot()[2] == middle[3][2]                           # no wire's path has changed
    assert route(theirs, state, 3, 32) is not None
    # Done again from the log on another copy: the same map.
    sites.replay(mine, sites.log(theirs))
    assert fingerprint(mine) == fingerprint(theirs) and map_errors(mine) == []
    assert np.allclose(mine.edge_width[:mine.num_edges], theirs.edge_width[:theirs.num_edges])
    assert [mine.sites[s.pad].centre for s in made] == [s.centre for s in made]
    # Undone: the map, the widths and the capacities exactly as they were.
    sites.undo(theirs, state, middle[0], middle[1])
    assert fingerprint(theirs) == middle[2] and map_errors(theirs) == [] and [s.centre for s in made] == middle[6]
    assert np.array_equal(theirs.edge_width[:len(middle[5])], middle[5]) and np.array_equal(state.cap[:len(middle[4])], middle[4])
    assert state.snapshot()[2] == middle[3][2] and state.invariant_errors() == []


def test_a_move_is_refused_when_it_would_turn_a_triangle_or_fill_a_gate():
    board = grid_board(6, 10.0, seed=3)
    pmap = planar_map.build(board)
    state = busy_state(pmap, PAIRS)
    site = sites.create(pmap, state, *next((t, sites.incentre(pmap, t)) for t in roomy_triangles(pmap, 3, seed=5)))
    before = fingerprint(pmap), state.snapshot(), state.cap.copy()
    far = max(pmap.edge_len_list[e] for e in site.spokes)
    assert not sites.move(pmap, state, site, (site.centre[0] + 3 * far, site.centre[1]))     # out of its triangles
    assert fingerprint(pmap) == before[0] and state.snapshot()[2] == before[1][2] and np.array_equal(state.cap, before[2])
    assert sites.move(pmap, state, site, (site.centre[0] + 0.05 * far, site.centre[1]), sure=True)
    assert map_errors(pmap) == [] and state.invariant_errors() == []


@pytest.mark.parametrize("via_at, to", [((20.3, 15.2), (20.9, 15.6)), ((20.1, 9.0), (19.6, 9.5)), ((19.3, 20.4), (19.9, 19.9))])
def test_a_moved_via_realises_clean(via_at, to):
    """M15: a via moved in the map realises with no violation, its own traces ending on it and the others still going round it."""
    board, maps, states = two_layer_case(via_at)
    rules = board.rules
    copper = Point(to).buffer(rules.via_diameter / 2.0, quad_segs=64)
    for pmap, state in zip(maps, states):
        site = pmap.sites[1000]
        while math.dist(site.centre, to) > 1e-9:   # in steps: a move is refused if it takes too much of a triangle at once
            reach = math.dist(site.centre, to)
            part = next((p for p in (1.0, 0.5, 0.25, 0.1, 0.03) if sites.move(pmap, state, site, tuple(
                c + p * (t - c) for c, t in zip(site.centre, to)) if p < 1.0 else to)), None)
            assert part is not None, f"stuck {reach:.2f} mm from where it was to go"
        with_via = copy.copy(board)
        with_via.pads = board.pads + sites.via_pads(pmap, rules)
        assert [p.centre for p in with_via.pads if p.is_via] == [pytest.approx(to)]
        straighten(state, with_via)
        assert state.invariant_errors() == [] and map_errors(pmap) == []
        lines, violations, wire_net = realize(state, with_via)
        assert violations == []
        own = [w for w in lines if wire_net[w] == 1]
        assert len(own) == 1 and min(math.dist(lines[own[0]][0], to), math.dist(lines[own[0]][-1], to)) < 1e-9
        gap = min(LineString(lines[w]).distance(copper) for w in lines if wire_net[w] != 1)
        assert gap >= rules.clearance + rules.trace_width / 2.0 - 1e-3

