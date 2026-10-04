import random

from shapely.geometry import LineString

from weaveengine.topo.barrier import Barrier
from weaveengine.topo.planar_map import TERMINAL
from weaveengine.topo.runs import airwire_path, cross_count, path_from_steps
from weaveengine.topo.search import route
from weaveengine.topo.state import TopoState


def test_cross_count_matches_geometry(grid):
    """17.1: topological crossing count against geometric crossings in free space."""
    _, pmap = grid
    state = TopoState(pmap)
    rng = random.Random(5)
    pads = sorted(pmap.pad_edges)
    wires = []
    for wire in range(40):
        a, b = rng.sample(pads, 2)
        r = route(pmap, state, a, b)
        if r is not None:
            state.insert(wire, r.steps)
            wires.append((a, b, path_from_steps(pmap, r.steps)))
    free = pmap.free_space.free.buffer(1e-6)
    checked = crossing = 0
    for a, b, path in wires:
        line = LineString([pmap.pad_centre[a]] + [pmap.edge_mid_list[g] for g in path.gates] + [pmap.pad_centre[b]])
        for _ in range(30):
            x, y = rng.sample(pads, 2)
            if {x, y} & {a, b}:
                continue  # a shared pad leaves the order free (undetermined)
            air = airwire_path(pmap, pmap.pad_centre[x], pmap.pad_centre[y])
            if any(pmap.edge_kind_list[g] == TERMINAL for g in path.gates if g in air.pos):
                continue  # the airwire runs through one of the wire's own pads: order is free there too
            hits = line.intersection(LineString([pmap.pad_centre[x], pmap.pad_centre[y]])).intersection(free)
            n = 0 if hits.is_empty else len(getattr(hits, "geoms", [hits]))
            k = cross_count(pmap, path, air)
            assert k == cross_count(pmap, air, path)
            assert k <= n and k % 2 == n % 2
            checked += 1
            crossing += k > 0
    assert checked > 200 and crossing > 20
    # Committed wires never cross each other.
    assert sum(cross_count(pmap, p, q) for i, (_, _, p) in enumerate(wires) for _, _, q in wires[i + 1:]) == 0


def test_barrier_closure_matches_connectivity():
    """17.1: union-find closure detection against brute-force connectivity."""
    rng = random.Random(2)
    for _ in range(50):
        barrier, edges = Barrier(), []
        for step in range(40):
            a, b = rng.randrange(12), rng.randrange(12)
            # brute force: is b reachable from a over the welds so far?
            seen, todo = {a}, [a]
            while todo:
                x = todo.pop()
                for p, q in edges:
                    for s, t in ((p, q), (q, p)):
                        if s == x and t not in seen:
                            seen.add(t)
                            todo.append(t)
            connected = b in seen
            assert (barrier.loop(a, b) is not None) == connected
            trial = barrier.trial([(a, b, step)])
            assert bool(trial) == connected
            assert barrier.weld(a, b, step) == connected
            edges.append((a, b))
            if connected and a != b:
                nodes, elements = trial[0]
                assert nodes[0] == b and nodes[-1] == a and len(elements) == len(nodes)
