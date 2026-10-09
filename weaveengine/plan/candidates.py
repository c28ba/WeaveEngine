"""Candidate generation and scoring (design sections 9.1 - 9.5)."""
from dataclasses import dataclass

from weaveengine import parallel
from weaveengine.plan.context import Connection, Context, Layer
from weaveengine.plan.path import Path, find
from weaveengine.topo import kernel
from weaveengine.topo.planar_map import GATE
from weaveengine.topo.runs import GatePath, cross_count, path_from_steps
from weaveengine.topo.search import Route, route


@dataclass
class Candidate:
    route: Route
    path: GatePath
    layer: int
    score: float = 0.0
    route_weight: float = 1.0  # gate load of the connection


def generate(ctx: Context, conn: Connection, k: int | None = None, first: dict | None = None) -> list[Candidate]:
    """Up to K topologically distinct routes per usable layer on the current
    state (9.3): route, penalise every gate of the result, search again.

    Layers are searched at the same time: each has its own map, state and
    search workspace, and the compiled kernel does not hold the interpreter lock.
    ``first``, if given, is filled per layer with the outcome of the plain
    search there, as ``find`` takes it.
    """
    k = ctx.params.K if k is None else k
    if _use_threads(ctx, conn):
        found = list(_threads().map(lambda li: _layer_routes(ctx, conn, li, k, first), conn.layers))
    else:
        found = [_layer_routes(ctx, conn, li, k, first) for li in conn.layers]
    cands: list[Candidate] = []
    for li, routes in zip(conn.layers, found):
        pmap = ctx.layers[li].pmap
        for r in routes:
            r.layer = li
            cand = Candidate(r, path_from_steps(pmap, r.steps), li, route_weight=conn.weight)
            cand.score = score(ctx, conn, cand)
            cands.append(cand)
    cands.sort(key=lambda c: c.score)
    return cands


_pool = None


def _use_threads(ctx: Context, conn: Connection) -> bool:
    """Layer threads only in the main process: a forked worker has no thread pool
    (threads do not survive a fork) and is already one of several workers."""
    return len(conn.layers) > 1 and kernel.AVAILABLE and ctx.workers != 1 and not parallel._inside


def _stop_threads() -> None:
    global _pool
    if _pool is not None:
        _pool.shutdown(wait=True)
        _pool = None


parallel.before_fork.append(_stop_threads)


def _threads():
    global _pool
    if _pool is None:
        from concurrent.futures import ThreadPoolExecutor
        _pool = ThreadPoolExecutor(max_workers=4, thread_name_prefix="weave-layer")
    return _pool


def _layer_routes(ctx: Context, conn: Connection, li: int, k: int, plain: dict | None = None) -> list[Route]:
    params = ctx.params
    layer = ctx.layers[li]
    pmap, state = layer.pmap, layer.state
    first = None
    if plain is not None:  # the search ``find`` starts with: it may go through pads of its net, which a candidate does not
        first, _ = plain[li] = route(pmap, state, conn.src, conn.dst, params, weight=conn.weight, net=conn.net_id, reach=True, through=True)
    if first is None or first.lead:
        first = route(pmap, state, conn.src, conn.dst, params, weight=conn.weight, net=conn.net_id)
    if first is None:
        return []
    found = {first.gates: first}
    limit = (1.0 + params.alpha) * first.cost + 1e-9
    penalty: dict[int, float] = {}
    last = first
    for _ in range(3 * k):
        if len(found) >= k:
            break
        bump = params.alpha * max(first.length, 1e-6) / len(last.steps)
        for g in last.gates:
            penalty[g] = penalty.get(g, 0.0) + bump
        r = route(pmap, state, conn.src, conn.dst, params, penalty=penalty, weight=conn.weight, net=conn.net_id)
        if r is None:
            break
        r.cost -= sum(penalty.get(g, 0.0) for g in r.gates)
        last = r
        if r.cost > limit:
            break
        found.setdefault(r.gates, r)
    return list(found.values())


def score(ctx: Context, conn: Connection, cand: Candidate) -> float:
    """Whole-path cost: in-search cost plus the lookahead terms of section 9."""
    params, opts = ctx.params, ctx.options
    layer = ctx.layers[cand.layer]
    total = cand.route.cost
    if opts.demand:
        total += params.w_d * demand_overflow(ctx, layer, conn, cand.path)
    if opts.lookahead:
        # A barrier on one layer can be bypassed on another: scale by layer availability (9.5).
        share = 1.0 / len(ctx.layers)
        total += share * params.lambda_x * airwire_crossings(ctx, layer, conn, cand.path)
        total += share * params.lambda_sever * len(severed(ctx, layer, conn, cand.path))
    return total


def demand_overflow(ctx: Context, layer: Layer, conn: Connection, path: GatePath) -> float:
    load, cap, demand = layer.state.load, layer.state.cap, layer.demand
    own = conn.demand_gates.get(layer.index, ()) if conn.wire_id in ctx.unrouted else ()
    share = conn.weight / max(1, len(conn.layers))
    total = 0.0
    for g in path.gates:
        over = load[g] + demand[g] - (share if g in own else 0.0) + conn.weight - cap[g]
        if over > 1e-9:
            total += over
    return total


def airwire_crossings(ctx: Context, layer: Layer, conn: Connection, path: GatePath) -> int:
    """How many unrouted airwires this path separates (9.4)."""
    others: set[int] = set()
    for g in path.gates:
        others |= layer.air_index.get(g, set())
    others.discard(conn.wire_id)
    return sum(cross_count(layer.pmap, path, ctx.conns[o].airwire[layer.index]) for o in others)


def severed(ctx: Context, layer: Layer, conn: Connection, path: GatePath) -> set[int]:
    """Unrouted connections cut off by barrier loops this path would close (9.5)."""
    pmap, state = layer.pmap, layer.state
    welds: list[tuple[int, int, object]] = [(pmap.pad_obs[conn.src], pmap.pad_obs[conn.dst], path)]
    for g in path.gates:
        if pmap.edge_kind_list[g] == GATE and not state.full(g) and state.full(g, conn.weight):
            u, v = pmap.edge_v_list[g]
            welds.append((pmap.v_obs_list[u], pmap.v_obs_list[v], g))
    cut: set[int] = set()
    for nodes, elements in layer.barrier.trial(welds):
        on_loop = set(nodes)
        for wid in ctx.unrouted:
            other = ctx.conns[wid]
            air = other.airwire.get(layer.index)
            if wid == conn.wire_id or wid in cut or air is None:
                continue
            if pmap.pad_obs[other.src] in on_loop or pmap.pad_obs[other.dst] in on_loop:
                continue
            parity = 0
            for el in elements:
                if isinstance(el, GatePath):
                    parity += cross_count(pmap, el, air)
                elif el in air.pos:
                    parity += 1
            if parity & 1:
                cut.add(wid)
    return cut


def plain_route(ctx: Context, conn: Connection, **kwargs) -> Route | None:
    """Cheapest in-search route on one layer, over the usable layers (searched at the same time)."""
    def one(li: int) -> Route | None:
        layer = ctx.layers[li]
        return route(layer.pmap, layer.state, conn.src, conn.dst, ctx.params, weight=conn.weight, net=conn.net_id, **kwargs)

    if _use_threads(ctx, conn):
        found = list(_threads().map(one, conn.layers))
    else:
        found = [one(li) for li in conn.layers]
    best = None
    for li, r in zip(conn.layers, found):
        if r is not None and (best is None or r.cost < best.cost):
            r.layer = li
            best = r
    return best


def best_route(ctx: Context, conn: Connection) -> Path | None:
    """Best route on the current state: on one layer under the full cost model
    (the lookahead terms of section 9), or through vias if that is cheaper."""
    first = {} if ctx.vias else None  # the plain search on each layer: both ways start with it
    if ctx.options.lookahead or ctx.options.demand:
        cands = generate(ctx, conn, ctx.params.K_reroute, first)
        direct = cands[0].route if cands else None
    else:
        direct = plain_route(ctx, conn)
    through = find(ctx, conn, first=first) if ctx.vias else None
    if through is not None and through.joints and (direct is None or through.cost < direct.cost):
        return through
    return Path.on_one_layer(direct) if direct is not None else through


def _generate_task(ctx: Context, wire_id: int) -> list[Candidate]:
    return generate(ctx, ctx.conns[wire_id])


def generate_all(ctx: Context) -> dict[int, list[Candidate]]:
    """Phase 1 candidates for every connection. Read-only on the state, so the
    connections are spread over worker processes (section 3 rule 7)."""
    ids = list(ctx.conns)
    found = parallel.run(_generate_task, ctx, ids, ctx.workers)
    return dict(zip(ids, found))


def _best_task(ctx: Context, wire_id: int) -> Path | None:
    return best_route(ctx, ctx.conns[wire_id])


def replay(ctx: Context, conn: Connection, path: Path) -> Path | None:
    """The same gate sequence on the present state (slots recomputed), or None
    if it no longer fits. A route in pieces is not replayed but planned again."""
    if path.joints:
        return None
    r = path.routes[0]
    layer = ctx.layers[r.layer]
    again = route(layer.pmap, layer.state, conn.src, conn.dst, ctx.params, mode="corridor",
                  corridor=set(r.gates), weight=conn.weight, net=conn.net_id)
    if again is None:
        return None
    again.layer = r.layer
    return Path.on_one_layer(again)


def route_batch(ctx: Context, wire_ids: list[int], compute=_best_task):
    """Yields (wire id, route or None) for the unrouted connections ``wire_ids``.

    Routes are computed for ``params.batch`` connections at a time against one
    snapshot of the state, in parallel; the caller commits each yielded route
    before the next one is yielded. A route that an earlier commit of the same
    batch has invalidated is recomputed on the spot. The outcome depends on the
    batch size but not on the number of workers.
    """
    size = max(1, ctx.params.batch)
    for at in range(0, len(wire_ids), size):
        chunk = wire_ids[at:at + size]
        routes = parallel.run(compute, ctx, chunk, ctx.workers)
        for i, (w, r) in enumerate(zip(chunk, routes)):
            conn = ctx.conns[w]
            if r is not None and i > 0:
                r = replay(ctx, conn, r) or best_route(ctx, conn)
            yield w, r
