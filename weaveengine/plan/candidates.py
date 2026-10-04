"""Candidate generation and scoring (design sections 9.1 - 9.5)."""
from dataclasses import dataclass

from weaveengine.plan.context import Connection, Context, Layer
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


def generate(ctx: Context, conn: Connection, k: int | None = None) -> list[Candidate]:
    """Up to K topologically distinct routes per usable layer on the current
    state (9.3): route, penalise every gate of the result, search again."""
    params = ctx.params
    k = params.K if k is None else k
    cands: list[Candidate] = []
    for li in conn.layers:
        layer = ctx.layers[li]
        pmap, state = layer.pmap, layer.state
        first = route(pmap, state, conn.src, conn.dst, params, weight=conn.weight)
        if first is None:
            continue
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
            r = route(pmap, state, conn.src, conn.dst, params, penalty=penalty, weight=conn.weight)
            if r is None:
                break
            r.cost -= sum(penalty.get(g, 0.0) for g in r.gates)
            last = r
            if r.cost > limit:
                break
            found.setdefault(r.gates, r)
        for r in found.values():
            r.layer = li
            cand = Candidate(r, path_from_steps(pmap, r.steps), li, route_weight=conn.weight)
            cand.score = score(ctx, conn, cand)
            cands.append(cand)
    cands.sort(key=lambda c: c.score)
    return cands


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
    """Cheapest in-search route over the usable layers."""
    best = None
    for li in conn.layers:
        layer = ctx.layers[li]
        r = route(layer.pmap, layer.state, conn.src, conn.dst, ctx.params, weight=conn.weight, **kwargs)
        if r is not None and (best is None or r.cost < best.cost):
            r.layer = li
            best = r
    return best


def best_route(ctx: Context, conn: Connection) -> Route | None:
    """Best route on the current state under the full cost model."""
    if not (ctx.options.lookahead or ctx.options.demand):
        return plain_route(ctx, conn)
    cands = generate(ctx, conn, ctx.params.K_reroute)
    return cands[0].route if cands else None
