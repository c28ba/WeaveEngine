"""Phase 2: commit in regret order (section 10)."""
import math

from weaveengine import parallel
from weaveengine.plan.candidates import Candidate, best_route, replay
from weaveengine.plan.context import Context
from weaveengine.plan.path import Path, place
from weaveengine.topo.search import Route, route


def commit_all(ctx: Context, cands: dict[int, list[Candidate]], selection: dict[int, int]) -> list[int]:
    """Commits every unrouted connection it can. Returns the wire ids that failed."""
    params = ctx.params
    pending = set(ctx.unrouted)
    users: dict[tuple[int, int], set[int]] = {}  # (layer, gate) -> connections whose candidates use it
    for w in pending:
        for c in cands.get(w, []):
            for g in c.path.gates:
                users.setdefault((c.layer, g), set()).add(w)

    def current_cost(c: Candidate) -> float:
        state = ctx.layers[c.layer].state
        over = sum(1 for g in c.path.gates if state.load[g] + c.route_weight - state.cap[g] > 1e-9)
        return c.score + params.pres_fac * over

    def regret(w: int) -> float:
        if not ctx.options.regret_order:
            return -ctx.conns[w].air_len  # shortest first
        costs = sorted(current_cost(c) for c in cands.get(w, []))
        if not costs:
            return -math.inf              # nothing to lose: route last
        if len(costs) == 1:
            return math.inf               # only one option: route first
        return costs[1] - costs[0]

    regrets = {w: regret(w) for w in pending}
    failed: list[int] = []
    total = len(pending)
    size = max(1, params.batch)
    while pending:
        # The next batch in regret order, placed against one snapshot in parallel.
        chunk = sorted(pending, key=lambda x: (-regrets[x], x))[:size]
        pending.difference_update(chunk)
        routes = parallel.run(_place_task, (ctx, cands, selection), chunk, ctx.workers)
        dirty: set[int] = set()
        for i, (w, r) in enumerate(zip(chunk, routes)):
            conn = ctx.conns[w]
            if r is not None and i > 0:
                r = replay(ctx, conn, r) or _place_task((ctx, cands, selection), w)
            if not place(ctx, conn, r):
                failed.append(w)
                continue
            for piece in r.routes:
                for g in piece.gates:
                    dirty |= users.get((piece.layer, g), set())
        for d in dirty & pending:  # only regrets touched by these commits
            regrets[d] = regret(d)
        ctx.report("commit", total - len(pending), total)
    return failed


def _place_task(shared, w: int) -> Path | None:
    ctx, cands, selection = shared
    chosen = cands[w][selection[w]] if w in selection and cands.get(w) else None
    found = _place(ctx, ctx.conns[w], chosen)
    return Path.on_one_layer(found) if isinstance(found, Route) else found


def _place(ctx: Context, conn, chosen: Candidate | None):
    params = ctx.params
    if chosen is not None:
        # 1. Replay the selected gate sequence through the feasibility rules.
        layer = ctx.layers[chosen.layer]
        pmap, state = layer.pmap, layer.state
        gates = set(chosen.path.gates)
        r = route(pmap, state, conn.src, conn.dst, params, mode="corridor", corridor=gates, weight=conn.weight)
        if r is not None:
            r.layer = chosen.layer
            return r
        # 2. Search a corridor around the candidate.
        wide: set[int] = set()
        for _, t, _, _ in chosen.route.steps[1:]:
            wide.update(pmap.tri_e_list[t])
            for n in pmap.tri_n[t].tolist():
                if n >= 0:
                    wide.update(pmap.tri_e_list[n])
        r = route(pmap, state, conn.src, conn.dst, params, mode="corridor", corridor=wide, weight=conn.weight)
        if r is not None:
            r.layer = chosen.layer
            return r
    # 3. Unrestricted search.
    return best_route(ctx, conn)
