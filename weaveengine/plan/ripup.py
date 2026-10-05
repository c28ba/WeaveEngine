"""Phase 3: negotiated rip-up and reroute, and Phase 4 refinement (section 10)."""
from weaveengine.plan.candidates import plain_route, route_batch
from weaveengine.plan.context import Context
from weaveengine.plan.path import find, place
from weaveengine.topo.search import Route

STALL_LIMIT = 8
QUIET_STALL_LIMIT = 3


def negotiate(ctx: Context, max_rounds: int | None = None) -> None:
    """Loop until zero overflow and zero unrouted, or the iteration limit.
    The best solution seen (fewest violations, then shortest) is kept."""
    params = ctx.params
    max_rounds = params.max_rounds if max_rounds is None else max_rounds
    best_key, best_snap, stall = None, None, 0
    first_violations = max(1, ctx.violations())

    for _ in range(max_rounds + 1):
        key = (ctx.violations(), ctx.estimated_length())
        if best_key is None or key < best_key:
            best_key, best_snap, stall = key, ctx.snapshot(), 0
        else:
            stall += 1
        over = [(layer, e) for layer in ctx.layers for e in layer.state.overflowed_gates()]
        open_conns = [w for w in ctx.unrouted if not ctx.conns[w].dead]
        # Once nothing is over-full, a round only changes which connections
        # are the open ones: if that has stopped helping, it will not start again.
        if (not over and not open_conns) or stall >= (STALL_LIMIT if over else QUIET_STALL_LIMIT) or ctx.rounds >= max_rounds:
            break
        ctx.rounds += 1
        ctx.report("rip-up", first_violations - min(first_violations, best_key[0]), first_violations)

        # 1. Raise prices, up to the cap.
        params.pres_fac = min(params.pres_cap, params.pres_fac * params.pres_growth)
        for layer, e in over:
            layer.state.hist[e] += params.hist_inc * layer.state.overflow(e)

        # 2. Rip-up set: wires on over-capacity gates, plus the blocking set
        #    of the cheapest relaxed path of each unrouted connection (8.5).
        rip: set[int] = set()
        for layer, e in over:
            rip.update(layer.state.gate_order[e])
        for w in open_conns:
            conn = ctx.conns[w]
            r = find(ctx, conn, mode="relaxed")
            if r is None:
                conn.dead = True
            else:
                rip |= r.blocking
        rip = {ctx.conns[w].parent or w for w in rip}  # a connection through vias goes as a whole
        for w in sorted(rip):
            ctx.rip(w)

        # 3. Barrier is append-only: rebuild from what is left.
        ctx.rebuild_barrier()

        # 4. Reroute: connections that were stuck go first, then the ripped ones.
        todo = sorted((w for w in ctx.unrouted if not ctx.conns[w].dead),
                      key=lambda w: (w in rip, -ctx.conns[w].fails, ctx.conns[w].air_len))
        for w, r in route_batch(ctx, todo):
            conn = ctx.conns[w]
            if not place(ctx, conn, r):
                conn.fails += 1
        ctx.report("rip-up", first_violations - min(first_violations, best_key[0]), first_violations)

    if best_snap is not None and (ctx.violations(), ctx.estimated_length()) > best_key:
        ctx.restore(best_snap)


def legalise(ctx: Context) -> None:
    """Final guarantee: no gate over capacity. Rips the worst offenders, then
    places whatever is unrouted that fits without creating overflow."""
    while True:
        count: dict[int, int] = {}
        for layer in ctx.layers:
            for e in layer.state.overflowed_gates():
                for w in layer.state.gate_order[e]:
                    w = ctx.conns[w].parent or w
                    count[w] = count.get(w, 0) + 1
        if not count:
            break
        ctx.rip(max(count, key=lambda w: (count[w], w)))
    ctx.rebuild_barrier()
    fill(ctx)


def fill(ctx: Context) -> int:
    """Places every open connection that fits with no gate over capacity,
    shortest first, until a pass places none (one placed can be what another
    was waiting for: its vias are new places to change layer). Nothing is
    ripped up, so this only ever adds. Returns the number placed.

    Negotiation does not do this: there an over-full gate is cheaper than a
    via (9), so that it settles which connections share a layer, and an open
    connection that could go round by vias keeps asking for the gate instead."""
    placed = 0
    while True:
        before = len(ctx.unrouted)
        for w in sorted(ctx.unrouted, key=lambda w: (ctx.conns[w].air_len, w)):
            place(ctx, ctx.conns[w], find(ctx, ctx.conns[w], hard_cap=True), hard_cap=True)
        if len(ctx.unrouted) == before:
            return placed
        placed += before - len(ctx.unrouted)


def settle(ctx: Context) -> None:
    """From whatever negotiation left to a legal routing with nothing more to
    add: no gate over capacity, routes shortened (Phase 4), and every open
    connection that fits placed. Shorter routes leave room, so filling and
    shortening alternate while filling places anything."""
    legalise(ctx)
    if ctx.options.refine:
        ctx.report("refinement")
        refine(ctx)
        while fill(ctx):
            refine(ctx)


def refine(ctx: Context, passes: int = 2) -> int:
    """Phase 4 (topology refinement): take each wire out and put it back by the
    cheapest legal route, longest detours first. A wire only moves when that
    shortens it, so the total never grows. Returns the number of wires moved."""
    moved = 0
    for _ in range(passes):
        changed = 0
        lengths = {}
        for layer in ctx.layers:
            mids = layer.pmap.edge_mid_list
            for w, path in layer.paths.items():
                g = path.gates
                lengths[w] = sum(((mids[a][0] - mids[b][0]) ** 2 + (mids[a][1] - mids[b][1]) ** 2) ** 0.5 for a, b in zip(g, g[1:]))
        for w in sorted(lengths, key=lambda w: ctx.conns[w].air_len - lengths[w]):
            conn = ctx.conns[w]
            if conn.parent is not None:
                continue  # a piece of a connection through vias: left as it is (via reduction is M16)
            layer = ctx.layers[conn.layer]
            # Stored slots are those at insertion time; other wires have come
            # and gone since, so read the wire's present position on each gate.
            order = layer.state.gate_order
            old_layer = conn.layer
            old_steps = [(e, t, k, order[e].index(w)) for e, t, k, _ in layer.state.wire_path[w]]
            ctx.rip(w)
            r = plain_route(ctx, conn, hard_cap=True, congestion=False)
            if r is not None and r.length < lengths[w] - 1e-6:
                ctx.commit(conn, r)
                changed += 1
            else:
                # Put it back exactly where it was.
                ctx.commit(conn, Route(old_steps, 0.0, 0.0, layer=old_layer))
        moved += changed
        if not changed:
            break
    ctx.rebuild_barrier()
    return moved
