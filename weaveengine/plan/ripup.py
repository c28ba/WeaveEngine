"""Phase 3: negotiated rip-up and reroute, and Phase 4 refinement (section 10)."""
import math

from weaveengine.plan import vias
from weaveengine.plan.candidates import best_route, plain_route, route_batch
from weaveengine.plan.context import Context
from weaveengine.topo.search import Route

STALL_LIMIT = 8
VIA_STALL_LIMIT = 3


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
        # With vias to follow (``complete``), a negotiation that has stopped
        # improving is not worth continuing: what is open now needs a via.
        if (not over and not open_conns) or stall >= (VIA_STALL_LIMIT if ctx.live_vias else STALL_LIMIT) or ctx.rounds >= max_rounds:
            break
        ctx.rounds += 1
        ctx.report("rip-up", first_violations - min(first_violations, best_key[0]), first_violations)

        # 1. Raise prices. Capped: beyond a few crossing penalties it only blunts the A* heuristic.
        params.pres_fac = min(10.0 * params.cross_penalty, params.pres_fac * params.pres_growth)
        for layer, e in over:
            layer.state.hist[e] += params.hist_inc * layer.state.overflow(e)

        # 2. Rip-up set: wires on over-capacity gates, plus the blocking set
        #    of the cheapest relaxed path of each unrouted connection (8.5).
        rip: set[int] = set()
        for layer, e in over:
            rip.update(layer.state.gate_order[e])
        for w in open_conns:
            conn = ctx.conns[w]
            r = plain_route(ctx, conn, mode="relaxed")
            if r is None:
                conn.dead = True
            else:
                rip |= r.blocking
        for w in rip:
            ctx.rip(w)

        # 3. Barrier is append-only: rebuild from what is left.
        ctx.rebuild_barrier()

        # 4. Reroute: connections that were stuck go first, then the ripped ones.
        todo = sorted((w for w in ctx.unrouted if not ctx.conns[w].dead),
                      key=lambda w: (w in rip, -ctx.conns[w].fails, ctx.conns[w].air_len))
        for w, r in route_batch(ctx, todo):
            conn = ctx.conns[w]
            if r is None:
                conn.fails += 1
            else:
                ctx.commit(conn, r)
        ctx.report("rip-up", first_violations - min(first_violations, best_key[0]), first_violations)

    if best_snap is not None and (ctx.violations(), ctx.estimated_length()) > best_key:
        ctx.restore(best_snap)


def legalise(ctx: Context) -> None:
    """Final guarantee: no gate over capacity. Rips the worst offenders, then
    tries once more to place whatever is unrouted without creating overflow."""
    while True:
        count: dict[int, int] = {}
        for layer in ctx.layers:
            for e in layer.state.overflowed_gates():
                for w in layer.state.gate_order[e]:
                    count[w] = count.get(w, 0) + 1
        if not count:
            break
        ctx.rip(max(count, key=lambda w: (count[w], w)))
    ctx.rebuild_barrier()
    for w in sorted(ctx.unrouted, key=lambda w: ctx.conns[w].air_len):
        conn = ctx.conns[w]
        r = plain_route(ctx, conn, hard_cap=True)
        if r is not None:
            ctx.commit(conn, r)


MAX_BLOCKERS = 8   # wires one connection may displace to get through


def complete(ctx: Context) -> int:
    """Connects what legal routing on single layers has left open, with vias
    (design 12.4). The state is legal on entry (``legalise``) and stays legal:
    nothing here over-fills a gate, so there is nothing to negotiate.

    Each open connection is given, in turn: a legal route; failing that, the
    cheapest legal way through vias; failing that, room, by displacing the
    wires that block it, which are then placed again the same way. A
    displacement is kept only if it leaves fewer board connections open, so
    the loop ends. A connection gets its vias all at once or not at all.
    Returns the number of board connections still open.
    """
    if not ctx.live_vias:
        return len(ctx.open_roots())
    ctx.collapse_idle()
    start = max(1, len(ctx.open_roots()))

    def place(w: int) -> None:
        conn = ctx.conns.get(w)
        if conn is None or w not in ctx.unrouted:
            return
        r = plain_route(ctx, conn, ctx.params.spare, hard_cap=True)
        if r is not None:
            ctx.commit(conn, r)
        else:
            vias.connect(ctx, conn)

    def in_order() -> list[int]:
        return sorted(ctx.unrouted, key=lambda w: (ctx.conns[w].air_len, w))

    tried: set = set()
    progress = True
    while progress and ctx.unrouted:
        progress = False
        for w in in_order():
            before = len(ctx.open_roots())
            place(w)
            progress |= len(ctx.open_roots()) < before
            ctx.report("vias", start - len(ctx.open_roots()), start)
        # What is still open is walled in on every layer: displace the wires in its way.
        for w in in_order():
            conn = ctx.conns.get(w)
            if conn is None or w not in ctx.unrouted:
                continue
            r = plain_route(ctx, conn, mode="relaxed", hard_cap=True)
            if r is None or not r.blocking or len(r.blocking) > MAX_BLOCKERS:
                continue
            key = (w, frozenset(r.blocking))
            if key in tried:
                continue
            tried.add(key)
            before, snap = len(ctx.open_roots()), ctx.snapshot()
            moved = sorted(r.blocking)
            for b in moved:
                ctx.rip(b)
            place(w)
            if w in ctx.unrouted:
                ctx.restore(snap)
                continue
            for b in sorted(moved, key=lambda b: (ctx.conns[b].air_len, b)):
                place(b)
            if len(ctx.open_roots()) < before:
                progress = True
            else:
                ctx.restore(snap)
            ctx.report("vias", start - len(ctx.open_roots()), start)
    # No half-connected leftovers: a board connection that is still open gives its vias back.
    for w in ctx.open_roots():
        if ctx.conns[w].children:
            ctx.abandon(ctx.conns[w])
    ctx.rebuild_barrier()
    return len(ctx.open_roots())


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
