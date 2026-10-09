"""Phase 3: negotiated rip-up and reroute, and Phase 4 refinement (section 10)."""
import math

from weaveengine.plan.candidates import route_batch
from weaveengine.plan.context import Context
from weaveengine.plan.path import find, place

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
        # For the estimate of the time left: the rounds still to run, this one
        # included, if none of them improves on the best; and at most.
        most = max_rounds - ctx.rounds + 1
        ctx.report("rip-up", first_violations - min(first_violations, best_key[0]), first_violations, round=ctx.rounds,
                   quiet=min((STALL_LIMIT if over else QUIET_STALL_LIMIT) - stall, most), most=most)

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


def fill(ctx: Context, hold=frozenset()) -> int:
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
        for w in sorted(ctx.unrouted.difference(hold), key=lambda w: (ctx.conns[w].air_len, w)):
            place(ctx, ctx.conns[w], find(ctx, ctx.conns[w], hard_cap=True), hard_cap=True)
        if len(ctx.unrouted) == before:
            return placed
        placed += before - len(ctx.unrouted)


def settle(ctx: Context, hold=frozenset(), vias: bool = False) -> None:
    """From whatever negotiation left to a legal routing with nothing more to
    add: no gate over capacity, routes shortened (Phase 4), and every open
    connection that fits placed. Shorter routes leave room, so filling and
    shortening alternate while filling places anything. With ``vias``, each
    connection through vias is then planned again, the others being in their
    final places. The connections in ``hold`` are left exactly as they are."""
    legalise(ctx)
    if ctx.options.refine:
        ctx.report("refinement")
        refine(ctx, hold=hold)
        while fill(ctx, hold):
            refine(ctx, hold=hold)
        if vias and replan(ctx, hold=hold):
            refine(ctx, hold=hold)
            fill(ctx, hold)


def refine(ctx: Context, passes: int = 2, hold=frozenset()) -> int:
    """Phase 4 (topology refinement): take each connection out and put it back
    by the cheapest legal route, longest detours first. A connection only moves
    when that leaves less copper (``Context.copper``: what it shares with its
    net is not its own), so the total never grows. Returns the number moved.
    Connections through vias are left to ``replan``."""
    moved = 0
    for _ in range(passes):
        was = ctx.copper()
        plain = [w for w in was if not ctx.conns[w].sites and w not in hold]
        changed = sum(_replan(ctx, ctx.conns[w], was[w], 0, 1e-6) for w in sorted(plain, key=lambda w: ctx.conns[w].air_len - was[w]))
        moved += changed
        if not changed:
            break
    ctx.rebuild_barrier()
    return moved


def replan(ctx: Context, passes: int = 3, hold=frozenset()) -> int:
    """Plans each connection through vias again as a whole, vias and all,
    longest detours first: where it changes layer was decided under the prices
    of the moment it was routed, often long before the board looked as it does
    now. One that moves leaves room for another, so this is done again while
    any does. Returns the number of times a connection came out better."""
    moved = 0
    for _ in range(passes):
        was = ctx.copper()
        through = [w for w in was if ctx.conns[w].sites and w not in hold]
        # better by a pitch at least: less is not worth moving a via for
        changed = sum(_replan(ctx, ctx.conns[w], was[w], len(ctx.conns[w].sites), ctx.layers[0].pmap.pitch)
                      for w in sorted(through, key=lambda w: ctx.conns[w].air_len - was[w]))
        moved += changed
        if not changed:
            break
    return moved


def via_worth(ctx: Context) -> float:
    """What a via is worth in length of trace when a routing is tidied: the
    track it takes away, which is the width it keeps clear, on every layer. A
    route is better without a via if that makes it no more than this much
    longer. (Not the price of a via while the board is negotiated (9): that
    one decides who gets a layer, and would trade a via for a detour of many
    times its size.)"""
    rules = ctx.board.rules
    return len(ctx.layers) * (rules.via_diameter + 2.0 * rules.clearance + rules.base_width)


def _planned(ctx: Context, plan, worth: float) -> float:
    """A plan's copper as ``Context.copper`` will measure it once it is in:
    what the search charged for it, less its vias, and the way to each via
    (the way on from it the search has counted)."""
    total = plan.cost - worth * len(plan.vias)
    for joint, before in zip(plan.joints, plan.routes):
        if not isinstance(joint, int):
            total += math.dist(ctx.layers[before.layer].pmap.edge_mid_list[before.steps[-1][0]], joint)
    return total


def _own_vias(ctx: Context, pieces) -> int:
    """How many vias these pieces of a connection have to themselves: no trace of another connection ends on them."""
    site, mine = ctx.layers[0].pmap.sites, {piece.wire_id for piece in pieces}
    return sum(1 for pad in {piece.dst for piece in pieces[:-1]} if pad in site
               and all(w in mine for layer in ctx.layers for e in layer.pmap.sites[pad].hole for w in layer.state.gate_order[e]))


def _replan(ctx: Context, conn, was: float, most: int, margin: float) -> bool:
    """Plans one connection again, with at most ``most`` new vias (as many as
    it goes through now, its own or its net's: it is given no more), and keeps the new plan if,
    once it is in, the connection is better off by ``margin``: less copper,
    counting each via it has to itself at ``via_worth``. So a via goes where a
    trace would do, a via of its own comes where the way round by one of its
    net's is longer than a via is worth, and a trace of its own goes where its
    net's copper would do. The connection is lifted for this; if no plan is
    better, or none fits, it is put back exactly."""
    worth = via_worth(ctx)
    was += worth * _own_vias(ctx, [ctx.conns[w] for w in conn.pieces]) - margin
    # As good as any route could be: the straight line, with the via its pads force. (Only for a
    # net of two pads: one of more may do better still, since beside its own copper a trace is free.)
    if conn.net_id in ctx.pairs and conn.air_len + (0.0 if conn.layers else worth) >= was:
        return False

    def better(pieces) -> bool:
        """Asked by ``commit`` with the plan in: is it what it promised, and is no gate over-full?"""
        copper = sum(ctx.wire_copper(ctx.layers[piece.layer], piece.wire_id) for piece in pieces)
        return copper + worth * _own_vias(ctx, pieces) < was and not any(layer.state.overflowed_gates() for layer in ctx.layers)

    saved = ctx.lift(conn)
    # With the fewest vias first: the search's cheapest plan is not always the
    # one that does best once it is in, and a plan without a via is the one wanted.
    for vias in range(most + 1):
        struck, through = [], True
        for _ in range(3):  # what does not fit as planned is struck out, and the plan made again (``path.place``)
            plan = find(ctx, conn, bound=was, struck=struck, hard_cap=True, congestion=False, max_vias=vias, via_cost=worth, through=through)
            if plan is None or not _planned(ctx, plan, worth) + worth * len(plan.vias) < was:
                break  # (a search's cost is never more than the copper it stands for, so the bound loses nothing)
            if ctx.commit(conn, plan, within=better if plan.joints else None):
                return True
            struck += plan.vias
            through = through and bool(plan.vias)
    ctx.put_back(conn, saved)
    return False
