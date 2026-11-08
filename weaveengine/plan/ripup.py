"""Phase 3: negotiated rip-up and reroute, and Phase 4 refinement (section 10)."""
import math
import os

from weaveengine.plan.candidates import route_batch
from weaveengine.plan.context import Context
from weaveengine.plan.path import find, place
from weaveengine.topo.search import route
from weaveengine.topo.state import TopoState

STALL_LIMIT = 8
QUIET_STALL_LIMIT = 3
DETOUR = 1.25      # a connection this many times longer than it would be alone on the board is in a detour (``untangle``)
MOST_MOVED = 12    # wires moved out of the way of one detour, at most


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
        # Detours before vias: undoing one moves wires and puts them back where they
        # now fit, through a via if need be, and what it frees lets others place.
        # Planned last, the vias see the routing as it will stay.
        if ctx.options.untangle and untangle(ctx, hold):
            refine(ctx, hold=hold)
            fill(ctx, hold)
        for _ in range(3 if vias else 0):  # (what is filled after may go through vias again)
            if not replan(ctx, hold=hold):
                break
            refine(ctx, hold=hold)
            fill(ctx, hold)


def alone(ctx: Context) -> dict[int, float]:
    """What each connection's route would be on the board with nothing else on
    it: its length through the middles of its gates on an empty state, the
    least over the layers it can use (a connection that must change layer:
    infinite). Worked out once per run; the board does not change."""
    kept = ctx.__dict__.get("_alone")
    if kept is None:
        empty = [TopoState(layer.pmap) for layer in ctx.layers]
        kept = ctx.__dict__["_alone"] = {}
        for w, c in ctx.conns.items():
            if c.parent is None:
                found = [route(ctx.layers[li].pmap, empty[li], c.src, c.dst, ctx.params, congestion=False, weight=c.weight, net=c.net_id)
                         for li in c.layers]
                kept[w] = min((r.length for r in found if r is not None), default=math.inf)
    return kept


def untangle(ctx: Context, hold=frozenset(), sweeps: int = 3) -> int:
    """Detours undone by moving what is in the way (design 11a).

    Most of the copper a routing has beyond what its connections would take
    alone on the board is in a few long detours, and what forces each of them
    is a handful of other wires: crossing its short way, or filling a gate on
    it. Refinement moves one connection at a time, with everything else where
    it is, so it never finds these. Here, longest detour first: the cheapest way
    for the connection with crossings allowed and capacity ignored names what is
    in the way; those wires are taken out with it, it is put in by its short way,
    and they are put back wherever they now fit (``fill``). The result is kept
    if nothing is left open that was not, no gate is over-full, and the copper
    (vias at their worth, as ``replan`` counts them) is less by a pitch;
    otherwise the board is restored exactly. Returns the number kept."""
    pitch, worth = ctx.layers[0].pmap.pitch, via_worth(ctx)
    lone = alone(ctx)

    def total(nets: set[int]) -> float:
        """The copper of these nets (a trace's copper depends only on its own net's
        wires beside it), and every via at its worth."""
        copper = sum(ctx.wire_copper(layer, w) for layer in ctx.layers for net in nets for w in layer.state.by_net.get(net, ()))
        return copper + worth * sum(1 for s in ctx.layers[0].pmap.sites.values() if s.active)

    def over() -> dict:
        """Over-full gates, and by how much (repair after the check can leave some for rip-up)."""
        return {(layer.index, e): layer.state.overflow(e) for layer in ctx.layers for e in layer.state.overflowed_gates()}

    kept, failed = 0, set()
    for _ in range(sweeps):
        was = ctx.copper()
        detours = sorted((w for w, cu in was.items() if w not in hold and cu > DETOUR * lone.get(w, math.inf) + 2.0 * pitch),
                         key=lambda w: lone[w] - was[w])
        changed = 0
        for w in detours:
            conn = ctx.conns[w]
            if not conn.routed:
                continue
            saved = ctx.lift(conn)
            short = find(ctx, conn, mode="relaxed", congestion=False)
            ctx.put_back(conn, saved)
            if short is None or was[w] - short.length < 2.0 * pitch:
                continue  # its short way is not much shorter than the way it has: nothing to win
            move = {ctx.conns[x].parent or x for x in short.blocking}
            for r in short.routes:  # and what fills the gates of the short way that have no room for it
                state = ctx.layers[r.layer].state
                for g in r.gates:
                    if not state.fits(g, conn.weight):
                        move |= {ctx.conns[x].parent or x for x in state.gate_order[g]}
            move.discard(w)
            if not move or len(move) > MOST_MOVED or move & hold or (w, frozenset(move)) in failed:
                continue  # (the same try, refused already in this call: it would be refused again but for chance)
            nets = {conn.net_id} | {ctx.conns[x].net_id for x in move} | {ctx.conns[x].net_id for x in ctx.unrouted}
            before, open_before, over_before = total(nets), set(ctx.unrouted), over()
            check = ctx.snapshot() if CHECK_UNDO else None
            # Lifted, not ripped, and what goes in recorded: a try that does not pay is
            # undone exactly, by taking out what it put in and putting back what it lifted, last first.
            lifted = [(ctx.conns[x], ctx.lift(ctx.conns[x])) for x in (w, *sorted(move)) if ctx.conns[x].routed]
            ctx.record()
            plan = find(ctx, conn, hard_cap=True, congestion=False)
            if plan is not None and ctx.commit(conn, plan):
                # What was moved goes back where it now fits, shortest first; the
                # first that does not fit ends the try (it would be left open).
                for x in sorted(move, key=lambda x: (ctx.conns[x].air_len, x)):
                    if not place(ctx, ctx.conns[x], find(ctx, ctx.conns[x], hard_cap=True), hard_cap=True):
                        break
            after = over()
            if (conn.routed and ctx.unrouted <= open_before and total(nets) < before - pitch
                    and all(n <= over_before.get(g, 0) for g, n in after.items())):  # (no gate over-full that was not, or more)
                ctx.trail = None
                changed += 1
            else:
                failed.add((w, frozenset(move)))
                ctx.undo()
                for c, saved in reversed(lifted):
                    ctx.put_back(c, saved)
                if check is not None:
                    _same(ctx, check)
        ctx.rebuild_barrier()
        kept += changed
        if not changed:
            break
    return kept


CHECK_UNDO = bool(os.environ.get("WEAVEENGINE_CHECK_UNDO"))  # tests: after each undone try, compare everything with before


def _same(ctx: Context, snap) -> None:
    """Raises if the routing is not exactly as in ``snap`` (``CHECK_UNDO``). The
    state's tables may have grown since, for via sites made and taken out
    again (they never shrink): the rows beyond have to be empty."""
    def equal(now, then, empty) -> bool:
        n = min(len(now), len(then))
        return list(now[:n]) == list(then[:n]) and all(x == empty for x in list(now[n:]) + list(then[n:]))

    now = ctx.snapshot()
    for a, b in zip(now["states"], snap["states"]):
        order, cnt, paths, load, weight, nets = a
        order0, cnt0, paths0, load0, weight0, nets0 = b
        n = min(len(load), len(load0))
        drift = abs(load[:n] - load0[:n]).max(initial=0.0) <= 1e-9 and not load[n:].any() and not load0[n:].any()  # (sums of weights, added and taken off)
        if not (equal(order, order0, []) and equal(cnt, cnt0, [0, 0, 0]) and drift):
            n = min(len(order), len(order0))
            li = now["states"].index(a)
            st = ctx.layers[li].state
            bad = [i for i, (x, y) in enumerate(zip(load.tolist(), load0.tolist())) if abs(x - y) > 1e-9][:3]
            raise AssertionError(f"tally now: {[(e, st.tally(e), [(w, st.net.get(w), st.weight.get(w)) for w in st.gate_order[e]]) for e in bad]} -- state differs after undo: order {[(e, order[e], order0[e]) for e in range(n) if order[e] != order0[e]][:3]} "
                                 f"tail {[x for x in order[n:] + order0[n:] if x][:3]} cnt {[(i, cnt[i], cnt0[i]) for i in range(min(len(cnt), len(cnt0))) if cnt[i] != cnt0[i]][:3]} "
                                 f"load {[(i, x, y) for i, (x, y) in enumerate(zip(load.tolist(), load0.tolist())) if x != y][:3]}")
        # A wire's steps keep the place it had on each gate when it went in, which a wire
        # put back (``put_back``) has afresh; where it is, is in the gates' orders above.
        paths = {w: [s[:3] for s in steps] for w, steps in paths.items()}
        paths0 = {w: [s[:3] for s in steps] for w, steps in paths0.items()}
        if not (paths == paths0 and weight == weight0 and nets == nets0):
            pw = [w for w in set(paths) | set(paths0) if paths.get(w) != paths0.get(w)][:3]
            raise AssertionError(f"wires differ after undo: {[(w, w in paths, w in paths0, (paths.get(w) or [])[:3], (paths0.get(w) or [])[:3]) for w in pw]} "
                                 f"weight {[(w, weight.get(w), weight0.get(w)) for w in set(weight) | set(weight0) if weight.get(w) != weight0.get(w)][:3]} "
                                 f"nets {[(w, nets.get(w), nets0.get(w)) for w in set(nets) | set(nets0) if nets.get(w) != nets0.get(w)][:3]}")
    assert now["vias"] == snap["vias"] and now["pieces"] == snap["pieces"] and now["next"] == snap["next"], "vias or pieces differ after undo"


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
            pmap = ctx.layers[before.layer].pmap
            total += math.dist(pmap.edge_mid_list[before.steps[-1][0]], joint) * pmap.length_cost
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

    # What it has now at the search's prices: its copper on a layer made dear costs it more (``CostParams.layer_cost``).
    priced = was + sum(ctx.wire_copper(ctx.layers[p.layer], p.wire_id) * (ctx.layers[p.layer].pmap.length_cost - 1.0)
                       for p in ([ctx.conns[w] for w in conn.pieces] or [conn]) if p.layer is not None)
    saved = ctx.lift(conn)
    # With the fewest vias first: the search's cheapest plan is not always the
    # one that does best once it is in, and a plan without a via is the one wanted.
    for vias in range(most + 1):
        struck, through = [], True
        for _ in range(3):  # what does not fit as planned is struck out, and the plan made again (``path.place``)
            plan = find(ctx, conn, bound=priced, struck=struck, hard_cap=True, congestion=False, max_vias=vias, via_cost=worth, through=through)
            if plan is None or not _planned(ctx, plan, worth) + worth * len(plan.vias) < priced:
                break  # (a search's cost is never more than the copper it stands for, at its layers' prices, so the bound loses nothing)
            if ctx.commit(conn, plan, within=better if plan.joints else None):
                return True
            struck += plan.vias
            through = through and bool(plan.vias)
    ctx.put_back(conn, saved)
    return False
