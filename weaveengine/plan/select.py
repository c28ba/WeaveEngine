"""Phase 1: choose one candidate per connection for all connections at once (section 10)."""
import random

from weaveengine.plan.candidates import Candidate
from weaveengine.plan.context import Context
from weaveengine.topo.runs import cross_count


def select(ctx: Context, cands: dict[int, list[Candidate]]) -> dict[int, int]:
    """Iterated conditional modes with random restarts. Returns wire id -> candidate index."""
    params = ctx.params
    ids = sorted(w for w, cs in cands.items() if cs)
    if not ids:
        return {}
    rng = random.Random(ctx.seed)

    # Sparse conflict table between candidates of different connections that share a gate.
    by_gate: dict[tuple[int, int], list[tuple[int, int]]] = {}
    for w in ids:
        for ci, c in enumerate(cands[w]):
            for g in c.path.gates:
                by_gate.setdefault((c.layer, g), []).append((w, ci))
    conflict: dict[tuple[int, int], dict[tuple[int, int], int]] = {}
    seen: set[tuple] = set()
    for users in by_gate.values():
        for i, a in enumerate(users):
            for b in users[i + 1:]:
                if a[0] == b[0] or (a, b) in seen:
                    continue
                seen.add((a, b))
                ca, cb = cands[a[0]][a[1]], cands[b[0]][b[1]]
                n = cross_count(ctx.layers[ca.layer].pmap, ca.path, cb.path)
                if n:
                    conflict.setdefault(a, {})[b] = n
                    conflict.setdefault(b, {})[a] = n

    # Gates as one number over all layers, and what each holds before this selection.
    width = max(layer.pmap.num_edges for layer in ctx.layers)
    held = [x for layer in ctx.layers for x in layer.state.load[:layer.pmap.num_edges].tolist() + [0.0] * (width - layer.pmap.num_edges)]
    cap = [x for layer in ctx.layers for x in layer.state.cap[:layer.pmap.num_edges].tolist() + [0.0] * (width - layer.pmap.num_edges)]
    keys = {w: [[c.layer * width + g for g in c.path.gates] for c in cands[w]] for w in ids}
    weight = {w: ctx.conns[w].weight for w in ids}
    no_conflict: dict = {}

    def marginal(w: int, ci: int, sel: dict[int, int], load: dict) -> float:
        crossings = sum(n for (w2, cj), n in conflict.get((w, ci), no_conflict).items() if sel.get(w2) == cj)
        overflow, mine = 0.0, weight[w]
        for key in keys[w][ci]:
            over = held[key] + load.get(key, 0.0) + mine - cap[key]
            if over > 1e-9:
                overflow += over
        return cands[w][ci].score + params.lambda_conf * (crossings + overflow)

    def run(sel: dict[int, int]) -> float:
        load: dict[int, float] = {}
        for w, ci in sel.items():
            for key in keys[w][ci]:
                load[key] = load.get(key, 0.0) + weight[w]
        for _ in range(30):
            changed = False
            visit = ids[:]
            rng.shuffle(visit)
            for w in visit:
                for key in keys[w][sel[w]]:
                    load[key] -= weight[w]
                best = min(range(len(cands[w])), key=lambda ci: marginal(w, ci, sel, load))
                changed |= best != sel[w]
                sel[w] = best
                for key in keys[w][best]:
                    load[key] = load.get(key, 0.0) + weight[w]
            if not changed:
                break
        # Objective: single scores + pairwise crossings (each pair once) + overflow.
        total = sum(cands[w][ci].score for w, ci in sel.items())
        pair = sum(n for a, row in conflict.items() if sel[a[0]] == a[1]
                   for b, n in row.items() if sel[b[0]] == b[1]) / 2
        over = sum(max(0.0, held[key] + n - cap[key]) for key, n in load.items())
        return total + params.lambda_conf * (pair + over)

    best_sel, best_cost = None, None
    for restart in range(params.restarts + 1):
        sel = {w: 0 for w in ids} if restart == 0 else {w: rng.randrange(len(cands[w])) for w in ids}
        cost = run(sel)
        if best_cost is None or cost < best_cost:
            best_sel, best_cost = dict(sel), cost
    return best_sel
