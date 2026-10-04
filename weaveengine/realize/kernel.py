"""Compiled Gauss-Seidel sweep for the relaxation (design sections 3 and 13.1).

Every crossing of a wire with a gate is a *slot* with a position along its
gate. The arrays describe the slots (gate frame, window, neighbours on the
gate) and the wires (their slots in order); the sweep moves each slot to the
shortest-path position inside its limits until nothing moves.
"""
import numpy as np

from weaveengine.topo.kernel import njit


@njit(cache=True, nogil=True)
def sweep(pos, frame, wlo, whi, terminal, has_prev, has_next, space, share,
          wire_ptr, wire_slot, centre, along_prev, along_next, slide, max_sweeps, tol, min_sin):
    """Relaxes ``pos`` in place. Returns the number of sweeps made.

    frame[s] = (ux, uy, dx, dy, L) of the slot's gate; (wlo, whi) its window;
    has_prev/has_next: a neighbouring wire below/above it on the same gate
    (slot ids s - 1 and s + 1); space[s]: spacing owed between s and s + 1;
    share[s]: spacing between wires sharing a pad edge; wire w owns the slots
    wire_slot[wire_ptr[w]:wire_ptr[w + 1]], from pad centre (centre[w, 0:2]) to
    pad centre (centre[w, 2:4]); along_prev/along_next: the previous and next
    slot of the same wire, or -1.
    """
    ns = pos.shape[0]
    nw = wire_ptr.shape[0] - 1
    sin = np.ones(ns)
    gap = np.zeros(ns)
    sweeps = 0
    for sw in range(max_sweeps):
        sweeps = sw + 1
        if sw % 4 == 0:  # angles change slowly; refreshing every sweep makes the iteration jitter
            for s in range(ns):
                px = frame[s, 0] + pos[s] * frame[s, 2]
                py = frame[s, 1] + pos[s] * frame[s, 3]
                low = 1.0
                for q in (along_prev[s], along_next[s]):
                    if q >= 0:
                        qx = frame[q, 0] + pos[q] * frame[q, 2] - px
                        qy = frame[q, 1] + pos[q] * frame[q, 3] - py
                        n = np.hypot(qx, qy)
                        if n > 1e-9:
                            v = abs(frame[s, 2] * qy - frame[s, 3] * qx) / n
                            if v < low:
                                low = v
                sin[s] = low
            for s in range(ns):
                if has_next[s] and not terminal[s]:
                    # Two wires a distance d apart cross a gate d / sin(angle) apart.
                    # That only works down to a point: on a gate the wires run
                    # almost along, no position on it can separate them (the
                    # gates they cross squarely do that), and insisting bends
                    # them out of line. Below min_sin the demand fades out.
                    m = sin[s] if sin[s] < sin[s + 1] else sin[s + 1]
                    if m >= min_sin:
                        gap[s] = space[s] / m
                    else:
                        gap[s] = space[s] * m / (min_sin * min_sin)
        moved = 0.0
        for w in range(nw):
            a0, a1 = wire_ptr[w], wire_ptr[w + 1]
            n = a1 - a0
            if slide:
                ax, ay = centre[w, 0], centre[w, 1]
                i0, i1 = 0, n
            else:
                s0 = wire_slot[a0]
                ax = frame[s0, 0] + pos[s0] * frame[s0, 2]
                ay = frame[s0, 1] + pos[s0] * frame[s0, 3]
                i0, i1 = 1, n - 1
            for i in range(i0, i1):
                s = wire_slot[a0 + i]
                ux, uy, dx, dy, L = frame[s, 0], frame[s, 1], frame[s, 2], frame[s, 3], frame[s, 4]
                if i < n - 1:
                    q = wire_slot[a0 + i + 1]
                    bx = frame[q, 0] + pos[q] * frame[q, 2]
                    by = frame[q, 1] + pos[q] * frame[q, 3]
                else:
                    bx, by = centre[w, 2], centre[w, 3]
                rx, ry = bx - ax, by - ay
                den = dx * ry - dy * rx
                if abs(den) < 1e-12:
                    v = pos[s]
                else:
                    v = ((ax - ux) * ry - (ay - uy) * rx) / den
                if i == 0 or i == n - 1:
                    # An end slides along its pad edge towards the straight line
                    # from the pad centre; wires sharing the edge keep their order.
                    lo = pos[s - 1] + share[s] if has_prev[s] else 0.0
                    hi = pos[s + 1] - share[s] if has_next[s] else L
                    if lo <= hi:
                        v = min(max(v, lo), hi)
                    else:
                        v = (lo + hi) / 2.0
                    v = min(max(v, 0.0), L)
                else:
                    lo, hi = wlo[s], whi[s]
                    if has_prev[s]:
                        lo = max(lo, pos[s - 1] + gap[s - 1])
                    if has_next[s]:
                        hi = min(hi, pos[s + 1] - gap[s])
                    if lo > hi:
                        v = (lo + hi) / 2.0  # no room for the spacing here; DRC will report it
                    else:
                        v = min(max(v, lo), hi)
                    # Never leave the window or overtake a neighbour: the order is the topology.
                    lo, hi = wlo[s], whi[s]
                    floor = pos[s - 1] if has_prev[s] else lo
                    ceil = pos[s + 1] if has_next[s] else hi
                    v = min(max(v, floor, lo), ceil, hi)
                d = abs(v - pos[s])
                if d > moved:
                    moved = d
                pos[s] = v
                ax, ay = ux + v * dx, uy + v * dy
        if moved < tol:
            break
    return sweeps
