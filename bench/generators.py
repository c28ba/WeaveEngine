"""Synthetic benchmark boards (design section 17.2)."""
import random

from shapely.geometry import LineString

from weaveengine.board import Board, Pad, Rules


def random_grid(nx: int = 8, ny: int = 8, pitch: float = 2.54, nets: int = 16, seed: int = 0,
                solvable: bool = False, rules: Rules | None = None) -> Board:
    """Pads on a jittered grid with random 2-pin nets. ``solvable`` only accepts
    nets whose straight airwires miss every other pad and airwire, so a
    crossing-free routing exists by construction."""
    rng = random.Random(seed)
    rules = rules or Rules(0.2, 0.2)
    margin = pitch
    board = Board.rectangle((nx - 1) * pitch + 2 * margin, (ny - 1) * pitch + 2 * margin, rules)
    radius = pitch * 0.22
    centres = {}
    for i in range(nx):
        for j in range(ny):
            pid = i * ny + j
            centres[pid] = (margin + i * pitch + rng.uniform(-0.1, 0.1) * pitch,
                            margin + j * pitch + rng.uniform(-0.1, 0.1) * pitch)
    free = list(centres)
    rng.shuffle(free)
    assigned: dict[int, int] = {}
    segments: list[LineString] = []
    keep_off = radius + rules.inflation + rules.pitch
    attempts = 0
    while len(segments) < nets and len(free) >= 2 and attempts < 50 * nets:
        attempts += 1
        a, b = rng.sample(free, 2)
        seg = LineString([centres[a], centres[b]])
        if solvable:
            if any(seg.distance(s) < 2 * rules.pitch for s in segments):
                continue
            if any(p not in (a, b) and seg.distance(LineString([c, c])) < keep_off for p, c in centres.items()):
                continue
        net = len(segments)
        assigned[a] = assigned[b] = net
        segments.append(seg)
        free.remove(a)
        free.remove(b)
    for pid, (x, y) in centres.items():
        board.pads.append(Pad.circle(pid, x, y, radius, assigned.get(pid, -1)))
    return board


def escape(n: int = 5, pitch: float = 2.0, seed: int = 0, rules: Rules | None = None) -> Board:
    """Dense n x n pad array; every pad must reach its own pad on the periphery."""
    rng = random.Random(seed)
    rules = rules or Rules(0.2, 0.2)
    count = n * n
    per_side = -(-count // 4)
    side = max((n + 5) * pitch, (per_side + 2) * pitch * 0.7)
    board = Board.rectangle(side, side, rules)
    radius = pitch * 0.2
    origin = (side - (n - 1) * pitch) / 2.0
    for i in range(n):
        for j in range(n):
            board.pads.append(Pad.circle(i * n + j, origin + i * pitch, origin + j * pitch, radius, i * n + j))
    order = list(range(count))
    rng.shuffle(order)
    inset = pitch * 0.9
    for idx, net in enumerate(order):
        edge, k = divmod(idx, per_side)
        along = inset + (k + 0.5) * (side - 2 * inset) / per_side
        x, y = [(along, inset), (side - inset, along), (side - along, side - inset), (inset, side - along)][edge]
        board.pads.append(Pad.rect(count + idx, x, y, radius * 2, radius * 2, net))
    return board


def channel(n: int = 8, pitch: float = 2.0, height: float = 8.0, seed: int = 0, rules: Rules | None = None) -> Board:
    """Classic channel: a top row and a bottom row of pins joined by a random permutation."""
    rng = random.Random(seed)
    rules = rules or Rules(0.2, 0.2)
    board = Board.rectangle((n + 3) * pitch, height + 4 * pitch, rules)
    perm = list(range(n))
    rng.shuffle(perm)
    radius = pitch * 0.2
    for i in range(n):
        board.pads.append(Pad.circle(i, (i + 2) * pitch, 2 * pitch, radius, i))
        board.pads.append(Pad.circle(n + i, (i + 2) * pitch, 2 * pitch + height, radius, perm[i]))
    return board


def smd_two_layer(n: int = 6, pitch: float = 2.0, seed: int = 0, rules: Rules | None = None) -> Board:
    """Two rows of front-only (surface-mount) pads joined by a random permutation,
    with a second layer available: crossings have to be resolved with vias."""
    rng = random.Random(seed)
    rules = rules or Rules(0.2, 0.2)
    board = Board.rectangle((n + 3) * pitch, 8 * pitch, rules, layers=["F.Cu", "B.Cu"])
    perm = list(range(n))
    rng.shuffle(perm)
    front = frozenset({0})
    for i in range(n):
        board.pads.append(Pad.rect(i, (i + 2) * pitch, 2 * pitch, pitch * 0.5, pitch * 0.4, i, layers=front))
        board.pads.append(Pad.rect(n + i, (i + 2) * pitch, 6 * pitch, pitch * 0.5, pitch * 0.4, perm[i], layers=front))
    # A wall on the front layer between the rows.
    board.add_keepout([(0, 3.9 * pitch), ((n + 3) * pitch, 3.9 * pitch), ((n + 3) * pitch, 4.1 * pitch), (0, 4.1 * pitch)], layers=front)
    return board


def real_boards() -> dict[str, Board]:
    """Boards exported from a CAD tool, if present in the repository root."""
    import os
    from weaveengine.io.dsn import read_dsn
    path = os.path.join(os.path.dirname(os.path.dirname(__file__)), "boards", "Word of RAM.dsn")
    return {"word_of_ram (KiCad, 2 layers)": read_dsn(path).board} if os.path.exists(path) else {}


def suite() -> dict[str, Board]:
    return {
        "grid_6x6_solvable": random_grid(6, 6, nets=10, seed=1, solvable=True),
        "grid_8x8_solvable": random_grid(8, 8, nets=16, seed=2, solvable=True),
        "grid_6x6_random": random_grid(6, 6, nets=8, seed=3),
        "grid_8x8_random": random_grid(8, 8, nets=14, seed=4),
        "grid_10x10_random": random_grid(10, 10, nets=20, seed=5),
        "escape_4x4": escape(4, seed=6),
        "escape_5x5": escape(5, seed=7),
        "channel_6": channel(6, seed=8),
        "channel_10": channel(10, seed=9),
        "smd_two_layer_6": smd_two_layer(6, seed=10),
        **real_boards(),
    }
