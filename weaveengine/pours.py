"""An estimate of the copper KiCad will fill in each pour of a routing (design section 11.3):
the pour's outline less every other net's copper and its clearance, without the slivers, in
pieces; the piece with most of the net's pads is the main one, and the others hold pads that
are cut off, or none (islands)."""
from dataclasses import dataclass, field

import shapely
from shapely.geometry import LineString

from weaveengine.plan.fanout import FILL


@dataclass
class Piece:
    shape: object                                   # a Polygon
    kind: str                                       # "main", "cut" (holds pads of the net, none of them reached), "via" (a pad through the board or a via joins it to the main piece) or "island"
    pads: list[str] = field(default_factory=list)   # names of the net's pads in it (vias not among them)
    pad_ids: set = field(default_factory=set)       # and the ids of those and of its vias


@dataclass
class Pour:
    net: int
    layer: int
    outline: object
    pieces: list[Piece] = field(default_factory=list)

    @property
    def cut_off(self) -> list[str]:
        return [name for piece in self.pieces if piece.kind == "cut" for name in piece.pads]

    @property
    def islands(self) -> int:
        return sum(1 for piece in self.pieces if piece.kind == "island")


def estimate(board, lines: dict, wire_net: dict, wire_layer: dict, teardrops: dict | None = None,
             clearance: float | None = None, min_width: float = FILL) -> list[Pour]:
    """The estimated fill of each pour of ``board`` (its vias among its pads) for the routed
    ``lines``. ``clearance``: the zones' (default: the board's, less the routing margin)."""
    rules = board.rules
    base = rules.clearance - rules.margin if clearance is None else clearance
    edge = (rules.clearance if rules.edge_clearance is None else rules.edge_clearance) - rules.margin
    inside = board.outline.buffer(-max(edge, base))
    layers = {at for _, at, _ in board.pours}
    # Each layer's copper: (shape, net, clearance asked; None for a keepout, which asks none).
    copper: dict[int, list] = {at: [] for at in layers}
    for p in board.pads:
        for at in layers:
            if p.on(at):
                copper[at].append((p.shape, p.net_id, max(base, p.clearance)))
    for o in board.obstacles:
        for at in layers:
            if o.on(at):
                copper[at].append((o.shape, o.net_id if o.copper or o.net_id >= 0 else -2, max(base, o.clearance) if o.copper or o.net_id >= 0 else 0.0))
    for w, line in lines.items():
        if len(line) >= 2 and wire_layer.get(w) in layers:
            net = wire_net[w]
            copper[wire_layer[w]].append((LineString(line).buffer(rules.drawn(net) / 2.0, quad_segs=4), net, base))
    for w, drops in (teardrops or {}).items():
        if wire_layer.get(w) in layers:
            for poly in drops:
                copper[wire_layer[w]].append((shapely.Polygon(poly[:5]).buffer(0), wire_net[w], base))
    trees = {at: shapely.STRtree([c[0] for c in cs]) for at, cs in copper.items() if cs}
    pours = []
    for net, at, outline in board.pours:
        fill = outline.intersection(inside)
        tree = trees.get(at)
        if tree is not None:
            near = [copper[at][i] for i in tree.query(outline, predicate="dwithin", distance=base + 1.0).tolist()]
            held = [s.buffer(ask, quad_segs=4) for s, other, ask in near if other != net]
            if held:
                fill = fill.difference(shapely.union_all(held))
        own = [p for p in board.pads if p.net_id == net and p.on(at) and p.shape.intersects(outline)]
        # (the net's own pads are copper whatever their width)
        fill = shapely.union_all([fill.buffer(-min_width / 2.0).buffer(min_width / 2.0), *[p.shape.intersection(inside) for p in own]])
        parts = sorted(getattr(fill, "geoms", [fill]), key=lambda g: -g.area)
        pour = Pour(net, at, outline)
        has = [[p for p in own if g.intersects(p.shape)] for g in parts]
        main = max(range(len(parts)), key=lambda k: (sum(not p.is_via for p in has[k]), parts[k].area), default=None)
        for k, g in enumerate(parts):
            if g.geom_type == "Polygon" and not g.is_empty:
                names = [p.name or str(p.pad_id) for p in has[k] if not p.is_via]
                pour.pieces.append(Piece(g, "main" if k == main and names else "", names, {p.pad_id for p in has[k]}))
        pours.append(pour)
    _join(pours)
    return pours


def _join(pours: list[Pour]) -> None:
    """Gives the pieces their kind. A pad through the board or a via is in a piece of each layer's pour of its net:
    pieces that share one are joined, and the joined pieces that hold a main piece are connected (KiCad sees the
    pad connect them); the rest hold pads that are cut off, or none."""
    for net in {pour.net for pour in pours}:
        pieces = [piece for pour in pours if pour.net == net for piece in pour.pieces]
        group = list(range(len(pieces)))

        def find(i: int) -> int:
            while group[i] != i:
                group[i] = group[group[i]]
                i = group[i]
            return i

        first: dict[int, int] = {}
        for i, piece in enumerate(pieces):
            for pad in piece.pad_ids:
                group[find(i)] = find(first.setdefault(pad, i))
        joined = {find(i) for i, piece in enumerate(pieces) if piece.kind == "main"}
        for i, piece in enumerate(pieces):
            if piece.kind != "main":
                piece.kind = "via" if find(i) in joined and piece.pad_ids else "cut" if piece.pads else "island"


def of_result(result) -> list[Pour]:
    return estimate(result.board, result.polylines, result.wire_net, result.wire_layer, result.teardrops)


def note(pour: Pour, board) -> str:
    """One line: ``GND on B.Cu in 3 pieces: 2 pads cut off (U1-4, C3-2), 1 island``."""
    names = [*board.layers, *board.unrouted]
    cut, islands = pour.cut_off, pour.islands
    bits = []
    if cut:
        bits.append(f"{len(cut)} pad{'s' if len(cut) > 1 else ''} cut off ({', '.join(cut[:8])}{f' and {len(cut) - 8} more' if len(cut) > 8 else ''})")
    if islands:
        bits.append(f"{islands} island{'s' if islands > 1 else ''}")
    count = len(pour.pieces)
    return (f"{board.net_names.get(pour.net, pour.net)} on {names[pour.layer] if pour.layer < len(names) else pour.layer} in "
            f"{count} piece{'s' if count != 1 else ''}" + (": " + ", ".join(bits) if bits else ""))
