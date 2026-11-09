"""Internal board model (design section 4). Units are float64 millimetres."""
import math
from dataclasses import dataclass, field

from shapely import affinity
from shapely.geometry import Point, Polygon, box

EPS = 1e-6


@dataclass
class Rules:
    """Design rules. ``trace_width`` is the default class; ``net_width`` holds
    the widths of nets in other classes. One clearance ``s`` applies to all
    classes (the largest in the design, so it is conservative).

    The values here are the built-in defaults (design section 4): what a DSN
    that does not give a rule is routed with."""
    trace_width: float = 0.2
    clearance: float = 0.2
    edge_clearance: float | None = None  # defaults to ``clearance``
    net_width: dict[int, float] = field(default_factory=dict)
    via_diameter: float = 0.6
    via_drill: float = 0.3
    via_clearance: float = 0.0           # what a via asks of foreign copper beyond ``clearance`` (0: nothing; see ``apart``)
    margin: float = 0.0                  # safety margin, already in ``clearance``; added to every clearance an item asks for itself
    min_width: float = 0.0               # narrowest trace the board's rules allow (0: not known); see ``router.reach``
    net_drawn: dict[int, float] = field(default_factory=dict)  # width written for a net routed narrower than drawn (``router.reach``)
    # A net narrowed to reach fine-pitch pads is planned at its own width but near those pads (``router.reach``):
    # net -> that width, and net -> (x, y, radius) round each of its fine-pitch pads, within which it is narrow.
    net_full: dict[int, float] = field(default_factory=dict)
    fine: dict[int, list] = field(default_factory=dict)

    def width(self, net_id: int) -> float:
        return self.net_width.get(net_id, self.trace_width)

    def drawn(self, net_id: int) -> float:
        """The width a net's traces are written with: its ``width``, unless it
        was routed with some of its margin given up (``router.reach``)."""
        return self.net_drawn.get(net_id, self.width(net_id))

    def full(self, net_id: int) -> float:
        """The width a net is planned at away from its fine-pitch pads: its own, if it was narrowed to reach them."""
        return self.net_full.get(net_id, self.width(net_id))

    def width_at(self, net_id: int, x: float, y: float) -> float:
        """The width a net is planned at at (x, y): ``width`` near its fine-pitch pads, else ``full``."""
        if net_id not in self.net_full:
            return self.width(net_id)
        if any(math.hypot(x - cx, y - cy) <= r for cx, cy, r in self.fine.get(net_id, ())):
            return self.width(net_id)
        return self.net_full[net_id]

    @property
    def base_width(self) -> float:
        """Narrowest trace. The planar map is inflated for this width; wider
        traces carry their extra width with them (see ``extra``)."""
        return min([self.trace_width, *self.net_width.values()])

    def extra(self, net_id: int) -> float:
        """Half of the width a net's trace has beyond the base width."""
        return (self.width(net_id) - self.base_width) / 2.0

    @property
    def pitch(self) -> float:
        return self.base_width + self.clearance

    def apart(self, own: float) -> float:
        """The clearance foreign copper keeps from an item that asks ``own`` of
        it (a local clearance of its part, or what the hole clearance needs where
        its copper ring is narrow), or only the rule's if that is more or ``own``
        is 0. KiCad checks the larger of the two. A negative ``own`` is a custom rule
        that asks exactly ``-own``, less than the rule's (``io.kicad_rules``)."""
        if own < 0:
            return -own + self.margin
        return max(self.clearance, own + self.margin) if own > 0 else self.clearance

    @property
    def inflation(self) -> float:
        """Centreline keep-off distance from foreign copper for a base-width trace: s + t/2."""
        return self.clearance + self.base_width / 2.0

    @property
    def outline_inset(self) -> float:
        ec = self.clearance if self.edge_clearance is None else self.edge_clearance
        return ec + self.base_width / 2.0


@dataclass
class Pad:
    pad_id: int
    net_id: int  # -1 = not connected
    shape: Polygon
    name: str = ""
    layers: frozenset | None = None  # layer indices; None = every layer (through-hole)
    radius: float | None = None      # set for circular pads (teardrops)
    is_via: bool = False
    clearance: float = 0.0           # what it asks of foreign copper beyond the rule (``Rules.apart``); negative: asks only that, less
    fanout: bool = False             # a via put beside a pad to join it to its net's plane (``plan.fanout``)
    anchor: tuple[float, float] | None = None  # where traces join it, if not its centroid (which a bent custom pad's is not on)
    zone_connect: str | None = None  # how a pour joins it, its own or its part's ("thermal", "solid", "none"; ``Zone``): None, the zone's
    thermal_gap: float | None = None  # its own thermal relief gap and spoke width, if it sets them (None: the zone's)
    spoke: float | None = None

    @property
    def centre(self) -> tuple[float, float]:
        if self.anchor is not None:
            return self.anchor
        c = self.shape.centroid
        return (c.x, c.y)

    def on(self, layer: int) -> bool:
        return self.layers is None or layer in self.layers

    @classmethod
    def circle(cls, pad_id: int, x: float, y: float, radius: float, net_id: int = -1, name: str = "",
               layers: frozenset | None = None, is_via: bool = False) -> "Pad":
        return cls(pad_id, net_id, circle_polygon(x, y, radius), name, layers, radius, is_via)

    @classmethod
    def rect(cls, pad_id: int, x: float, y: float, w: float, h: float, net_id: int = -1,
             rotation: float = 0.0, name: str = "", layers: frozenset | None = None) -> "Pad":
        shape = box(x - w / 2.0, y - h / 2.0, x + w / 2.0, y + h / 2.0)
        if rotation:
            shape = affinity.rotate(shape, rotation, origin=(x, y))
        return cls(pad_id, net_id, shape, name, layers)


def circle_polygon(x: float, y: float, radius: float) -> Polygon:
    """16-gon circumscribing the true circle, so polygon clearance is conservative."""
    return Point(x, y).buffer(radius / math.cos(math.pi / 16.0), quad_segs=4)


@dataclass
class Obstacle:
    """Keepout or fixed copper. ``net_id`` -1 means it belongs to no net."""
    shape: Polygon
    net_id: int = -1
    layers: frozenset | None = None
    clearance: float = 0.0           # what it asks of foreign copper beyond the rule (``Rules.apart``)
    copper: bool = False             # copper of no net (text, a logo): kept at the clearance, not only out of
    via: bool = False                # a fixed via: drilled, and a way to its net's plane (``plan.fanout``)

    def on(self, layer: int) -> bool:
        return self.layers is None or layer in self.layers


@dataclass
class Zone:
    """What a pour's zone asks, from the .kicad_pcb (``io.kicad``). None: not
    known; the board's clearance, or KiCad's defaults."""
    clearance: float | None = None   # what its fill keeps from other nets' copper
    gap: float | None = None         # how far its fill keeps from a pad it joins by a thermal relief
    spoke: float | None = None       # the width of the relief's spokes
    connect: str = "thermal"         # how it joins its pads: "thermal", "solid", "none", or "tht" (thermal for pads through the board, solid for the rest)
    priority: int = 0                # where zones of two nets overlap, the higher fills first and the other keeps clear of it


@dataclass
class Component:
    """A placed part, for display and selection (the router itself only needs its pads)."""
    reference: str
    value: str = ""
    footprint: str = ""
    side: str = "front"
    x: float = 0.0
    y: float = 0.0
    rotation: float = 0.0
    outlines: list[list[tuple[float, float]]] = field(default_factory=list)  # polylines, board coordinates
    pads: list[int] = field(default_factory=list)                              # pad ids

    def bounds(self) -> tuple[float, float, float, float] | None:
        pts = [p for line in self.outlines for p in line]
        if not pts:
            return None
        xs, ys = [p[0] for p in pts], [p[1] for p in pts]
        return (min(xs), min(ys), max(xs), max(ys))


@dataclass
class Board:
    outline: Polygon
    rules: Rules = field(default_factory=Rules)
    pads: list[Pad] = field(default_factory=list)
    obstacles: list[Obstacle] = field(default_factory=list)
    net_names: dict[int, str] = field(default_factory=dict)
    layers: list[str] = field(default_factory=lambda: ["F.Cu"])
    unrouted: list[str] = field(default_factory=list)  # copper layers left out of the routing: no trace there, but every via goes
                                                       # through them, so their copper, keepouts and pours are kept, with the
                                                       # layer indices after the routed layers'
    plane_nets: set[int] = field(default_factory=set)  # nets connected by a copper plane: not routed as traces
    skip_nets: set[int] = field(default_factory=set)   # nets left unrouted by the user's choice (a pour, if any, is still fanned out to)
    pours: list = field(default_factory=list)          # (net, layer index, outline) of each of those planes (``plan.fanout``)
    pour_min: dict = field(default_factory=dict)       # (net, layer index) -> the least width of that pour's fill, from its zone's min_thickness
    pour_zone: dict = field(default_factory=dict)      # (net, layer index) -> its zone's settings (``Zone``), from the .kicad_pcb
    pour_override: float = 0.0                         # mm: the least width of every pour's fill if set (the user's), else the zones' own
    margin: list = field(default_factory=list)         # drawings on KiCad's Margin layer: kept clear of by ``plan.fanout`` only
    no_via: list = field(default_factory=list)         # areas no via may touch and traces may cross (a DSN's via_keepout)
    components: list[Component] = field(default_factory=list)

    @classmethod
    def rectangle(cls, width: float, height: float, rules: Rules | None = None, layers: list[str] | None = None) -> "Board":
        return cls(outline=box(0.0, 0.0, width, height), rules=rules or Rules(), layers=layers or ["F.Cu"])

    def add_keepout(self, vertices: list[tuple[float, float]], layers: frozenset | None = None) -> None:
        self.obstacles.append(Obstacle(Polygon(vertices), layers=layers))

    def pour_width(self, net: int, layer: int) -> float:
        """The narrowest the fill of the pour of ``net`` on ``layer`` may be: the user's choice, else its zone's
        minimum thickness, else KiCad's default (0.25 mm)."""
        return self.pour_override or self.pour_min.get((net, layer), 0.25)

    def zone(self, net: int, layer: int) -> Zone:
        """The settings of the zone of the pour of ``net`` on ``layer`` (KiCad's defaults if not known)."""
        return self.pour_zone.get((net, layer)) or Zone()

    def pour_clearance(self, net: int, layer: int) -> float:
        """What the fill of that pour keeps from other nets' copper: its zone's clearance, or the board's,
        whichever is more (KiCad's fill keeps the larger), less the routing margin."""
        rules = self.rules
        zone = self.zone(net, layer).clearance or 0.0
        return max(rules.clearance - rules.margin, zone)

    def pad(self, pad_id: int) -> Pad:
        return {p.pad_id: p for p in self.pads}[pad_id]

    def pads_on(self, layer: int) -> list[Pad]:
        return [p for p in self.pads if p.on(layer)]

    def obstacles_on(self, layer: int) -> list[Obstacle]:
        return [o for o in self.obstacles if o.on(layer)]

    def nets(self) -> dict[int, list[int]]:
        """net id -> pad ids, for nets with at least two pads."""
        nets: dict[int, list[int]] = {}
        for p in self.pads:
            if p.net_id >= 0:
                nets.setdefault(p.net_id, []).append(p.pad_id)
        return {n: pads for n, pads in nets.items() if len(pads) >= 2 and n not in self.plane_nets and n not in self.skip_nets}
