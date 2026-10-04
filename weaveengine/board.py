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
    classes (the largest in the design, so it is conservative)."""
    trace_width: float = 0.15
    clearance: float = 0.15
    edge_clearance: float | None = None  # defaults to ``clearance``
    net_width: dict[int, float] = field(default_factory=dict)
    via_diameter: float = 0.6
    via_drill: float = 0.3

    def width(self, net_id: int) -> float:
        return self.net_width.get(net_id, self.trace_width)

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

    @property
    def centre(self) -> tuple[float, float]:
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

    def on(self, layer: int) -> bool:
        return self.layers is None or layer in self.layers


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
    plane_nets: set[int] = field(default_factory=set)  # nets connected by a copper plane: not routed as traces
    components: list[Component] = field(default_factory=list)

    @classmethod
    def rectangle(cls, width: float, height: float, rules: Rules | None = None, layers: list[str] | None = None) -> "Board":
        return cls(outline=box(0.0, 0.0, width, height), rules=rules or Rules(), layers=layers or ["F.Cu"])

    def add_keepout(self, vertices: list[tuple[float, float]], layers: frozenset | None = None) -> None:
        self.obstacles.append(Obstacle(Polygon(vertices), layers=layers))

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
        return {n: pads for n, pads in nets.items() if len(pads) >= 2 and n not in self.plane_nets}
