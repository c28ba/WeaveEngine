"""Exact taut path through a sequence of portals (the funnel algorithm).

A wire crosses its gates in a fixed order; on each gate it may pass anywhere
inside a window. The shortest such path is found in one pass by string pulling.
"""

Point = tuple[float, float]


def _area(a: Point, b: Point, c: Point) -> float:
    """Twice the signed area of a, b, c: positive when c is to the left of a -> b."""
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def string_pull(start: Point, portals: list[tuple[Point, Point]], end: Point) -> list[tuple[int, Point]]:
    """Corners of the shortest path from ``start`` to ``end`` that crosses each
    portal (left point, right point, as seen in the direction of travel) in order.

    Returns (portal index, point) pairs: index -1 for the start, len(portals)
    for the end, otherwise the portal whose end point the path turns on.
    """
    gates = [(start, start)] + list(portals) + [(end, end)]
    out = [(-1, start)]
    apex, left, right = start, start, start
    apex_i = left_i = right_i = 0
    i = 1
    guard = 0
    while i < len(gates):
        guard += 1
        if guard > 20 * len(gates) + 100:
            break  # degenerate input: give up on what is left, the caller polishes
        new_left, new_right = gates[i]
        # Tighten the right side.
        if _area(apex, right, new_right) >= 0.0:
            if apex == right or _area(apex, left, new_right) < 0.0:
                right, right_i = new_right, i
            else:
                # The right side crossed over the left: turn on the left point.
                out.append((left_i - 1, left))
                apex, apex_i = left, left_i
                left, right = apex, apex
                left_i = right_i = apex_i
                i = apex_i + 1
                continue
        # Tighten the left side.
        if _area(apex, left, new_left) <= 0.0:
            if apex == left or _area(apex, right, new_left) > 0.0:
                left, left_i = new_left, i
            else:
                out.append((right_i - 1, right))
                apex, apex_i = right, right_i
                left, right = apex, apex
                left_i = right_i = apex_i
                i = apex_i + 1
                continue
        i += 1
    if out[-1][1] != end:
        out.append((len(portals), end))
    return out
