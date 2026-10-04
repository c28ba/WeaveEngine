"""Barrier closure (design section 9.5): union-find over obstacle groups.

Welds are committed wires (joining their two end obstacles) and full gates
(joining their two endpoint obstacles). A weld inside one group closes a loop.
Union-find cannot delete, so this is append-only: rebuild it per rip-up round.
"""
from collections import deque


class Barrier:
    def __init__(self):
        self.parent: dict[int, int] = {}
        self.adj: dict[int, list[tuple[int, object]]] = {}  # forest of welds, with their elements

    def find(self, x: int) -> int:
        parent = self.parent
        root = x
        while parent.setdefault(root, root) != root:
            root = parent[root]
        while parent[x] != root:
            parent[x], x = root, parent[x]
        return root

    def weld(self, a: int, b: int, element) -> bool:
        """Adds a weld. Returns True if it closed a loop (and was therefore not added to the forest)."""
        ra, rb = self.find(a), self.find(b)
        if ra == rb:
            return True
        self.parent[ra] = rb
        self.adj.setdefault(a, []).append((b, element))
        self.adj.setdefault(b, []).append((a, element))
        return False

    def loop(self, a: int, b: int, extra: dict[int, list[tuple[int, object]]] | None = None):
        """Forest path a -> b as (nodes, elements), or None if they are not connected."""
        if a == b:
            return [a], []
        prev: dict[int, tuple[int, object]] = {a: (a, None)}
        queue = deque([a])
        while queue:
            x = queue.popleft()
            for y, element in self.adj.get(x, []) + (extra.get(x, []) if extra else []):
                if y in prev:
                    continue
                prev[y] = (x, element)
                if y == b:
                    nodes, elements = [b], []
                    while y != a:
                        y, element = prev[y]
                        nodes.append(y)
                        elements.append(element)
                    return nodes, elements
                queue.append(y)
        return None

    def trial(self, welds: list[tuple[int, int, object]]) -> list[tuple[list[int], list[object]]]:
        """Loops that ``welds`` would close if added in order, without modifying the barrier."""
        loops = []
        extra: dict[int, list[tuple[int, object]]] = {}
        for a, b, element in welds:
            found = self.loop(a, b, extra)
            if found is not None:
                loops.append((found[0], found[1] + [element]))
            else:
                extra.setdefault(a, []).append((b, element))
                extra.setdefault(b, []).append((a, element))
        return loops
