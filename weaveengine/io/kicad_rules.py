"""KiCad's custom design rules (``.kicad_dru``): the useful subset (design 4.1).

A rule is ``(rule "name" (condition "...") (layer ...) (constraint kind (min X) (max Y) ...))``.
What is read of a condition: comparisons of ``A.NetClass``, ``A.NetName`` and
``A.Reference`` (also ``A.Parent``) with a string (``*`` and ``?`` match, as in
KiCad), ``A.hasNetclass('x')``, ``A.Type == 'Pad'``, ``A.intersectsArea('name')`` and
``A.insideArea('name')`` (a rule area of the board), ``A.insideCourtyard('ref')`` and
``A.intersectsCourtyard('ref')``, joined by ``&&``, ``||``,
``!`` and parentheses. Anything else (``B.``, lengths, differential pairs, ...)
makes the rule skipped, and ``read_rules`` says which. Of the constraints,
``clearance``, ``track_width``, ``via_diameter``, ``hole_clearance`` and
``edge_clearance`` are taken (``io.kicad.apply`` says how); the rest are ignored
without a word, as those rules are of no concern to routing.
"""
import fnmatch
import os
import re
from dataclasses import dataclass, field

TAKEN = ("clearance", "track_width", "via_diameter", "hole_clearance", "edge_clearance")
UNITS = {"mm": 1.0, "mil": 0.0254, "in": 25.4, "um": 0.001, "": 1.0}


class Unsupported(Exception):
    """A rule uses something here that is not read."""


@dataclass
class Rule:
    name: str
    condition: str = ""
    layer: str = ""                                                        # "" = every layer
    limits: dict[str, dict[str, float]] = field(default_factory=dict)      # kind -> {"min": mm, "max": mm}
    test: object = None                                                    # condition -> callable(context) -> bool (None: always)


def _mm(text: str) -> float:
    m = re.fullmatch(r"\s*([-+]?(?:\d+\.?\d*|\.\d+))\s*([a-z]*)\s*", text)
    if not m or m.group(2) not in UNITS:
        raise Unsupported(f"value {text!r}")
    return float(m.group(1)) * UNITS[m.group(2)]


def _tokens(text: str) -> list[str]:
    out = re.findall(r"'(?:[^'\\]|\\.)*'|\"(?:[^\"\\]|\\.)*\"|&&|\|\||==|!=|[()!,]|[A-Za-z_][\w.]*|[^\s]", text)
    return out


class _Condition:
    """Recursive descent over a condition; ``compile`` returns a function of a context (a dict with the keys
    ``net`` (name), ``class``, ``ref`` and ``shape`` of what is tested, ``kind`` and ``areas``), which raises
    Unsupported for a key the context does not have."""

    def __init__(self, text: str):
        self.toks, self.i = _tokens(text), 0

    def peek(self):
        return self.toks[self.i] if self.i < len(self.toks) else None

    def take(self, want=None):
        t = self.peek()
        if t is None or (want is not None and t != want):
            raise Unsupported("condition")
        self.i += 1
        return t

    def compile(self):
        got = self.either()
        if self.peek() is not None:
            raise Unsupported("condition")
        return got

    def either(self):
        parts = [self.both()]
        while self.peek() == "||":
            self.take()
            parts.append(self.both())
        return parts[0] if len(parts) == 1 else (lambda c: any(p(c) for p in parts))

    def both(self):
        parts = [self.single()]
        while self.peek() == "&&":
            self.take()
            parts.append(self.single())
        return parts[0] if len(parts) == 1 else (lambda c: all(p(c) for p in parts))

    def single(self):
        t = self.peek()
        if t == "!":
            self.take()
            inner = self.single()
            return lambda c: not inner(c)
        if t == "(":
            self.take()
            inner = self.either()
            self.take(")")
            return inner
        return self.test()

    def test(self):
        name = self.take()
        if not name.startswith("A."):
            raise Unsupported(name)
        key = name[2:]
        if key in ("intersectsArea", "insideArea", "enclosedByArea"):
            self.take("(")
            area = self.string()
            self.take(")")
            inside = key != "intersectsArea"
            return lambda c: _area(c, area, inside)
        if key in ("insideCourtyard", "intersectsCourtyard"):
            self.take("(")
            ref = self.string()
            self.take(")")
            return lambda c: _yard(c, ref, key == "insideCourtyard")
        if key == "hasNetclass":
            self.take("(")
            want = self.string()
            self.take(")")
            return lambda c: fnmatch.fnmatchcase(_get(c, "class"), want)
        field_ = {"NetClass": "class", "NetName": "net", "Reference": "ref", "Parent": "ref", "Type": "kind"}.get(key)
        if field_ is None:
            raise Unsupported(name)
        op = self.take()
        if op not in ("==", "!="):
            raise Unsupported(name)
        want = self.string()
        if field_ == "kind":
            return lambda c: (_get(c, "kind").lower() == want.lower()) == (op == "==")
        return lambda c: fnmatch.fnmatchcase(_get(c, field_), want) == (op == "==")

    def string(self) -> str:
        t = self.take()
        if len(t) < 2 or t[0] not in "'\"" or t[-1] != t[0]:
            raise Unsupported("condition")
        return t[1:-1]


def _get(context: dict, key: str) -> str:
    if context.get(key) is None:
        raise Unsupported(key)
    return context[key]


def _area(context: dict, name: str, inside: bool) -> bool:
    shape = context.get("shape")
    if shape is None:
        raise Unsupported("area")
    area = context["areas"].get(name)
    if area is None:
        return False
    return area.contains(shape) if inside else area.intersects(shape)


def _yard(context: dict, ref: str, inside: bool) -> bool:
    shape = context.get("shape")
    if shape is None:
        raise Unsupported("courtyard")
    return any((yard.buffer(1e-3).contains(shape) if inside else yard.intersects(shape))
               for name, yard in context["courtyards"].items() if fnmatch.fnmatchcase(name, ref))


def parse_rules(text: str) -> tuple[list[Rule], list[str]]:
    """(rules whose conditions and constraints are understood, names of those skipped, each with why)."""
    from weaveengine.io.dsn import parse_sexpr
    text = "\n".join(l for l in text.splitlines() if not l.lstrip().startswith("#"))
    try:
        root = parse_sexpr("(rules " + text + ")")
    except (ValueError, IndexError):
        return [], ["the file (not readable)"]
    rules, skipped = [], []
    for node in root[1:]:
        if not isinstance(node, list) or not node or node[0] != "rule":
            continue
        name = node[1] if len(node) > 1 and isinstance(node[1], str) else "?"
        rule = Rule(name)
        try:
            for part in node[2:]:
                if isinstance(part, list) and part and part[0] == "constraint" and len(part) > 1 and part[1] in TAKEN:
                    rule.limits[part[1]] = {c[0]: _mm(c[1]) for c in part[2:]
                                            if isinstance(c, list) and len(c) > 1 and c[0] in ("min", "max")}
            if not rule.limits:  # (a rule with no constraint here asks nothing of the routing)
                continue
            for part in node[2:]:
                if isinstance(part, list) and len(part) > 1 and part[0] == "condition":
                    rule.condition = part[1]
                    rule.test = _Condition(part[1]).compile()
                elif isinstance(part, list) and len(part) > 1 and part[0] == "layer":
                    rule.layer = part[1]
        except Unsupported as why:
            skipped.append(f"{name} ({why})")
            continue
        rules.append(rule)
    return rules, skipped


def read_rules(stem: str) -> tuple[list[Rule], list[str]]:
    """The rules of ``stem``.kicad_dru, if it is beside the DSN, and those skipped."""
    path = stem + ".kicad_dru"
    if not os.path.exists(path):
        return [], []
    try:
        with open(path, encoding="utf-8", errors="replace") as f:
            return parse_rules(f.read())
    except OSError:
        return [], []


def last(rules: list[Rule], kind: str, context: dict, unread: set[str]) -> Rule | None:
    """The last rule giving ``kind`` that holds in ``context`` (KiCad: later rules win), or None.
    A rule that tests what the context does not tell is added to ``unread``."""
    got = None
    for rule in rules:
        if kind in rule.limits:
            try:
                if rule.test is None or rule.test(context):
                    got = rule
            except Unsupported:
                unread.add(rule.name)
    return got
