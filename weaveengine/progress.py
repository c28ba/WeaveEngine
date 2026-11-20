"""How long a run still has to go: an estimate as a range, and a console bar that shows it.

How long a run takes is not known in advance, because three things in it go
on until a rule says stop (design section 24):

- rip-up runs until some rounds in a row bring no improvement;
- design-rule repair is repeated while the check finds something, four times at most;
- a heat of raced variants is followed by another if none of them connected everything.

So the estimate is a range, and the third of these is said in words: the
range is for the heat that is running, with what another heat would add
beside it. Every duration in it is one this run has measured itself (a round,
the geometry, a repair, a heat) as soon as there is one. What is not measured
but counted, the rounds and repairs still to come, runs from what the stop
rules make the least to what the runs on record make likely.
"""
import hashlib
import json
import math
import os
import re
import sys
import time

# Before a run has measured them itself: durations as shares of what its
# variant took to reach rip-up ("the start": candidates, selection, commit),
# or of a round of rip-up. From the raced variants of 33 boards (design
# section 24): the ranges hold eight in ten of them.
ROUND = (0.04, 0.8)    # one round of rip-up, against the start
TAIL = (2.7, 8.0)      # refinement and geometry, against the mean of a round and the start (sqrt(round * start))
REPAIR = (0.6, 2.4)    # one design-rule repair and the refinement after it, against the first refinement
REST = (0.5, 24.0)     # everything after the start, before any of it is seen
REPAIRS = (1, 3)       # design-rule repairs a variant goes through: nearly always one, seldom more than three
# The finishing after the routing (``router._Steps``: corners, blunting,
# teardrops, joining the pours), in seconds a pad: it is the pours that take
# the time. Said by the router's "setup" event; without it, a share of the run.
FINISH = (0.004, 0.03)         # a board with no pour
FINISH_POURED = (0.015, 0.12)  # a board with pours
# Whether a round of rip-up improves on the best, and so puts back the rounds
# the stop rule allows: after round m, k rounds since the last improvement, in
# a rip-up that began with V violations, the chance is
# CHANCE * exp(-m / (SCALE * ln(2 + V))) * STALE ** k. Fitted on the 232
# rip-ups of 33 boards (design section 24).
CHANCE, SCALE, STALE = 0.9, 2.0, 0.85
LIKELY = 0.9           # the most rounds of rip-up counted: nine runs in ten have ended by then
UNEVEN = (0.8, 1.3)    # rounds to come against the last few: they are not all alike
AFTER = 0.03           # smoothing and teardrops, as a share of the run so far
AGAIN = (0.9, 1.15)    # the same board with the same settings, against the time it took before
NEXT_HEAT = (0.6, 1.3)  # another heat, when one is likely, against the one running (heats of a run are much alike)


def clock(seconds: float) -> str:
    seconds = max(0, int(round(seconds)))
    return f"{seconds // 3600}:{seconds % 3600 // 60:02d}:{seconds % 60:02d}" if seconds >= 3600 else f"{seconds // 60}:{seconds % 60:02d}"


def middle(left: tuple[float, float]) -> float:
    """One figure for a range: its middle as ratios go, since the errors are
    ratios (twice as long is as likely as half as long)."""
    return math.sqrt(max(left[0], 0.05 * left[1]) * left[1])


def spread(lo: float, hi: float) -> tuple[float, float, float]:
    """A range with its middle: (at least, middle, at most)."""
    return lo, middle((lo, hi)), hi


def total(*parts: tuple[float, float, float]) -> tuple[float, float, float]:
    """The time left is a sum of parts, and so is its middle: a part that is
    better known than by its range (rip-up's rounds) keeps its own middle."""
    return tuple(sum(x) for x in zip(*parts))


def phrase(left: "tuple[float, float, float] | None", more: float = 0.0) -> str:
    """The time left as it is shown: the middle figure, and the range it comes
    from unless that is narrow. ``left``: (at least, middle, at most).
    ``more``: what another heat of variants would add, if one may follow."""
    if left is None:
        return ""
    lo, mid, hi = left
    if hi < 1.0:
        text = "nearly done"
    elif hi <= 1.3 * lo + 2.0:
        text = f"about {clock((lo + hi) / 2.0)} left"
    else:
        text = f"about {clock(mid)} left ({clock(lo) + ' to ' if lo >= 1.0 else 'up to '}{clock(hi)})"
    return text + (f", and about {clock(more)} more if no variant connects everything" if more >= 1.0 else "")


class Timings:
    """How long each board took the last time, by the board file and the
    settings it was routed with: the one thing that says closely how long it
    will take again. A small file beside the settings."""

    def __init__(self, path: str):
        self.path = path

    @staticmethod
    def key(dsn_path: str, settings) -> str:
        """Names a run: the file's contents and every setting (``settings``: a
        dict). The same key means the same work."""
        digest = hashlib.sha1()
        with open(dsn_path, "rb") as f:
            digest.update(f.read())
        digest.update(json.dumps(settings, sort_keys=True, default=str).encode())
        return digest.hexdigest()

    def _read(self) -> dict:
        try:
            with open(self.path, encoding="utf-8") as f:
                data = json.load(f)
            return data if isinstance(data, dict) else {}
        except (OSError, ValueError):
            return {}

    def last(self, key: str) -> float | None:
        seconds = self._read().get(key)
        return float(seconds) if isinstance(seconds, (int, float)) and seconds > 0 else None

    def record(self, key: str, seconds: float) -> None:
        data = self._read()
        data.pop(key, None)
        data[key] = round(seconds, 2)
        try:
            with open(self.path, "w", encoding="utf-8") as f:
                json.dump(dict(list(data.items())[-200:]), f, indent=0)  # the most recent 200
        except OSError:
            pass  # nowhere to write: the next run is estimated like a first one


def rounds_left(n: int, quiet: int, limit: int, violations: float, most: int) -> tuple[int, int, int]:
    """The rounds of rip-up still to run, this one (round ``n``) included: at
    least ``quiet`` (if none improves on the best), as likely as not more than
    the middle, and at most what nine runs in ten stay under. An improvement
    puts back ``limit`` rounds, but improvements come less often the longer
    rip-up has run (later on a board with more to clear) and the longer since
    the last one: so the middle falls as rip-up goes on, as the truth does."""
    scale = SCALE * math.log(2.0 + violations)
    stalled = {limit - quiet: 1.0}  # rounds since the last improvement: how likely each
    over, mid = 0.0, None           # how likely rip-up has ended
    for r in range(1, most + 1):
        better, nxt = CHANCE * math.exp(-(n + r - 1) / scale), {}
        for k, w in stalled.items():
            p = min(better * STALE ** k, 1.0)
            nxt[0] = nxt.get(0, 0.0) + w * p
            if k + 1 < limit:
                nxt[k + 1] = nxt.get(k + 1, 0.0) + w * (1.0 - p)
            else:
                over += w * (1.0 - p)  # the stop rule ends it after this round
        stalled = nxt
        mid = mid or (r if over >= 0.5 else None)
        if over >= LIKELY:
            return quiet, mid, r
    return quiet, mid or most, most


class _Variant:
    """What is known of one raced variant, from its events."""

    def __init__(self, at: float):
        self.start = at
        self.phase, self.since = "candidates", at
        self.commit = None            # (when it began, share done)
        self.rip = None               # when rip-up began: the start took rip - start
        self.rounds: list[float] = [] # when each round of the present rip-up began
        self.round = None             # a round's duration, lately
        self.violations = 1           # when the present rip-up began
        self.limit = 1                # the rounds an improvement puts back: the most still to run seen
        self.span = (1, 1, 1)         # rounds of rip-up still to run, this one included (``rounds_left``)
        self.tail_at = None           # when refinement (or geometry) began
        self.tail = None              # how long they took
        self.repairs: list[float] = []  # when each design-rule repair began
        self.repair_most = 4
        self.repair = None            # the longest repair so far
        self.end = None
        self.complete = False
        self.open = None              # connections not routed, by its last picture

    def event(self, e: dict, at: float) -> None:
        phase = e["phase"]
        if phase == "commit":
            self.commit = (self.commit[0] if self.commit else at, e["done"] / e["total"] if e["total"] else 0.0)
        elif phase == "rip-up":
            if self.rip is None:
                self.rip = at
            self.rounds.append(at)
            if len(self.rounds) > 1:
                last = self.rounds[-4:]
                self.round = (last[-1] - last[0]) / (len(last) - 1)
            if len(self.rounds) == 1:  # a rip-up begins (the first, or a repair's)
                self.violations, self.limit = e["total"], 1
            self.limit = max(self.limit, e.get("quiet", 1))
            self.span = rounds_left(len(self.rounds), e.get("quiet", 1), self.limit, self.violations, e.get("most", 1))
        elif phase in ("refinement", "geometry") and self.tail_at is None:
            self.tail_at = at
        elif phase in ("design-rule repair", "finished"):
            if self.repairs:
                self.repair = max(self.repair or 0.0, at - self.repairs[-1])
            elif self.tail_at is not None:
                self.tail = at - self.tail_at
            if phase == "finished":
                self.end, self.complete = at, bool(e.get("complete"))
            else:
                self.repairs.append(at)
                self.repair_most = int(e["total"])
                self.rounds = []
        if phase != self.phase:
            self.phase, self.since = phase, at

    def left(self, now: float) -> "tuple[float, float, float] | None":
        """(at least, middle, at most) seconds until this variant has
        finished; None while there is nothing to go on."""
        if self.end is not None:
            return 0.0, 0.0, 0.0
        if self.rip is None:
            # The start is not over. Commit says how far it is; before that, nothing does.
            if not self.commit or self.commit[1] < 0.1:
                return None
            so_far, share = now - self.start, min(self.commit[1], 1.0)
            rest = (now - self.commit[0]) * (1.0 - share) / share
            start = (so_far + 0.8 * rest, so_far + 2.0 * rest)  # the last connections are the slow ones
            return spread(start[0] - so_far + REST[0] * start[0], start[1] - so_far + REST[1] * start[1])
        start = self.rip - self.start
        in_phase = now - self.since
        if self.round:
            round_ = (UNEVEN[0] * self.round, UNEVEN[1] * self.round)
        else:
            round_ = (max(ROUND[0] * start, 0.5 * in_phase), max(ROUND[1] * start, 2.0 * in_phase))
        if self.tail is not None:
            tail = (self.tail, self.tail)
        else:
            unit = math.sqrt(max(start, 1e-3) * (self.round or math.sqrt(round_[0] * round_[1])))
            tail = (TAIL[0] * unit, TAIL[1] * unit)
        if self.repair:
            repair = (0.6 * self.repair, 1.2 * self.repair)
        else:
            repair = (REPAIR[0] * tail[0], REPAIR[1] * tail[1])
        rip = (0.0, 0.0, 0.0)
        if self.phase == "rip-up" and not self.repairs:  # (a repair's own rip-up is in the repair's time)
            # With the middle of its rounds, not of its range: the low end comes back with every improvement.
            in_round = now - self.rounds[-1]
            rip = tuple(max(d - in_round, 0.0) + (r - 1) * d for d, r in zip(spread(*round_), self.span))
        most = min(REPAIRS[1], self.repair_most)
        if not self.repairs:
            lo, hi = REPAIRS[0] * repair[0], most * repair[1]
            if self.tail is None:
                waited = now - self.tail_at if self.tail_at is not None else 0.0
                lo += max(tail[0] - waited, 0.0)
                hi += max(tail[1] - waited, 0.25 * waited)
        else:
            waited = now - self.repairs[-1]  # the repair under way, and those that may follow it
            lo = max(repair[0] - waited, 0.0)
            hi = max(repair[1] - waited, 0.25 * waited) + max(most - len(self.repairs), 0) * repair[1]
        return total(rip, spread(lo, hi))


class Estimator:
    """Follows a run through its events (``weaveengine.session``) and says how
    long it still has to go, as a range. It need not see every variant: the
    console sees only the plain one."""

    def __init__(self, now: float | None = None, last: float | None = None, deadline: float | None = None):
        self.start = time.time() if now is None else now
        self.last = last               # how long this very run took the time before, if it is on record
        self.deadline = deadline       # the time limit, as a time on the clock: nothing is bettered past it (``Context.late``)
        self.variants: dict[int, _Variant] = {}
        self.heat = (0, 0)             # the variants racing now: first, one past the last
        self.heat_at = self.start
        self.heats_to_come = 0
        self.heat_took = None          # how long a heat took, if one is over
        self.over = None               # when the last heat ended
        self.fraction = 0.0
        self.phase = ""
        self.finish = None             # (at least, at most) seconds of finishing, once the router has said the board's size
        self.finishing = None          # when the finishing began (``router._Steps``)

    def event(self, e: dict, now: float | None = None) -> None:
        now = time.time() if now is None else now
        kind = e.get("type")
        if kind == "snapshot" and e["variant"] in self.variants:
            self.variants[e["variant"]].open = e["total"] - e["routed"]
        if kind == "pass":
            self.over = self.over or now
        if kind != "progress":
            return
        phase = e["phase"]
        if phase == "setup":  # the board's size: what the finishing takes goes by it
            share = FINISH_POURED if e.get("poured") else FINISH
            self.finish = (share[0] * e.get("pads", 0), share[1] * e.get("pads", 0))
            return
        if phase.startswith("finishing"):
            self.over = self.over or now
            self.finishing = self.finishing or now
            self.phase = phase
            return
        heat = re.fullmatch(r"racing variants (\d+)-(\d+) of (\d+)", phase)
        if heat:
            first, last, count = (int(x) for x in heat.groups())
            if first > 1:
                self.heat_took = now - self.heat_at
            self.heat, self.heat_at = (first - 1, last), now
            self.heats_to_come = -(-(count - last) // max(1, last - first + 1))
        elif phase.endswith("kept"):
            self.over = now
        else:
            v = e.get("variant", 0)
            if phase == "candidates" or v not in self.variants:
                self.variants[v] = _Variant(now)
            self.variants[v].event(e, now)
            if v <= 0:
                self.phase = phase

    def _racing(self) -> dict:
        return {v: x for v, x in self.variants.items() if self.heat[0] <= v < max(self.heat[1], self.heat[0] + 1)} or self.variants

    def left(self, now: float | None = None) -> "tuple[float, float, float] | None":
        """(at least, middle, at most) seconds to go if the heat that is running is the
        last (see ``more``), or None before there is anything to go on. With a
        time limit, never past it: what is done after it (the geometry of the
        best so far, its check) is not counted."""
        now = time.time() if now is None else now
        left = self._left(now)
        if left is None or self.deadline is None:
            return left
        limit = max(self.deadline - now, 0.0)
        return tuple(min(x, limit) for x in left)

    def _finish(self, now: float) -> tuple[float, float, float]:
        """(at least, middle, at most) seconds of finishing still to come."""
        if self.finish is None:
            return spread(0.0, AFTER * ((self.over or now) - self.start))
        waited = now - (self.finishing or self.over or now)
        return spread(max(self.finish[0] - waited, 0.0), max(self.finish[1] - waited, 0.25 * waited))

    def _left(self, now: float) -> "tuple[float, float, float] | None":
        if self.over is not None:
            return self._finish(now)
        if self.as_before(now):
            lo, hi = max(AGAIN[0] * self.last - (now - self.start), 0.0), AGAIN[1] * self.last - (now - self.start)
            return lo, (lo + hi) / 2.0, hi
        racing = self._racing()
        if not racing:
            return None
        # The first variant, in their order, that connected everything is taken:
        # the heat is over when that one and those before it have finished.
        taken = min((v for v, x in racing.items() if x.end is not None and x.complete), default=None)
        waited_for = [x.left(now) for v, x in racing.items() if taken is None or v < taken]
        if None in waited_for:
            return None
        heat = tuple(max(x, default=0.0) for x in zip(*waited_for)) if waited_for else (0.0, 0.0, 0.0)  # the slowest, by each figure
        parts = [heat, self._finish(now)]
        if self._again(racing):  # another heat is likely: it is counted in, not only said
            took = self.heat_took or now - self.heat_at + heat[1]
            parts.append(spread(NEXT_HEAT[0] * took, NEXT_HEAT[1] * took))
        return total(*parts)

    def _again(self, racing: dict) -> bool:
        """Whether another heat of variants is likely to follow this one: one
        may, and every variant in this one still has connections open by its
        latest picture (a board that cannot be routed whole runs every heat)."""
        return (self.heats_to_come > 0 and len(racing) > 1 and all(x.open is not None and x.open > 0 for x in racing.values())
                and not any(x.end is not None and x.complete for x in racing.values()))

    def as_before(self, now: float | None = None) -> bool:
        """Whether the estimate is the time the same run took before (it is,
        until the run has outlasted that)."""
        now = time.time() if now is None else now
        return self.last is not None and self.over is None and now - self.start < AGAIN[1] * self.last

    def status(self) -> str:
        """What is going on, in a few words: the heat, and the phase of the first variant in it still at work."""
        racing = self._racing()
        phase = next((x.phase for _, x in sorted(racing.items()) if x.end is None), self.phase)
        if self.over is not None:
            return self.phase if self.phase.startswith("finishing") else "finishing"
        if self.deadline is not None and time.time() >= self.deadline:
            return "time is up: " + phase
        return (f"variants {self.heat[0] + 1} to {self.heat[1]}: " if self.heat[1] - self.heat[0] > 1 else "") + phase

    def more(self, now: float | None = None) -> float:
        """Seconds another heat would add, if one may follow: about what this
        one takes. Nothing once a variant has connected everything."""
        now = time.time() if now is None else now
        racing = self._racing()
        if (self.over is not None or self.as_before(now) or not self.heats_to_come or self._again(racing)
                or any(x.end is not None and x.complete for x in racing.values())):
            return 0.0
        left = self.left(now)
        if left is None:
            return 0.0
        more = self.heat_took or now - self.heat_at + left[1]
        if self.deadline is not None:  # (a heat starts only before the time limit, and stops at it)
            more = min(more, max(self.deadline - now - left[1], 0.0))
        return more

    def progress(self, now: float | None = None) -> float:
        """Share of the run that is over, for a bar: by the middle of the
        range (with half of another heat, while one may follow), and never
        moving backwards."""
        now = time.time() if now is None else now
        left = self.left(now)
        if left is not None:
            gone = now - self.start
            to_go = left[1] + 0.5 * self.more(now)
            self.fraction = max(self.fraction, gone / (gone + to_go) if gone + to_go > 0 else 0.0)
        return self.fraction


class ProgressBar:
    """The console's view of a run: ``route_board(events=bar.event)``.

    Shows the phase, a bar, the time gone and the time left. A raced variant
    runs in a process of its own with its own copy of the bar; only the plain
    one draws.
    """

    def __init__(self, stream=None, width: int = 20, deadline: float | None = None):
        self.stream = stream or sys.stderr
        self.width = width
        self.live = self.stream.isatty()
        self.estimate = Estimator(deadline=deadline)
        self.phase = None
        self.last_draw = 0.0

    def event(self, e: dict) -> None:
        if e.get("type") != "progress" or e.get("variant", 0) > 0:
            return
        self.estimate.event(e)
        changed = e["phase"] != self.phase
        self.phase = e["phase"]
        if changed or time.time() - self.last_draw > (0.5 if self.live else 10.0):
            self.draw(changed or not self.live)

    def draw(self, phase_changed: bool = False) -> None:
        now = self.last_draw = time.time()
        share = self.estimate.progress(now)
        filled = int(round(self.width * share))
        more = self.estimate.more(now)  # said briefly: a console line must not wrap
        text = (f"[{'#' * filled}{'.' * (self.width - filled)}] {100 * share:3.0f}%  {self.estimate.status() or 'setup':<32s} "
                f"{clock(now - self.estimate.start)} gone, {phrase(self.estimate.left(now)) or 'timing it'}"
                + (f", or {clock(more)} more" if more >= 1.0 else ""))
        if self.live:
            self.stream.write("\r" + text + "\x1b[K")
            self.stream.flush()
        elif phase_changed:
            self.stream.write(text + "\n")
            self.stream.flush()

    def finish(self) -> None:
        if self.live:
            self.stream.write("\r\x1b[K")
            self.stream.flush()
