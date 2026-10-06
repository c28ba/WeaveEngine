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
# variant took to reach rip-up ("the start": candidates, selection, commit).
# From the raced variants of the five boards (design section 24).
ROUND = (0.03, 0.5)    # one round of rip-up
TAIL = (0.15, 0.5)     # refinement and geometry
REST = (0.5, 24.0)     # everything after the start, before any of it is seen
REPAIR_ROUNDS = 5      # rounds of rip-up one design-rule repair may take (router._route_once)
REPAIRS = (1, 3)       # design-rule repairs a variant goes through: nearly always one, seldom more than three
# Rounds of rip-up beyond those the stop rule still allows if nothing improves:
# improvements come early, so nine runs in ten stay under EXTRA rounds at the
# start and under half of that for every HALVING rounds gone.
EXTRA, HALVING = 24.0, 8.0
UNEVEN = (0.8, 1.3)    # rounds to come against the last few: they are not all alike
AFTER = 0.03           # smoothing and teardrops, as a share of the run so far
AGAIN = (0.9, 1.15)    # the same board with the same settings, against the time it took before


def clock(seconds: float) -> str:
    seconds = max(0, int(round(seconds)))
    return f"{seconds // 3600}:{seconds % 3600 // 60:02d}:{seconds % 60:02d}" if seconds >= 3600 else f"{seconds // 60}:{seconds % 60:02d}"


def middle(left: tuple[float, float]) -> float:
    """One figure for a range: its middle as ratios go, since the errors are
    ratios (twice as long is as likely as half as long)."""
    return math.sqrt(max(left[0], 0.05 * left[1]) * left[1])


def phrase(left: "tuple[float, float] | None", more: float = 0.0) -> str:
    """The time left as it is shown: the middle figure, and the range it comes
    from unless that is narrow. ``more``: what another heat of variants would
    add, if one may follow."""
    if left is None:
        return ""
    lo, hi = left
    if hi < 1.0:
        text = "nearly done"
    elif hi <= 1.3 * lo + 2.0:
        text = f"about {clock((lo + hi) / 2.0)} left"
    else:
        text = f"about {clock(middle(left))} left ({clock(lo) + ' to ' if lo >= 1.0 else 'up to '}{clock(hi)})"
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
            with open(self.path) as f:
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
            with open(self.path, "w") as f:
                json.dump(dict(list(data.items())[-200:]), f, indent=0)  # the most recent 200
        except OSError:
            pass  # nowhere to write: the next run is estimated like a first one


class _Variant:
    """What is known of one raced variant, from its events."""

    def __init__(self, at: float):
        self.start = at
        self.phase, self.since = "candidates", at
        self.commit = None            # (when it began, share done)
        self.rip = None               # when rip-up began: the start took rip - start
        self.rounds: list[float] = [] # when each round of the present rip-up began
        self.round = None             # a round's duration, lately
        self.quiet = self.most = 1    # rounds still to run, this one included: if none improves; at most
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
            self.quiet, self.most = e.get("quiet", 1), e.get("most", 1)
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

    def left(self, now: float) -> "tuple[float, float] | None":
        """(at least, at most) seconds until this variant has finished; None
        while there is nothing to go on."""
        if self.end is not None:
            return 0.0, 0.0
        if self.rip is None:
            # The start is not over. Commit says how far it is; before that, nothing does.
            if not self.commit or self.commit[1] < 0.1:
                return None
            so_far, share = now - self.start, min(self.commit[1], 1.0)
            rest = (now - self.commit[0]) * (1.0 - share) / share
            start = (so_far + 0.8 * rest, so_far + 2.0 * rest)  # the last connections are the slow ones
            return start[0] - so_far + REST[0] * start[0], start[1] - so_far + REST[1] * start[1]
        start = self.rip - self.start
        in_phase = now - self.since
        if self.round:
            round_ = (UNEVEN[0] * self.round, UNEVEN[1] * self.round)
        else:
            round_ = (max(ROUND[0] * start, 0.5 * in_phase), max(ROUND[1] * start, 2.0 * in_phase))
        tail = (self.tail, self.tail) if self.tail is not None else (TAIL[0] * start, TAIL[1] * start)
        if self.repair:
            repair = (0.6 * self.repair, 1.2 * self.repair)
        else:
            repair = (2 * round_[0] + 0.3 * tail[0], REPAIR_ROUNDS * round_[1] + 0.5 * tail[1])
        lo = hi = 0.0
        if self.phase == "rip-up":
            in_round = now - self.rounds[-1]
            # At least: no round from here improves on the best. At most, in the
            # first rip-up: the improvements still likely. (A repair's is short.)
            extra = 0.0 if self.repairs else EXTRA * 0.5 ** (len(self.rounds) / HALVING)
            lo = max(round_[0] - in_round, 0.0) + (self.quiet - 1) * round_[0]
            hi = max(round_[1] - in_round, 0.0) + (min(self.most, self.quiet + extra) - 1) * round_[1]
        if not self.repairs:
            if self.tail is None:
                waited = now - self.tail_at if self.tail_at is not None else 0.0
                lo += max(tail[0] - waited, 0.0)
                hi += max(tail[1] - waited, 0.25 * waited)
        else:
            lo += 0.3 * tail[0]  # the geometry that follows a repair's rip-up
            hi += 0.5 * tail[1]
        done = len(self.repairs)
        return (lo + max(REPAIRS[0] - done, 0) * repair[0],
                hi + min(max(REPAIRS[1] - done, 1), self.repair_most - done) * repair[1])


class Estimator:
    """Follows a run through its events (``weaveengine.session``) and says how
    long it still has to go, as a range. It need not see every variant: the
    console sees only the plain one."""

    def __init__(self, now: float | None = None, last: float | None = None):
        self.start = time.time() if now is None else now
        self.last = last               # how long this very run took the time before, if it is on record
        self.variants: dict[int, _Variant] = {}
        self.heat = (0, 0)             # the variants racing now: first, one past the last
        self.heat_at = self.start
        self.heats_to_come = 0
        self.heat_took = None          # how long a heat took, if one is over
        self.over = None               # when the last heat ended
        self.fraction = 0.0
        self.phase = ""

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

    def left(self, now: float | None = None) -> "tuple[float, float] | None":
        """(at least, at most) seconds to go if the heat that is running is the
        last (see ``more``), or None before there is anything to go on."""
        now = time.time() if now is None else now
        if self.over is not None:
            return 0.0, max(AFTER * (self.over - self.start) - (now - self.over), 0.0)
        if self.as_before(now):
            return max(AGAIN[0] * self.last - (now - self.start), 0.0), AGAIN[1] * self.last - (now - self.start)
        racing = self._racing()
        if not racing:
            return None
        # The first variant, in their order, that connected everything is taken:
        # the heat is over when that one and those before it have finished.
        taken = min((v for v, x in racing.items() if x.end is not None and x.complete), default=None)
        waited_for = [x.left(now) for v, x in racing.items() if taken is None or v < taken]
        if None in waited_for:
            return None
        lo = max((a for a, _ in waited_for), default=0.0)
        hi = max((b for _, b in waited_for), default=0.0)
        return lo, hi + AFTER * (now - self.start)

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
            return "finishing"
        return (f"variants {self.heat[0] + 1} to {self.heat[1]}: " if self.heat[1] - self.heat[0] > 1 else "") + phase

    def more(self, now: float | None = None) -> float:
        """Seconds another heat would add, if one may follow: about what this
        one takes. Nothing once a variant has connected everything."""
        now = time.time() if now is None else now
        racing = self._racing()
        if (self.over is not None or self.as_before(now) or not self.heats_to_come
                or any(x.end is not None and x.complete for x in racing.values())):
            return 0.0
        left = self.left(now)
        if left is None:
            return 0.0
        return self.heat_took or now - self.heat_at + middle(left)

    def progress(self, now: float | None = None) -> float:
        """Share of the run that is over, for a bar: by the middle of the
        range (with half of another heat, while one may follow), and never
        moving backwards."""
        now = time.time() if now is None else now
        left = self.left(now)
        if left is not None:
            gone = now - self.start
            to_go = middle(left) + 0.5 * self.more(now)
            self.fraction = max(self.fraction, gone / (gone + to_go) if gone + to_go > 0 else 0.0)
        return self.fraction


class ProgressBar:
    """The console's view of a run: ``route_board(events=bar.event)``.

    Shows the phase, a bar, the time gone and the time left. A raced variant
    runs in a process of its own with its own copy of the bar; only the plain
    one draws.
    """

    def __init__(self, stream=None, width: int = 20):
        self.stream = stream or sys.stderr
        self.width = width
        self.live = self.stream.isatty()
        self.estimate = Estimator()
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
