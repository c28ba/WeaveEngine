"""Console progress bar with an estimate of the time remaining."""
import sys
import time

# Share of one routing pass spent up to the start of each phase (measured on the
# ALU board; only used to turn phase progress into one overall fraction).
PHASES = [
    ("candidates", 0.00),
    ("global selection", 0.04),
    ("commit", 0.10),
    ("rip-up", 0.22),
    ("refinement", 0.62),
    ("geometry", 0.66),
    ("design-rule repair", 0.80),
    ("placing vias", 0.98),
]
PASS_FACTOR = 45.0  # a pass takes roughly this many times its candidate phase


def clock(seconds: float) -> str:
    seconds = max(0, int(round(seconds)))
    return f"{seconds // 3600}:{seconds % 3600 // 60:02d}:{seconds % 60:02d}" if seconds >= 3600 else f"{seconds // 60}:{seconds % 60:02d}"


class ProgressBar:
    """Callable for ``route_board(progress=...)``.

    Shows the pass, the phase, a bar for the current pass and the time left in
    it. A new pass starts whenever vias are added, so the total number of passes
    is not known in advance; the first estimate is printed once the candidate
    phase of a pass has been timed.
    """

    def __init__(self, stream=None, width: int = 28):
        self.stream = stream or sys.stderr
        self.width = width
        self.live = self.stream.isatty()
        self.start = time.time()
        self.pass_start = self.start
        self.pass_name = ""
        self.phase = ""
        self.fraction = 0.0
        self.candidates_at = None
        self.estimate = None
        self.last_draw = 0.0

    def __call__(self, phase: str, done: float = 0.0, total: float = 1.0) -> None:
        now = time.time()
        name, _, phase = phase.rpartition(": ")
        if name != self.pass_name:
            self.pass_name, self.pass_start, self.fraction, self.estimate, self.candidates_at = name, now, 0.0, None, None
            self.phase = ""
        starts = dict(PHASES)
        if phase in starts:
            order = [p for p, _ in PHASES]
            i = order.index(phase)
            lo = starts[phase]
            hi = PHASES[i + 1][1] if i + 1 < len(PHASES) else 1.0
            part = min(1.0, done / total) if total else 0.0
            # Never move backwards: design-rule repair re-enters rip-up.
            self.fraction = max(self.fraction, lo + (hi - lo) * part)
            if phase == "candidates":
                self.candidates_at = now
            elif self.candidates_at is not None and self.estimate is None:
                self.estimate = PASS_FACTOR * (now - self.candidates_at)
                self._line(f"{self.pass_name}: rough estimate for this pass {clock(self.estimate)}")
        if phase.endswith("kept"):
            self.fraction = 1.0  # the race is over: this pass is done
        changed = phase != self.phase
        self.phase = phase
        if changed or now - self.last_draw > (0.5 if self.live else 10.0):
            self.draw(changed or not self.live)

    def remaining(self) -> float | None:
        elapsed = time.time() - self.pass_start
        if self.fraction >= 0.12:
            return elapsed * (1.0 - self.fraction) / self.fraction
        if self.estimate is not None:
            return max(0.0, self.estimate - elapsed)
        return None

    def draw(self, phase_changed: bool = False) -> None:
        self.last_draw = time.time()
        filled = int(round(self.width * self.fraction))
        left = self.remaining()
        text = (f"{self.pass_name or 'setup'} [{'#' * filled}{'.' * (self.width - filled)}] {100 * self.fraction:3.0f}%  "
                f"{self.phase:<19s} elapsed {clock(time.time() - self.start)}  "
                f"left in pass {'~' + clock(left) if left is not None else '?'}")
        if self.live:
            self.stream.write("\r" + text + "\x1b[K")
            self.stream.flush()
        elif phase_changed:
            self._line(text)

    def _line(self, text: str) -> None:
        if self.live:
            self.stream.write("\r" + text + "\x1b[K\n")
        else:
            self.stream.write(text + "\n")
        self.stream.flush()

    def finish(self) -> None:
        if self.live:
            self.stream.write("\r\x1b[K")
            self.stream.flush()
