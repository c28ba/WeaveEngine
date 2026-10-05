"""State of the compiled kernels, and a self-test that can never crash.

The search and the relaxation each have a hot loop compiled with numba. If
numba is missing, fails to load, or fails to compile, the same loops run as
plain Python: identical results, many times slower. ``check()`` finds out which
it is, once, and puts it in words for the user. Front ends must call it before
routing and show ``Status.warning`` if there is one.
"""
import time
import warnings
from dataclasses import dataclass

from weaveengine.realize import kernel as relax_kernel
from weaveengine.topo import kernel as search_kernel

SLOWDOWN = "the same results, but roughly 10 to 50 times slower"


@dataclass
class Status:
    compiled: bool       # the compiled kernels are in use
    message: str         # one line, always present
    detail: str = ""     # the underlying error, if any
    seconds: float = 0.0  # time the check took (a first run compiles: about ten seconds)

    @property
    def warning(self) -> str:
        """Text to show before routing, or '' when everything is compiled."""
        if self.compiled:
            return ""
        return (f"WARNING: running without the compiled kernels: {self.message}. Routing will use the "
                f"pure-Python fallback: {SLOWDOWN}." + (f"\n  ({self.detail})" if self.detail else ""))


_status: Status | None = None


def disable(reason: str, detail: str = "") -> Status:
    """Switch to the Python loops for the rest of this process."""
    global _status
    search_kernel.AVAILABLE = False
    search_kernel.WHY_NOT = reason
    for name in ("pull", "reach", "lifts", "inside", "blocked", "heading"):
        plain = getattr(getattr(relax_kernel, name), "py_func", None)
        if plain is not None:
            setattr(relax_kernel, name, plain)
    _status = Status(False, reason, detail)
    return _status


def failed(where: str, error: BaseException) -> None:
    """A compiled kernel raised while routing: fall back and say so, once."""
    disable(f"the compiled {where} kernel failed while running", f"{type(error).__name__}: {error}")
    warnings.warn(_status.warning, RuntimeWarning, stacklevel=2)


def check(force: bool = False) -> Status:
    """Load, compile and run both kernels on a tiny board. Never raises."""
    global _status
    if _status is not None and not force:
        return _status
    _status = None
    start = time.perf_counter()
    if not search_kernel.AVAILABLE:
        _status = Status(False, search_kernel.WHY_NOT or "numba is not installed")
        return _status
    try:
        from weaveengine.board import Board, Pad, Rules
        from weaveengine.realize.relax import relax
        from weaveengine.topo import planar_map
        from weaveengine.topo.search import route
        from weaveengine.topo.state import TopoState

        board = Board.rectangle(10.0, 6.0, Rules(0.2, 0.2))
        board.pads.append(Pad.circle(0, 2.0, 3.0, 0.5, net_id=0))
        board.pads.append(Pad.circle(1, 8.0, 3.0, 0.5, net_id=0))
        pmap = planar_map.build(board)
        state = TopoState(pmap)
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", RuntimeWarning)  # a failure is reported through the status
            found = route(pmap, state, 0, 1)        # compiles and runs the search kernel
            if _status is not None:                 # it failed and fell back (see ``failed``)
                _status.seconds = time.perf_counter() - start
                return _status
            if found is None:
                raise RuntimeError("the compiled search found no path on the self-test board")
            search_kernel.AVAILABLE = False
            try:
                plain = route(pmap, state, 0, 1)    # the Python loop, for comparison
            finally:
                search_kernel.AVAILABLE = True
            if plain is None or plain.gates != found.gates:
                raise RuntimeError("the compiled search disagrees with the Python search on the self-test board")
            state.insert(1, found.steps)
            line = relax(state, board)[1]           # compiles and runs the relaxation kernel
            if _status is not None:
                _status.seconds = time.perf_counter() - start
                return _status
            if len(line) < 2 or abs(line[0][1] - 3.0) > 1e-6 or abs(line[-1][0] - 8.0) > 1e-6:
                raise RuntimeError("the compiled relaxation gave a wrong trace on the self-test board")
    except BaseException as error:  # including errors raised from inside LLVM
        if isinstance(error, (KeyboardInterrupt, SystemExit)):
            raise
        status = disable("numba is installed but its compiled code does not work here", f"{type(error).__name__}: {error}")
        status.seconds = time.perf_counter() - start
        return status
    import numba
    note = ("; " + search_kernel.NOTES[0]) if search_kernel.NOTES else ""
    _status = Status(True, f"compiled kernels ready (numba {numba.__version__}){note}", seconds=time.perf_counter() - start)
    return _status
