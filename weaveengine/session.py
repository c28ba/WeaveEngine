"""Running the router for a front end: in its own process, with live events and cancel.

A ``Job`` routes one DSN in a separate process, so the front end stays
responsive and can stop the run. Events arrive through ``Job.poll()`` as dicts:

    {"type": "status", "compiled": bool, "message": str, "warning": str}   compiled-kernel check
    {"type": "note", "text": str}                                           rules in use, planes skipped, ...
    {"type": "board", "design": Design}                                     the board as it will be routed
    {"type": "progress", "pass", "variant", "phase", "done", "total"}       and, for the estimate of the time left
                                                                            (weaveengine.progress): "round", "quiet",
                                                                            "most" with rip-up; "complete" with finished
    {"type": "snapshot", "pass", "variant", "final", "wires", "open", "vias",
     "routed", "total", "overflow", "rounds", "length", "per_layer"}        the routing as it stands
    {"type": "pass", "pass", "open", "connections", "vias"}                 a pass has finished
    {"type": "done", "result": Result summary, "measured": Measured | None, "seconds": float}
    {"type": "error", "text": str}

``variant`` numbers the raced variants of a pass from 0 (the plain one); -1
marks the outcome of a pass. ``wires`` is a list of (layer index, net id,
points); it is rough (gate midpoints) until ``final`` is true.
"""
import multiprocessing
import os
import signal
import sys
import time
import traceback

from weaveengine.settings import Settings


def _work(dsn_path: str, settings: Settings, events, resume=None) -> None:
    """Body of the routing process. ``resume``: the ``kept`` of an earlier result to go on from."""
    try:
        if hasattr(os, "setsid"):
            os.setsid()  # own process group: cancelling takes the worker processes too
    except OSError:
        pass
    try:
        from weaveengine import accel
        from weaveengine.io.dsn import read_dsn
        from weaveengine.router import route_board
        from weaveengine.settings import cost_params
        from weaveengine.topo import planar_map

        start = time.time()
        status = accel.check()
        events.put({"type": "status", "compiled": status.compiled, "message": status.message, "warning": status.warning})
        design = read_dsn(dsn_path)
        want_clearance, want_edge, notes = settings.apply_rules(design, dsn_path)
        rules = design.board.rules
        notes.insert(0, f"rules: trace {rules.trace_width:g} mm, clearance {want_clearance:g} mm, "
                        f"via {rules.via_diameter:g}/{rules.via_drill:g} mm; {len(design.board.layers)} layers, "
                        f"{len(design.board.pads)} pads")
        for text in notes:
            events.put({"type": "note", "text": text})
        events.put({"type": "board", "design": design})
        params = cost_params(settings, planar_map.build(design.board))
        result = route_board(design.board, params, settings.options(), seed=settings.seed,
                             drc_rounds=settings.drc_rounds,
                             workers=settings.workers or None, events=events.put, resume=resume)
        summary = result.summary()
        events.put({"type": "done", "result": summary, "design": design, "seconds": time.time() - start,
                    "want": (want_clearance, want_edge)})
    except BaseException as error:  # report, never die silently
        if isinstance(error, KeyboardInterrupt):
            return
        events.put({"type": "error", "text": f"{type(error).__name__}: {error}", "trace": traceback.format_exc()})


class Job:
    """One routing run in a background process."""

    def __init__(self, dsn_path: str, settings: Settings | None = None, resume: dict | None = None):
        self.dsn_path = dsn_path
        self.settings = settings or Settings()
        self.resume = resume  # ``Result.kept`` of an earlier run of this board with these rules: go on from it
        # A fresh interpreter, not a fork: the front end may be a GUI with threads.
        # (Inside it, the router forks its own workers where the platform allows.)
        self._ctx = multiprocessing.get_context("spawn")
        # A SimpleQueue writes straight to its pipe. (A Queue hands writes to a
        # helper thread, and events from the router's forked workers get lost.)
        self._events = self._ctx.SimpleQueue()
        self._process = None
        self.finished = False

    def start(self) -> None:
        self._process = self._ctx.Process(target=_work, args=(self.dsn_path, self.settings, self._events, self.resume), daemon=False)
        self._process.start()

    def poll(self, limit: int = 200) -> list[dict]:
        """Events that have arrived since the last call (never blocks)."""
        out = []
        for _ in range(limit):
            if self._events.empty():
                break
            event = self._events.get()
            if event["type"] in ("done", "error"):
                self.finished = True
            out.append(event)
        if not out and not self.finished and self._process is not None and not self._process.is_alive():
            self.finished = True
            out.append({"type": "error", "text": f"the routing process stopped unexpectedly (exit code {self._process.exitcode})"})
        return out

    def running(self) -> bool:
        return self._process is not None and self._process.is_alive() and not self.finished

    def cancel(self) -> None:
        """Stops the run and its worker processes."""
        self.finished = True
        process = self._process
        if process is None or not process.is_alive():
            return
        try:
            if hasattr(os, "killpg"):
                os.killpg(process.pid, signal.SIGTERM)  # the routing process leads its own group
            else:
                process.terminate()
        except (OSError, ProcessLookupError):
            process.terminate()
        process.join(timeout=3)
        if process.is_alive():
            process.kill()

    def wait(self, timeout: float | None = None, on_event=None) -> dict | None:
        """Blocks until the run ends; returns the final ("done" or "error") event."""
        end = None if timeout is None else time.time() + timeout
        while True:
            for event in self.poll():
                if on_event:
                    on_event(event)
                if event["type"] in ("done", "error"):
                    return event
            if end is not None and time.time() > end:
                return None
            time.sleep(0.05)
