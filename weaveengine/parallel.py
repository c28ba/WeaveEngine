"""Process-level parallelism (design section 22).

The hot loops hold the GIL (and the compiled kernels are single-threaded per
call), so work is spread over *processes*. Workers are forked: they inherit the
maps and the current topological state copy-on-write, so only small task
arguments and results are pickled. A fork is a snapshot, so every batch gets
fresh workers.

Where fork is not available (Windows) a batch runs in this process: same
results, one core. Only the raced variants of a pass (``first_accepted``) are
spread over processes there, started afresh, each given a copy of what it
needs once: a variant is a whole pass, so the copy costs little beside it.
(WEAVEENGINE_START=spawn does the same where fork exists, to try it.)
"""
import multiprocessing
import os
import sys
import time

before_fork: list = []  # callables run before workers are forked (to stop helper threads: fork and threads do not mix)
_task = None     # (function, shared) of the batch being run; read by forked workers
_inside = False  # set in workers: no nested pools


def cpu_count() -> int:
    """Worker count: WEAVEENGINE_WORKERS if set, else the machine's cores (at most 12)."""
    env = os.environ.get("WEAVEENGINE_WORKERS")
    if env:
        return max(1, int(env))
    return max(1, min(12, os.cpu_count() or 1))


def can_fork() -> bool:
    return (sys.platform != "win32" and "fork" in multiprocessing.get_all_start_methods()
            and os.environ.get("WEAVEENGINE_START") != "spawn")


def travels(fn) -> bool:
    """Whether the callable ``fn`` still works in a worker that was given a
    copy of it: the ``put`` of a multiprocessing queue (it writes to a pipe the
    worker is handed as it starts). Anything else may not survive the copy."""
    import multiprocessing.queues
    return isinstance(getattr(fn, "__self__", None), (multiprocessing.queues.SimpleQueue, multiprocessing.queues.Queue))


def _call(item):
    fn, shared = _task
    return fn(shared, item)


def _mark_worker():
    global _inside
    _inside = True


def _given(task):
    """Starts a worker that was not forked: it is given the task, as a copy."""
    global _task, _inside
    _task, _inside = task, True


def run(fn, shared, items, workers: int | None = None, min_items: int = 2) -> list:
    """[fn(shared, item) for item in items], spread over forked workers.

    ``fn`` must be a module-level function. ``shared`` is whatever it needs
    (inherited, not pickled); items and results are pickled. Order is kept, and
    the result does not depend on the number of workers.
    """
    global _task
    items = list(items)
    workers = cpu_count() if workers is None else workers
    workers = min(workers, len(items))
    if workers <= 1 or len(items) < min_items or _inside or not can_fork():
        return [fn(shared, item) for item in items]
    for halt in before_fork:
        halt()
    _task = (fn, shared)
    try:
        ctx = multiprocessing.get_context("fork")
        chunk = max(1, len(items) // (workers * 4))
        with ctx.Pool(workers, initializer=_mark_worker) as pool:
            return pool.map(_call, items, chunksize=chunk)
    finally:
        _task = None


class Stopped(Exception):
    """Raised inside a worker whose result is no longer wanted."""


def first_accepted(fn, shared, items, accept, workers: int | None = None, stop=None) -> list:
    """Like ``run``, but stops at the first item, in order, whose result
    ``accept`` approves, and returns the results up to and including it (all of
    them if none is accepted). Later items are abandoned. Because results are
    taken in item order, what is returned does not depend on which worker
    happens to finish first.

    ``stop``, if given, is an event the abandoned workers can see (create it
    with ``stop_flag()`` and put it in ``shared``): it is set once a result is
    accepted, and they are given a few seconds to notice and return on their
    own before being terminated. A worker killed in the middle of writing to a
    shared pipe would leave it locked, so this matters when workers send events.
    """
    global _task
    items = list(items)
    workers = min(cpu_count() if workers is None else workers, len(items))
    if workers <= 1 or _inside:
        out = []
        for item in items:
            out.append(fn(shared, item))
            if accept(out[-1]):
                break
        return out
    fork = can_fork()
    if fork:
        for halt in before_fork:
            halt()
        _task = (fn, shared)
    try:
        if fork:
            ctx = multiprocessing.get_context("fork")
            pool = ctx.Pool(workers, initializer=_mark_worker)
        else:
            pool = multiprocessing.get_context("spawn").Pool(workers, initializer=_given, initargs=((fn, shared),))
        out = []
        with pool:
            for result in pool.imap(_call, items):
                out.append(result)
                if accept(result):
                    if stop is not None:
                        stop.set()
                        pool.close()
                        deadline = time.time() + 6.0
                        while time.time() < deadline and any(p.is_alive() and p.exitcode is None for p in pool._pool) \
                                and pool._cache:
                            time.sleep(0.02)
                    pool.terminate()
                    break
        return out
    finally:
        _task = None


def stop_flag():
    """An event the workers of ``first_accepted`` share with their parent."""
    return multiprocessing.get_context("fork" if can_fork() else "spawn").Event()
