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
import threading
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


def _call_numbered(numbered):
    """``_call`` for an item given with its place in the batch, which comes back with the result."""
    n, item = numbered
    return n, _call(item)


def _orphaned() -> None:
    """In a worker: ends it once the process that started it has gone. A worker
    can outlive it (the routing process cancelled while a sibling, killed, held
    the lock of the pool's queue) and would then wait on that lock for ever,
    holding pipes that keep its grandparent from exiting."""
    parent = multiprocessing.parent_process()
    if parent is None:
        return
    ppid = os.getppid()

    def watch():
        while True:
            parent.join(1.0)
            if not parent.is_alive() or os.getppid() != ppid:
                os._exit(1)

    threading.Thread(target=watch, daemon=True).start()


def _mark_worker():
    global _inside
    _inside = True
    _orphaned()


def _given(task):
    """Starts a worker that was not forked: it is given the task, as a copy."""
    global _task, _inside
    _task, _inside = task, True
    _orphaned()


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
            return _watch(pool, pool.map_async(_call, items, chunksize=chunk).get)
    except WorkerDied:  # the same work in this process: the same results, if slower
        return [fn(shared, item) for item in items]
    finally:
        _task = None


class WorkerDied(Exception):
    """A worker process ended while it held work (killed, out of memory, crashed)."""


POLL = 0.5  # s between looks at the workers while waiting for results


def _watch(pool, wait, seen: list | None = None):
    """``wait(timeout)`` until it returns, raising ``WorkerDied`` if one of the
    pool's workers has ended in the meantime. A pool replaces a worker that
    dies, but the work that worker held is lost and its result never comes:
    without this, the wait would never end. ``seen``: the workers watched by
    earlier waits on the same batch, to which this one adds its own; a worker
    that died while other results were still coming in has been replaced by
    the time the wait for its own result begins."""
    workers = list(pool._pool)
    if seen is not None:
        seen.extend(p for p in workers if p not in seen)
        workers = seen
    while True:
        try:
            return wait(POLL)
        except multiprocessing.TimeoutError:
            if any(p.exitcode not in (None, 0) for p in workers):
                raise WorkerDied from None


class Stopped(Exception):
    """Raised inside a worker whose result is no longer wanted."""


def first_accepted(fn, shared, items, accept, workers: int | None = None, stop=None, arrived=None) -> list:
    """Like ``run``, but stops at the first item, in order, whose result
    ``accept`` approves, and returns the results up to and including it (all of
    them if none is accepted). Later items are abandoned. Because results are
    taken in item order, what is returned does not depend on which worker
    happens to finish first.

    ``arrived``, if given, is called in this process with each result as soon
    as it comes, in the order they finish (not in item order): to save the best
    so far as it goes. It may see a result of an item after the accepted one.

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
        return _in_order(fn, shared, items, accept, arrived)
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
        out, waiting, seen = [], {}, []
        with pool:
            # Taken as they finish, kept in order: the decision is made in item order.
            results = pool.imap_unordered(_call_numbered, list(enumerate(items)))
            for _ in items:
                n, result = _watch(pool, results.next, seen)
                if arrived is not None:
                    arrived(result)
                waiting[n] = result
                while len(out) in waiting:
                    out.append(waiting.pop(len(out)))
                    if accept(out[-1]):
                        break
                if out and accept(out[-1]):
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
    except WorkerDied:  # in this process, in order, as without workers
        return _in_order(fn, shared, items, accept, arrived)
    finally:
        _task = None


def _in_order(fn, shared, items, accept, arrived) -> list:
    """``first_accepted`` in this process: one item after another."""
    out = []
    for item in items:
        out.append(fn(shared, item))
        if arrived is not None:
            arrived(out[-1])
        if accept(out[-1]):
            break
    return out


def stop_flag():
    """An event the workers of ``first_accepted`` share with their parent."""
    return multiprocessing.get_context("fork" if can_fork() else "spawn").Event()
