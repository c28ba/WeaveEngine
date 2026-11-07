import multiprocessing
import os
import signal
import time

import pytest

from weaveengine import parallel


def _dies_in_a_worker(shared, item):
    """Doubles ``item``; a worker given 3 ends at once, as one killed would."""
    if parallel._inside and item == 3:
        os._exit(1)
    return item * 2


@pytest.mark.skipif(not parallel.can_fork(), reason="workers are forked")
def test_a_worker_that_dies_does_not_leave_the_batch_waiting():
    assert parallel.run(_dies_in_a_worker, None, range(8), workers=2) == [i * 2 for i in range(8)]


@pytest.mark.skipif(not parallel.can_fork(), reason="workers are forked")
def test_a_raced_worker_that_dies_does_not_leave_the_race_waiting():
    out = parallel.first_accepted(_dies_in_a_worker, None, range(8), lambda r: r == 10, workers=2)
    assert out == [0, 2, 4, 6, 8, 10]


def _hangs(shared, item):
    """Tells which worker took ``item``, then waits for ever."""
    shared.put(os.getpid())
    time.sleep(600)


def _runs_a_batch_that_hangs(pids):
    parallel.run(_hangs, pids, range(2), workers=2)


@pytest.mark.skipif(not parallel.can_fork(), reason="workers are forked")
def test_a_worker_whose_parent_is_gone_ends():
    """A routing process killed (cancelled) while its workers wait: they do not
    wait for ever, holding pipes that keep the program that started it from exiting."""
    ctx = multiprocessing.get_context("fork")
    pids = ctx.SimpleQueue()
    middle = ctx.Process(target=_runs_a_batch_that_hangs, args=(pids,))
    middle.start()
    workers = [pids.get(), pids.get()]
    os.kill(middle.pid, signal.SIGKILL)
    middle.join()
    deadline = time.time() + 10
    alive = workers
    while alive and time.time() < deadline:
        time.sleep(0.1)
        alive = [pid for pid in alive if _exists(pid)]
    for pid in alive:
        os.kill(pid, signal.SIGKILL)
    assert alive == []


def _exists(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    return True
