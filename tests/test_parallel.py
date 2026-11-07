import os

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
