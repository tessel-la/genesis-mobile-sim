import pytest

from genesis_mobile_sim.timing import next_tick


def test_camera_overrun_is_recovered_instead_of_accumulating_time_drift():
    deadline = wall = 0.0
    for step in range(50):
        wall += 0.006 + (0.050 if step % 10 == 0 else 0)
        deadline, delay = next_tick(deadline, wall, 0.02)
        wall += delay
    assert wall == pytest.approx(1.0)


def test_long_stall_bounds_catch_up_without_negative_sleep():
    deadline, delay = next_tick(1.0, 30.0, 0.02)
    assert deadline == pytest.approx(29.8)
    assert delay == 0.0
