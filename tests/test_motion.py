import pytest

from genesis_mobile_sim.motion import Motion


def test_velocity_drives_only_an_upright_robot_and_expires_to_stand():
    motion = Motion()
    motion.command(0.2, 0, 0, 10.0, 100, True)
    assert motion.mode == "walk"
    motion.update(101, 10.49, 1.0, 0.3)
    assert motion.mode == "walk"
    motion.update(102, 10.5, 1.0, 0.3)
    assert motion.mode == "stand"


def test_fall_damps_without_resetting_position_and_recovery_needs_a_stable_stance():
    motion = Motion()
    motion.command(0.2, 0, 0, 10, 60, True)
    motion.update(61, 10, -0.9, 0.12)
    assert motion.mode == "fallen"
    assert motion.commands.current(10) == (0, 0, 0)
    with pytest.raises(ValueError):
        motion.command(0.2, 0, 0, 10.1, 62, False)
    with pytest.raises(ValueError):
        motion.request("stand", 62, False)
    motion.request("recover", 62, False)
    for step in range(63, 88):
        motion.update(step, 10.2, 1, 0.3)
    assert motion.mode == "recover"  # min 1 s, even if upright early
    motion.update(88, 10.2, 0.7, 0.3)  # stability count restarts after a wobble
    for step in range(89, 114):
        motion.update(step, 10.3, 1, 0.3)
    assert motion.mode == "stand"
    assert motion.reason == "Recovery complete"


def test_recovery_times_out_without_replaying_old_velocity():
    motion = Motion()
    motion.command(0.2, 0, 0, 10, 60, True)
    motion.request("recover", 61, False)
    assert motion.commands.current(10.1) == (0, 0, 0)
    with pytest.raises(ValueError):
        motion.command(0.2, 0, 0, 10.2, 62, True)
    motion.update(661, 20, 0, 0.15)
    assert motion.mode == "fallen"
    motion.request("recover", 662, False)
    assert motion.mode == "recover"


def test_unknown_motion_does_not_change_current_mode():
    motion = Motion()
    with pytest.raises(ValueError):
        motion.request("jump", 100, True)
    assert motion.mode == "stand"
