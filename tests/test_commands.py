import math

import pytest

from genesis_mobile_sim.policy import (
    COMMAND_TIMEOUT,
    MAX_ANGULAR,
    MAX_LATERAL,
    MAX_LINEAR,
    VelocityCommand,
)


def test_command_expires_even_if_simulation_stalls():
    command = VelocityCommand()
    command.request(0.2, -0.1, 0.3, 5.0)
    assert command.current(5.1) == (0.2, -0.1, 0.3)
    assert command.current(5.0 + COMMAND_TIMEOUT) == (0.0, 0.0, 0.0)


@pytest.mark.parametrize("invalid", [math.inf, -math.inf, math.nan])
@pytest.mark.parametrize("index", range(4))
def test_invalid_commands_stop_existing_motion(invalid, index):
    command = VelocityCommand()
    command.request(0.2, 0, 0, 0)
    values = [0.2, 0.1, 0.3, 0.1]
    values[index] = invalid
    with pytest.raises(ValueError):
        command.request(*values)
    assert command.current(0.1) == (0.0, 0.0, 0.0)


def test_command_limits_preserve_direction():
    command = VelocityCommand()
    command.request(10, -10, 10, 0)
    assert command.current(0) == (MAX_LINEAR, -MAX_LATERAL, MAX_ANGULAR)
