import numpy as np
import pytest

from lisdf.plan_executor.gripper_executor import ActuateGripperExecutor
from lisdf.planner_output.command import ActuateGripper, GripperPosition


@pytest.mark.parametrize(
    "command, expected_configuration",
    [
        (
            ActuateGripper(configurations={"gripper_1": GripperPosition.close}),
            np.zeros(9),
        ),
        (
            ActuateGripper(configurations={"gripper_1": GripperPosition.open}),
            np.array([0, 0, 0, 0, 0, 0, 0, 0.04, 0.04]),
        ),
    ],
)
def test_actuate_gripper_executor(command, expected_configuration, panda):
    executor = ActuateGripperExecutor(panda, command, 2.0)
    executor.execute(2.5)
    assert np.allclose(panda.configuration, expected_configuration)


def test_actuate_gripper_executor_raises_error(panda):
    # Two gripper actions which we don't support
    command = ActuateGripper(
        configurations={
            "gripper_1": GripperPosition.open,
            "gripper_2": GripperPosition.close,
        }
    )
    executor = ActuateGripperExecutor(panda, command, 1.0)
    with pytest.raises(ValueError):
        executor.execute(1.1)
