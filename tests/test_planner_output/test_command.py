from dataclasses import dataclass

import numpy as np
import pytest

from lisdf.planner_output.command import (
    ActuateGripper,
    Command,
    GripperPosition,
    JointSpacePath,
    Waypoint,
)


@dataclass(frozen=True)
class _ConcreteCommand(Command, type="_ConcreteCommand"):
    other_attr: str = "willshen"

    def validate(self):
        pass


def test_command_to_dict():
    assert _ConcreteCommand().to_dict() == {
        "type": "_ConcreteCommand",
        "other_attr": "willshen",
    }


def test_waypoint_raises_value_error():
    with pytest.raises(ValueError):
        # Empty waypoint
        Waypoint({})


@pytest.mark.parametrize(
    "joint_confs, expected_joint_values",
    [
        ({"joint_1": 0.0, "joint_2": 0.5, "joint_3": -0.2}, [0.0, 0.5, -0.2]),
        (
            {f"joint_{num}": 0.1 * num for num in range(10)},
            [0.1 * num for num in range(10)],
        ),
    ],
)
def test_waypoint(joint_confs, expected_joint_values):
    waypoint = Waypoint(joint_confs)
    assert waypoint.configurations == joint_confs
    assert waypoint.dimensionality == len(expected_joint_values)
    assert waypoint.values_as_list() == expected_joint_values
    assert (waypoint.values_as_np_array() == np.array(expected_joint_values)).all()


@pytest.mark.parametrize(
    "waypoints, duration",
    [
        pytest.param([], 1.0, id="empty waypoints"),
        pytest.param(
            [Waypoint({"joint_1": 0.0})],
            2.5,
            id="only one waypoint",
        ),
        pytest.param(
            [Waypoint({"joint_1": 0.0}), Waypoint({"joint_1": 0.2})],
            0.0,
            id="zero duration",
        ),
        pytest.param(
            [Waypoint({"joint_1": 0.0}), Waypoint({"joint_1": 0.2})],
            -999,
            id="negative duration",
        ),
        pytest.param(
            [Waypoint({"joint_1": 0.0}), Waypoint({"joint_2": 1.0})],
            5.0,
            id="waypoints with different joint names",
        ),
    ],
)
def test_joint_space_path_raises_value_error(waypoints, duration):
    with pytest.raises(ValueError):
        JointSpacePath(waypoints, duration)


@pytest.mark.parametrize(
    "waypoints, duration",
    [([Waypoint({"joint_1": 0.0}), Waypoint({"joint_1": 1.0})], 5.0)],
)
def test_joint_space_path(waypoints, duration):
    path = JointSpacePath(waypoints, duration)
    assert path.waypoints == waypoints
    assert path.duration == duration
    assert path.type == "JointSpacePath"


@pytest.mark.parametrize(
    "configurations",
    [{}, {"gripper_1": "lis is cool"}, {"gripper_n": "my gripper might be open?"}],
)
def test_actuate_gripper_raises_value_error(configurations):
    with pytest.raises(ValueError):
        ActuateGripper(configurations)


@pytest.mark.parametrize(
    "configurations",
    [
        {"gripper_1": GripperPosition.OPEN},
        {"gripper_left": GripperPosition.CLOSE, "gripper_right": GripperPosition.OPEN},
    ],
)
def test_actuate_gripper(configurations):
    actuate_gripper = ActuateGripper(configurations)
    assert actuate_gripper.configurations == configurations
    assert actuate_gripper.type == "ActuateGripper"

    # Check we can get position via gripper joint name
    test_gripper_joint = next(iter(configurations.keys()))
    expected_gripper_position = configurations[test_gripper_joint]
    assert (
        actuate_gripper.position_for_gripper_joint(test_gripper_joint)
        == expected_gripper_position
    )

    # Non-existent joint raises error
    with pytest.raises(ValueError):
        actuate_gripper.position_for_gripper_joint("non-existent-gripper-joint")
