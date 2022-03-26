from dataclasses import dataclass
from typing import Dict, List

import numpy as np
import pytest

from lisdf.planner_output.command import (
    ActuateGripper,
    Command,
    GripperPosition,
    JointName,
    JointSpacePath,
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


@pytest.mark.parametrize(
    "waypoints, duration",
    [
        pytest.param([], 1.0, id="empty waypoints"),
        pytest.param({"joint_1": 0.0}, 1.0, id="waypoint is a float"),
        pytest.param(
            {"joint_1": [0.0]},
            2.5,
            id="only one waypoint",
        ),
        pytest.param(
            {"joint_1": [0.0, "lis is cool", 1.0]},
            3.0,
            id="waypoint is not a list of numbers",
        ),
        pytest.param(
            {"joint_1": [0.0, 1.0, 1.0], "joint_2": [0.5, 0.6]},
            42.0,
            id="waypoints are not the same length",
        ),
        pytest.param(
            {"joint_1": [0.0, 0.5, 1.0]},
            0.0,
            id="zero duration",
        ),
        pytest.param(
            {"joint_1": [0.0, 0.5, 1.0]},
            -999,
            id="negative duration",
        ),
    ],
)
def test_joint_space_path_raises_value_error(waypoints, duration):
    with pytest.raises(ValueError):
        JointSpacePath(waypoints, duration)


@pytest.mark.parametrize(
    "waypoints, duration, expected_dimensionality, expected_num_waypoints",
    [
        ({"joint_1": [0.0, 0.5, 1.0]}, 5.0, 1, 3),
        (
            {"joint_1": list(range(10)), "joint_2": list(reversed(range(10)))},
            3.0,
            2,
            10,
        ),
    ],
)
def test_joint_space_path(
    waypoints, duration, expected_dimensionality, expected_num_waypoints
):
    path = JointSpacePath(waypoints, duration, label="my_label")
    assert path.waypoints == waypoints
    assert path.duration == duration
    assert path.type == "JointSpacePath"
    assert path.label == "my_label"

    # Other properties
    assert path.dimensionality == expected_dimensionality
    assert path.num_waypoints == expected_num_waypoints


@pytest.fixture
def complex_path() -> JointSpacePath:
    return JointSpacePath(
        waypoints={
            "joint_1": [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0],
            "joint_2": [0.0, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4, 0.45, 0.5],
            "joint_3": [-0.2, -0.1, 0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8],
            "joint_4": [
                -1.0,
                -0.85,
                -0.7,
                -0.55,
                -0.4,
                -0.25,
                -0.1,
                0.05,
                0.2,
                0.35,
                0.5,
            ],
            "joint_5": [-0.5, -0.4, -0.3, -0.2, -0.1, 0.0, 0.1, 0.2, 0.3, 0.4, 0.5],
            "joint_6": [1.0, 0.95, 0.9, 0.85, 0.8, 0.75, 0.7, 0.65, 0.6, 0.55, 0.5],
            "joint_7": [1.0, 0.85, 0.7, 0.55, 0.4, 0.25, 0.1, -0.05, -0.2, -0.35, -0.5],
        },
        duration=42.0,
        label="complex_joint_space_path",
    )


def _expected_waypoint_at_idx(
    waypoints: Dict[JointName, List[float]], idx: int
) -> Dict[JointName, float]:
    return {joint_name: positions[idx] for joint_name, positions in waypoints.items()}


def test_joint_space_path_waypoints_derived_properties(complex_path):
    assert complex_path.dimensionality == 7
    assert complex_path.num_waypoints == 11


def test_joint_space_path_waypoints_for_joint(complex_path):
    """Getting the waypoints for a given joint"""
    assert (
        complex_path.waypoints_for_joint("joint_5") == complex_path.waypoints["joint_5"]
    )
    with pytest.raises(ValueError):
        complex_path.waypoints_for_joint("joint_999")


def test_joint_space_path_waypoint(complex_path):
    """Test getting the waypoints at a given index"""
    assert complex_path.waypoint(0) == _expected_waypoint_at_idx(
        complex_path.waypoints, 0
    )
    assert complex_path.waypoint(10) == _expected_waypoint_at_idx(
        complex_path.waypoints, 10
    )
    with pytest.raises(ValueError):
        complex_path.waypoint(999)


def test_joint_space_path_waypoint_as_np_array(complex_path):
    """Testing getting the waypoints at a given index as a np.array"""
    # joint_1, joint_2, ..., joint_7
    joint_name_ordering = [f"joint_{num}" for num in range(1, 8)]
    expected_waypoint_at_idx_0 = list(
        _expected_waypoint_at_idx(complex_path.waypoints, 0).values()
    )
    expected_waypoint_at_idx_10 = list(
        _expected_waypoint_at_idx(complex_path.waypoints, 10).values()
    )

    assert np.array_equal(
        complex_path.waypoint_as_np_array(0, joint_name_ordering),
        np.array(expected_waypoint_at_idx_0),
    )
    assert np.array_equal(
        complex_path.waypoint_as_np_array(10, joint_name_ordering),
        np.array(expected_waypoint_at_idx_10),
    )

    # Reverse ordering: joint_7, joint_6, ..., joint_1
    reversed_joint_name_ordering = list(reversed(joint_name_ordering))
    assert np.array_equal(
        complex_path.waypoint_as_np_array(0, reversed_joint_name_ordering),
        np.array(list(reversed(expected_waypoint_at_idx_0))),
    )
    assert np.array_equal(
        complex_path.waypoint_as_np_array(10, reversed_joint_name_ordering),
        np.array(list(reversed(expected_waypoint_at_idx_10))),
    )

    # Invalid joint name ordering 1_joint, 2_joint, etc.
    with pytest.raises(ValueError):
        complex_path.waypoint_as_np_array(
            0, [f"{idx + 1}_joint" for idx in range(1, 8)]
        )


def test_waypoints_as_np_array(complex_path):
    """Test getting all the waypoints as a np.array"""
    # joint_1, joint_2, ..., joint_7
    joint_name_ordering = [f"joint_{num}" for num in range(1, 8)]
    expected_np_array = np.array(
        [complex_path.waypoints[joint_name] for joint_name in joint_name_ordering]
    )
    assert expected_np_array.shape == (7, 11)

    assert np.array_equal(
        complex_path.waypoints_as_np_array(joint_name_ordering), expected_np_array
    )

    # Reverse ordering: joint_7, joint_6, ..., joint_1
    # Check matrix is flipped as well
    reversed_joint_name_ordering = list(reversed(joint_name_ordering))
    assert np.array_equal(
        complex_path.waypoints_as_np_array(reversed_joint_name_ordering),
        np.flip(expected_np_array, axis=0),
    )

    # Invalid joint name ordering 1_joint, 2_joint, etc.
    with pytest.raises(ValueError):
        complex_path.waypoint_as_np_array(
            0, [f"{idx + 1}_joint" for idx in range(1, 8)]
        )


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
