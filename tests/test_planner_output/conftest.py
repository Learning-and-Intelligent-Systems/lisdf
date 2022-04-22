from typing import Dict, List

import pytest

from lisdf.planner_output.command import (
    ActuateGripper,
    Command,
    GripperPosition,
    JointSpacePath,
)


def generate_complex_commands() -> List[Command]:
    return [
        JointSpacePath(
            waypoints={
                "joint_1": [0.0, 0.25, 0.5],
                "joint_2": [0.0, 0.1, 0.2],
                "joint_3": [0.0, 0.15, 0.3],
            },
            duration=5.0,
            label="move_to_pick",
        ),
        ActuateGripper(
            configurations={"gripper_1": GripperPosition.close}, label="pick"
        ),
        JointSpacePath(
            waypoints={
                "joint_1": [0.5, 0.2],
                "joint_2": [0.2, 0.6],
                "joint_3": [0.3, 0.15],
            },
            duration=3.0,
            label="move_to_place",
        ),
        ActuateGripper(
            configurations={"gripper_1": GripperPosition.open}, label="place"
        ),
        JointSpacePath(
            waypoints={
                "joint_1": [0.2, 0.0],
                "joint_2": [0.6, 0.0],
                "joint_3": [0.15, 0.0],
            },
            duration=2.5,
            label="go_to_zero",
        ),
    ]


@pytest.fixture
def complex_commands() -> List[Command]:
    return generate_complex_commands()


@pytest.fixture
def expected_complex_lisdf_plan_dict(lisdf_problem, version) -> Dict:
    """Expected plain python dict representation of a complex LISDF plan."""
    return {
        "commands": [
            {
                "duration": 5.0,
                "label": "move_to_pick",
                "type": "JointSpacePath",
                "waypoints": {
                    "joint_1": [0.0, 0.25, 0.5],
                    "joint_2": [0.0, 0.1, 0.2],
                    "joint_3": [0.0, 0.15, 0.3],
                },
            },
            {
                "configurations": {"gripper_1": "close"},
                "label": "pick",
                "type": "ActuateGripper",
            },
            {
                "duration": 3.0,
                "label": "move_to_place",
                "type": "JointSpacePath",
                "waypoints": {
                    "joint_1": [0.5, 0.2],
                    "joint_2": [0.2, 0.6],
                    "joint_3": [0.3, 0.15],
                },
            },
            {
                "configurations": {"gripper_1": "open"},
                "label": "place",
                "type": "ActuateGripper",
            },
            {
                "duration": 2.5,
                "label": "go_to_zero",
                "type": "JointSpacePath",
                "waypoints": {
                    "joint_1": [0.2, 0.0],
                    "joint_2": [0.6, 0.0],
                    "joint_3": [0.15, 0.0],
                },
            },
        ],
        "lisdf_problem": lisdf_problem,
        "version": version,
    }
