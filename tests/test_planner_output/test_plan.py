import json
import os
from typing import Dict, List

import pytest

from lisdf.planner_output.command import (
    ActuateGripper,
    Command,
    GripperPosition,
    JointSpacePath,
    Waypoint,
)
from lisdf.planner_output.plan import LISDFPlan

_CURRENT_DIR = os.path.dirname(__file__)

_VALID_JOINT_SPACE_PATH = JointSpacePath(
    waypoints=[Waypoint({"joint_1": 0.0}), Waypoint({"joint_1": 1.0})], duration=1.0
)
_VALID_COMMANDS = [_VALID_JOINT_SPACE_PATH]


@pytest.mark.parametrize(
    "lisdf_path, version, commands",
    [
        pytest.param(
            "lisdf-non-existent-path-i-hope",
            "0.0",
            _VALID_COMMANDS,
            id="lisdf_path does not exist",
        ),
        pytest.param(
            _CURRENT_DIR,
            "1.z",
            _VALID_COMMANDS,
            id="invalid version number",
        ),
        pytest.param(
            _CURRENT_DIR,
            "99.99",
            _VALID_COMMANDS,
            id="unsupported version",
        ),
        pytest.param(
            _CURRENT_DIR,
            "0.5",
            ["abc", "its", "easy", "as", "123", _VALID_JOINT_SPACE_PATH],
            id="invalid commands",
        ),
    ],
)
def test_lisdf_plan_raises_value_error(lisdf_path, version, commands):
    with pytest.raises(ValueError):
        LISDFPlan(lisdf_path, version, commands)


@pytest.mark.parametrize(
    "lisdf_path, version, commands", [(_CURRENT_DIR, "0.1", _VALID_COMMANDS)]
)
def test_lisdf_plan(lisdf_path, version, commands):
    lisdf_plan = LISDFPlan(lisdf_path, version, commands)
    assert lisdf_plan.lisdf_path == lisdf_path
    assert lisdf_plan.version == version
    assert lisdf_plan.commands == commands


@pytest.fixture
def complex_commands() -> List[Command]:
    return [
        JointSpacePath(
            [
                Waypoint({"joint_1": 0.0, "joint_2": 0.0, "joint_3": 0.0}),
                Waypoint({"joint_1": 0.25, "joint_2": 0.1, "joint_3": 0.15}),
                Waypoint({"joint_1": 0.5, "joint_2": 0.2, "joint_3": 0.3}),
            ],
            duration=5.0,
            label="move_to_pick",
        ),
        ActuateGripper(
            configurations={"gripper_1": GripperPosition.CLOSE}, label="pick"
        ),
        JointSpacePath(
            [
                Waypoint({"joint_1": 0.5, "joint_2": 0.2, "joint_3": 0.3}),
                Waypoint({"joint_1": 0.2, "joint_2": 0.6, "joint_3": 0.15}),
            ],
            duration=3.0,
            label="move_to_place",
        ),
        ActuateGripper(
            configurations={"gripper_1": GripperPosition.OPEN}, label="place"
        ),
        JointSpacePath(
            [
                Waypoint({"joint_1": 0.2, "joint_2": 0.6, "joint_3": 0.15}),
                Waypoint({"joint_1": 0.0, "joint_2": 0.0, "joint_3": 0.0}),
            ],
            duration=2.5,
            label="go_to_zero",
        ),
    ]


@pytest.fixture
def expected_complex_lisdf_plan_dict() -> Dict:
    """Expected plain python dict representation of a complex LISDF plan."""
    return {
        "commands": [
            {
                "duration": 5.0,
                "label": "move_to_pick",
                "type": "JointSpacePath",
                "waypoints": [
                    {
                        "configurations": {
                            "joint_1": 0.0,
                            "joint_2": 0.0,
                            "joint_3": 0.0,
                        }
                    },
                    {
                        "configurations": {
                            "joint_1": 0.25,
                            "joint_2": 0.1,
                            "joint_3": 0.15,
                        }
                    },
                    {
                        "configurations": {
                            "joint_1": 0.5,
                            "joint_2": 0.2,
                            "joint_3": 0.3,
                        }
                    },
                ],
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
                "waypoints": [
                    {
                        "configurations": {
                            "joint_1": 0.5,
                            "joint_2": 0.2,
                            "joint_3": 0.3,
                        }
                    },
                    {
                        "configurations": {
                            "joint_1": 0.2,
                            "joint_2": 0.6,
                            "joint_3": 0.15,
                        }
                    },
                ],
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
                "waypoints": [
                    {
                        "configurations": {
                            "joint_1": 0.2,
                            "joint_2": 0.6,
                            "joint_3": 0.15,
                        }
                    },
                    {
                        "configurations": {
                            "joint_1": 0.0,
                            "joint_2": 0.0,
                            "joint_3": 0.0,
                        }
                    },
                ],
            },
        ],
        "lisdf_path": _CURRENT_DIR,
        "version": "0.1",
    }


def test_complex_lisdf_plan(complex_commands, expected_complex_lisdf_plan_dict):
    """Complex test case where we check entire functionality of LISDFPlan"""
    lisdf_plan = LISDFPlan(
        lisdf_path=_CURRENT_DIR, version="0.1", commands=complex_commands
    )

    json_as_dict = json.loads(lisdf_plan.to_json())
    assert json_as_dict == expected_complex_lisdf_plan_dict
