import json
import os
from typing import Dict

import pytest
import yaml

from lisdf.planner_output.command import (
    ActuateGripper,
    Command,
    GripperPosition,
    JointSpacePath,
)
from lisdf.planner_output.config import CURRENT_LISDF_PLAN_VERSION
from lisdf.planner_output.plan import LISDFPlan

_CURRENT_DIR = os.path.dirname(__file__)
_VALID_VERSION = CURRENT_LISDF_PLAN_VERSION

_VALID_JOINT_SPACE_PATH = JointSpacePath(
    waypoints={"joint_1": [0.0, 1.0]}, duration=1.0
)
_VALID_COMMANDS = [_VALID_JOINT_SPACE_PATH]


class _UnvalidatedCommand(Command, type="unvalidated_command_for_tests"):
    @classmethod
    def _from_json_dict(cls, json_dict: Dict) -> "Command":
        raise NotImplementedError

    def validate(self):
        pass


@pytest.mark.parametrize(
    "lisdf_problem, version, commands",
    [
        pytest.param(
            "lisdf-non-existent-path-i-hope",
            _VALID_VERSION,
            _VALID_COMMANDS,
            id="lisdf_problem does not exist",
            # We're not checking paths at the moment, remove this mark once we do
            marks=pytest.mark.xfail,
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
        pytest.param(_CURRENT_DIR, _VALID_VERSION, [], id="empty commands"),
        pytest.param(
            _CURRENT_DIR,
            _VALID_VERSION,
            [
                _VALID_JOINT_SPACE_PATH,
                "abc",
                "its",
                "easy",
            ],
            id="invalid commands",
        ),
        pytest.param(
            _CURRENT_DIR,
            _VALID_VERSION,
            [
                JointSpacePath({"joint_1": [0.0, 1.0], "joint_2": [0.0, 2.0]}),
                JointSpacePath(
                    {
                        "joint_1": [0.0, 1.0],
                        "joint_2": [0.0, 2.0],
                        "joint_3": [0.2, 0.3],
                    }
                ),
            ],
            id="joint space paths different joint dims",
        ),
        pytest.param(
            _CURRENT_DIR,
            _VALID_VERSION,
            [
                JointSpacePath({"joint_1": [0.0, 1.0], "joint_2": [0.0, 2.0]}),
                JointSpacePath(
                    {
                        "joint_1": [1.0, 0.25],
                        "joint_2": [
                            1.99,
                            1.0,
                        ],  # first element should be 2.0 to match previous position
                    }
                ),
            ],
            id="joint space paths different inconsistent joint states",
        ),
        pytest.param(
            _CURRENT_DIR,
            _VALID_VERSION,
            [
                ActuateGripper({"gripper_1": GripperPosition.open}),
                ActuateGripper(
                    {
                        "gripper_1": GripperPosition.close,
                        "gripper_2": GripperPosition.open,
                    }
                ),
            ],
            id="actuate gripper commands different joint dims",
        ),
        pytest.param(
            _CURRENT_DIR,
            _VALID_VERSION,
            [
                ActuateGripper({"gripper_1": GripperPosition.open}),
                _UnvalidatedCommand(),
            ],
            id="unvalidated command in plan",
        ),
    ],
)
def test_lisdf_plan_raises_value_error(lisdf_problem, version, commands):
    with pytest.raises(ValueError):
        LISDFPlan(lisdf_problem=lisdf_problem, commands=commands, version=version)


@pytest.mark.parametrize(
    "commands",
    [
        _VALID_COMMANDS,
        [
            JointSpacePath({"joint_1": [0.0, 1.0], "joint_2": [0.0, 2.0]}),
            ActuateGripper({"gripper_1": GripperPosition.open}),
            JointSpacePath(
                {
                    "joint_1": [1.0, 0.25],
                    "joint_2": [
                        2.0,
                        1.0,
                    ],
                }
            ),
        ],
        [
            ActuateGripper(
                {
                    "gripper_1": GripperPosition.open,
                    "gripper_2": GripperPosition.open,
                }
            ),
            ActuateGripper(
                {
                    "gripper_1": GripperPosition.close,
                    "gripper_2": GripperPosition.close,
                }
            ),
        ],
    ],
)
def test_lisdf_plan(commands):
    lisdf_plan = LISDFPlan(
        lisdf_problem=_CURRENT_DIR, commands=commands, version=_VALID_VERSION
    )
    assert lisdf_plan.lisdf_problem == _CURRENT_DIR
    assert lisdf_plan.version == _VALID_VERSION
    assert lisdf_plan.commands == commands


def test_lisdf_plan_with_default_version():
    lisdf_plan = LISDFPlan(lisdf_problem=_CURRENT_DIR, commands=_VALID_COMMANDS)
    assert lisdf_plan.lisdf_problem == _CURRENT_DIR
    assert lisdf_plan.version == CURRENT_LISDF_PLAN_VERSION
    assert lisdf_plan.commands == _VALID_COMMANDS


@pytest.mark.parametrize("lisdf_problem, version", [(_CURRENT_DIR, _VALID_VERSION)])
def test_lisdf_plan_with_complex_commands(
    lisdf_problem, version, complex_commands, expected_complex_lisdf_plan_dict
):
    """Complex test case where we check entire functionality of LISDFPlan"""
    lisdf_plan = LISDFPlan(
        lisdf_problem=lisdf_problem, version=version, commands=complex_commands
    )

    # Check to_json() and from_json() works as expected
    json_str = lisdf_plan.to_json()
    assert json.loads(json_str) == expected_complex_lisdf_plan_dict
    assert LISDFPlan.from_json(json_str) == lisdf_plan

    # Check to_yaml() and from_json() works as expected
    yaml_str = lisdf_plan.to_yaml()
    assert yaml.safe_load(yaml_str) == expected_complex_lisdf_plan_dict
    assert LISDFPlan.from_yaml(yaml_str) == lisdf_plan

    # Check from_json_dict() works as expected
    assert LISDFPlan.from_json_dict(expected_complex_lisdf_plan_dict) == lisdf_plan
