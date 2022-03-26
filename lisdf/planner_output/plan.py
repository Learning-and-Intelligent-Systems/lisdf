import os
from dataclasses import dataclass
from typing import Dict, List

from lisdf.planner_output.command import ActuateGripper, Command, JointSpacePath
from lisdf.planner_output.common import OutputElement
from lisdf.planner_output.config import (
    ENFORCE_JOINT_DIMENSIONALITIES,
    SUPPORTED_COMMAND_TYPES,
    SUPPORTED_PLANNER_OUTPUT_VERSIONS,
)


@dataclass(frozen=True)
class LISDFPlan(OutputElement):
    """
    Overarching dataclass for a LISDF plan. Check
    `tests/test_planner_output/test_plan.py` for examples on usage.

    We automatically validate the parameters passed to __init__ for
    a LISDFPlan and all the elements within it. This is to ensure they
    are not malformed and match our specification.

    You can also see `scripts/planner_output_demo.py` for a demo of how
    to use the entire model structure.
    """

    # Path of the LISDF folder where the world and model files are located
    lisdf_path: str

    # Version of the LISDF plan output specification
    version: str

    # List of Commands that need to be executed by the simulator
    #   We run multiple validation checks on these commands:
    #     1. All JointSpacePath commands have the same dimensionality
    #     2. All ActuateGripper commands have the same dimensionality
    #     3. The last waypoint of a JointSpacePath command is the same as the first
    #        waypoint of the next JointSpaceCommand
    #   These checks allow us to ensure we create valid robot commands.
    commands: List[Command]

    def _commands_for_type(self, command_type: str) -> List[Command]:
        return [command for command in self.commands if command.type == command_type]

    def _validate_joint_space_paths(self):
        # Checks below enforce paths have the same dims and does error checking
        if not ENFORCE_JOINT_DIMENSIONALITIES:
            return

        # noinspection PyTypeChecker
        joint_space_paths: List[JointSpacePath] = self._commands_for_type(
            JointSpacePath.type
        )
        if not joint_space_paths:
            # No need to check if no JointSpacePath commands exists
            return

        # Check that the joint names are the same for each path hence enforcing
        # dimensionalities
        joint_names = set(frozenset(path.joint_names) for path in joint_space_paths)
        if len(joint_names) != 1:
            raise ValueError("Joint names are different across joint space paths")

        # Check that the initial joint positions for each JointSpacePath are consistent
        # i.e., the first joint position in a JointSpacePath is the same as the last
        # joint position in the previous JointSpacePath
        # TODO(willshen): load initial joint positions from robot model file
        # Use joint position in last waypoint in first path as initial joint position
        current_joint_positions = joint_space_paths[0].waypoint(-1)
        for path in joint_space_paths[1:]:
            if path.waypoint(0) != current_joint_positions:
                raise ValueError(
                    "Joint positions between JointSpacePaths are inconsistent"
                )
            # Set current joint position to last waypoint
            current_joint_positions = path.waypoint(-1)

    def _validate_actuate_grippers(self):
        # Checks below enforce gripper commands have the same dims
        if not ENFORCE_JOINT_DIMENSIONALITIES:
            return

        # noinspection PyTypeChecker
        actuate_grippers: List[ActuateGripper] = self._commands_for_type(
            ActuateGripper.type
        )
        if not actuate_grippers:
            # No need to check if no ActuateGripper commands exists
            return

        # Check that the joint names are the same for each ActuateGripper command
        joint_names = set(
            frozenset(command.joint_names) for command in actuate_grippers
        )
        if len(joint_names) != 1:
            raise ValueError(
                "Joint names are different across actuate gripper commands"
            )

    def _validate_commands(self):
        # Check that commands confirm to Command type
        for command in self.commands:
            if not isinstance(command, Command):
                raise ValueError(f"Invalid command type: {type(command)}")
            elif not isinstance(command, SUPPORTED_COMMAND_TYPES):
                # If a new Command type is added, there should be a new validation
                # check for it.
                raise ValueError(f"Unsupported command type: {command.type}")

        # Validate the individual commands
        self._validate_joint_space_paths()
        self._validate_actuate_grippers()

    def validate(self):
        # Check path of LISDF exists
        if not os.path.exists(self.lisdf_path):
            # TODO(willshen): validate models/lisdf/sdf exists within path?
            raise ValueError(f"LISDF path does not exist: {self.lisdf_path}")

        # Check version number is valid and supported
        major, minor = self.version.split(".")
        if not major.isdigit() or not minor.isdigit():
            raise ValueError(f"Invalid version number: {self.version}")

        if self.version not in SUPPORTED_PLANNER_OUTPUT_VERSIONS:
            raise ValueError(f"Unsupported version number: {self.version}")

        self._validate_commands()

    @classmethod
    def from_json_dict(cls, json_dict: Dict) -> "LISDFPlan":
        # Convert the commands individually as they have their own types
        commands = [
            Command.from_json_dict(command_dict)
            for command_dict in json_dict["commands"]
        ]
        json_dict["commands"] = commands

        return cls(**json_dict)
