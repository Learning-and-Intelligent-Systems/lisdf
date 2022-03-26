import os
from dataclasses import dataclass
from typing import List

from lisdf.planner_output.command import Command
from lisdf.planner_output.common import OutputElement
from lisdf.planner_output.config import SUPPORTED_PLANNER_OUTPUT_VERSIONS


@dataclass(frozen=True)
class LISDFPlan(OutputElement):
    lisdf_path: str
    version: str
    commands: List[Command]

    def validate(self):
        # Check path of LISDF exists
        if not os.path.exists(self.lisdf_path):
            # TODO(willshen): validate model.lisdf/sdf exists within path?
            raise ValueError(f"LISDF path does not exist: {self.lisdf_path}")

        # Check version number is valid and supported
        major, minor = self.version.split(".")
        if not major.isdigit() or not minor.isdigit():
            raise ValueError(f"Invalid version number: {self.version}")

        if self.version not in SUPPORTED_PLANNER_OUTPUT_VERSIONS:
            raise ValueError(f"Unsupported version number: {self.version}")

        # Check commands are expected types
        for command in self.commands:
            if not isinstance(command, Command):
                raise ValueError(f"Invalid command type: {type(command)}")

        # FIXME: validate commands are in sequence
        # FIXME: error check init joint position is valid across commands
        # FIXME: validate joint dimensionalities make sense across the commands
