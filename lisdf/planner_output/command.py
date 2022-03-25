from abc import ABC
from dataclasses import dataclass
from enum import Enum
from typing import ClassVar, Dict, List, Optional

import numpy as np

from lisdf.planner_output.common import OutputElement
from lisdf.planner_output.config import ENFORCE_JOINT_DIMENSIONALITIES


class Command(OutputElement, ABC):
    """
    Hack based on Jiayuan's code for enforcing subclasses to define a type which is
    then used for serialization.
    """

    # TODO: figure out a clean way to move the 'label' field here
    type: ClassVar[str]

    def __init_subclass__(cls, type: str, **kwargs):
        super().__init_subclass__(**kwargs)
        setattr(cls, "type", type)

    def to_dict(self) -> Dict:
        output = {"type": self.type}
        output = {**output, **super().to_dict()}
        return output


@dataclass(frozen=True)
class Waypoint(OutputElement):
    # Mapping of Joint Name to Position
    configurations: Dict[str, float]

    @property
    def dimensionality(self):
        return len(self.configurations)

    def values_as_list(self) -> List[float]:
        return list(self.configurations.values())

    def values_as_np_array(self) -> np.ndarray:
        return np.array(self.values_as_list())

    def validate(self):
        if not self.configurations:
            raise ValueError("Waypoint must have at least one configuration")


@dataclass(frozen=True)
class JointSpacePath(Command, type="JointSpacePath"):
    waypoints: List[Waypoint]
    duration: Optional[float] = None
    label: Optional[str] = None

    def validate(self):
        if len(self.waypoints) < 2:
            raise ValueError(
                f"There must be at least two waypoints in {self.type}. "
                "The first waypoint should indicate the initial configuration."
            )

        # Check that all waypoints have the same keys and hence dimensionality
        if ENFORCE_JOINT_DIMENSIONALITIES:
            waypoint_keys = {
                frozenset(waypoint.configurations.keys()) for waypoint in self.waypoints
            }
            if len(waypoint_keys) != 1:
                raise ValueError(
                    f"Waypoints in {self.type} must have the same joint names"
                )

            # Sanity check they have the same dimensionality
            waypoint_dimensionalities = set(
                waypoint.dimensionality for waypoint in self.waypoints
            )
            if len(waypoint_dimensionalities) != 1:
                raise ValueError(
                    f"Waypoints in {self.type} must have the same dimensionality"
                )

        if self.duration is not None and not self.duration > 0:
            raise ValueError(f"Duration must be positive in {self.type}")


class GripperPosition(Enum):
    OPEN = "open"
    CLOSE = "close"


@dataclass(frozen=True)
class ActuateGripper(Command, type="ActuateGripper"):
    # Mapping of Gripper Joint Name to GripperPosition (i.e., open or close)
    configurations: Dict[str, GripperPosition]
    label: Optional[str] = None

    def position_for_gripper_joint(self, gripper_joint: str) -> GripperPosition:
        if gripper_joint not in self.configurations:
            raise ValueError(f"Gripper joint {gripper_joint} not in configuration")

        return self.configurations[gripper_joint]

    def validate(self):
        if not self.configurations:
            raise ValueError(f"Empty configurations in {self.type}")

        for gripper_joint, gripper_position in self.configurations.items():
            if gripper_position not in [GripperPosition.OPEN, GripperPosition.CLOSE]:
                raise ValueError(
                    f"Invalid gripper position {gripper_position} for "
                    f"gripper joint {gripper_joint}"
                )
