from abc import ABC
from dataclasses import dataclass
from enum import Enum
from typing import ClassVar, Dict, List, Optional

import numpy as np

from lisdf.planner_output.common import OutputElement


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


JointName = str


@dataclass(frozen=True)
class JointSpacePath(Command, type="JointSpacePath"):
    # Mapping of Joint Names to Position waypoints
    waypoints: Dict[JointName, List[float]]
    duration: Optional[float] = None
    label: Optional[str] = None

    @property
    def dimensionality(self) -> int:
        """Dimensionality is number of joints"""
        return len(self.waypoints)

    @property
    def num_waypoints(self) -> int:
        """Number of waypoints - i.e., how long is the list of joint positions"""
        return len(next(iter(self.waypoints.values())))

    def waypoints_for_joint(self, joint_name: str) -> List[float]:
        """Get all the waypoints for a given joint"""
        if joint_name not in self.waypoints:
            raise ValueError(
                f"Joint {joint_name} not found in waypoints for {self.type}"
            )
        return self.waypoints[joint_name]

    def waypoint(self, waypoint_index: int) -> Dict[str, float]:
        """Get the joint positions at a given waypoint index"""
        if not 0 <= waypoint_index < self.num_waypoints:
            raise ValueError(
                f"Waypoint index {waypoint_index} out of range in {self.type}"
            )

        # Get the joint positions at a given waypoint
        return {
            joint_name: joint_positions[waypoint_index]
            for joint_name, joint_positions in self.waypoints.items()
        }

    def _check_joint_name_ordering(self, joint_name_ordering: List[str]) -> None:
        """
        Check that the joint name ordering is valid
        """
        if set(joint_name_ordering) != set(self.waypoints.keys()):
            raise ValueError(
                f"Joint names for ordering {joint_name_ordering} does not match "
                f"waypoint joint names {list(self.waypoints.keys())}"
            )

    def waypoint_as_np_array(
        self, waypoint_index: int, joint_name_ordering: List[str]
    ) -> np.ndarray:
        self._check_joint_name_ordering(joint_name_ordering)

        # Get the joint positions at a given waypoint
        joint_positions_at_waypoint = self.waypoint(waypoint_index)
        joint_positions_array = np.array(
            [
                joint_positions_at_waypoint[joint_name]
                for joint_name in joint_name_ordering
            ]
        )
        return joint_positions_array

    def waypoints_as_np_array(self, joint_name_ordering: List[str]) -> np.ndarray:
        """
        Return the joint positions as a numpy array
        """
        self._check_joint_name_ordering(joint_name_ordering)

        joint_name_to_waypoints = {
            joint_name: np.array(positions)
            for joint_name, positions in self.waypoints.items()
        }

        # Get a list of np.arrays in the order specified by the joint name ordering
        joint_positions = [
            joint_name_to_waypoints[joint_name] for joint_name in joint_name_ordering
        ]
        joint_positions_array = np.array(joint_positions)
        return joint_positions_array

    def validate(self):
        # Check waypoints dict is not None
        if not self.waypoints:
            raise ValueError(f"{self.type} must have at least one joint waypoint")

        # Check waypoints are list of floats
        for joint_positions in self.waypoints.values():
            if isinstance(joint_positions, list):
                # Check all the elements are numbers
                if not all(isinstance(pos, (int, float)) for pos in joint_positions):
                    raise ValueError(
                        f"{self.type} waypoints must be a list of int and/or floats"
                    )
            else:
                raise ValueError(f"{self.type} waypoints must be a list of floats")

        # Check joints have same number of waypoints
        num_waypoints = set(len(waypoints) for waypoints in self.waypoints.values())
        if len(num_waypoints) != 1:
            raise ValueError(
                f"{self.type} must have the same number of waypoints for all joints"
            )

        # Check there are at least two waypoints
        num_waypoints = num_waypoints.pop()
        if num_waypoints < 2:
            raise ValueError(
                f"There must be at least two waypoints in {self.type}. "
                "The first waypoint should indicate the initial configuration."
            )

        # Check duration is valid
        if self.duration is not None and not self.duration > 0:
            raise ValueError(f"Duration must be positive in {self.type}")


class GripperPosition(Enum):
    OPEN = "open"
    CLOSE = "close"


@dataclass(frozen=True)
class ActuateGripper(Command, type="ActuateGripper"):
    # Mapping of Gripper Joint Name to GripperPosition (i.e., open or close)
    configurations: Dict[JointName, GripperPosition]
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
