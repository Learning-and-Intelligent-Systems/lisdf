from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum
from typing import ClassVar, Dict, List, Optional, Type

import numpy as np

from lisdf.planner_output.common import OutputElement


class Command(OutputElement, ABC):
    """
    Hack based on Jiayuan's code for enforcing subclasses to define a type which is
    then used for serialization.

    When inherit from this class, child classes should pass type="XXX"
    as a keyword argument. This will register a new Command type
    in this class which we use when deserializing from a JSON dict.
    """

    # TODO: figure out a clean way to move the 'label' field here

    # The type of the command
    type: ClassVar[str]

    type_mapping: ClassVar[Dict[str, Type["Command"]]] = dict()

    def __init_subclass__(cls, type: str, **kwargs):
        super().__init_subclass__(**kwargs)
        setattr(cls, "type", type)
        Command.type_mapping[type] = cls

    @classmethod
    @abstractmethod
    def _from_json_dict(cls, json_dict: Dict) -> "Command":
        # Children of this class should implement this method
        raise NotImplementedError

    @classmethod
    def from_json_dict(cls, json_dict: Dict) -> "Command":
        """
        Children of this class SHOULD NOT override this method and should
        implement _from_json_dict instead. This is so we can use the right
        implementation when loading in the main LISDFPlan class.
        """
        # Check type is in the mapping
        if json_dict["type"] not in Command.type_mapping:
            raise ValueError(f"Command type {json_dict['type']} not supported")

        # Delete type as that is a classvar
        type_cls = Command.type_mapping[json_dict["type"]]
        del json_dict["type"]

        return type_cls._from_json_dict(json_dict)

    def to_dict(self) -> Dict:
        # Add the type to the output dict
        output = {"type": self.type}
        output = {**output, **super().to_dict()}
        return output


JointName = str


@dataclass(frozen=True)
class JointSpacePath(Command, type="JointSpacePath"):
    # Mapping of joint name to a list of joint positions.
    #   The length of the list of joint positions must be the same for each joint,
    #   as each element indicates a single waypoint.
    waypoints: Dict[JointName, List[float]]

    # Duration of the path in seconds, if specified must be > 0
    duration: Optional[float] = None

    # Label to denote this path by
    label: Optional[str] = None

    @property
    def joint_names(self) -> List[JointName]:
        return list(self.waypoints.keys())

    @property
    def dimensionality(self) -> int:
        """Dimensionality is number of joints"""
        return len(self.waypoints)

    @property
    def num_waypoints(self) -> int:
        """
        Number of waypoints - i.e., how long is the list of joint positions
        """
        return len(next(iter(self.waypoints.values())))

    def waypoints_for_joint(self, joint_name: str) -> List[float]:
        """
        Get all the waypoints for a given joint
        """
        if joint_name not in self.waypoints:
            raise ValueError(
                f"Joint {joint_name} not found in waypoints for {self.type}"
            )
        return self.waypoints[joint_name]

    def waypoint(self, waypoint_index: int) -> Dict[str, float]:
        """
        Get the joint positions at a given waypoint index
        """
        if not -self.num_waypoints <= waypoint_index < self.num_waypoints:
            raise ValueError(
                f"Waypoint index {waypoint_index} out of range in {self.type}"
            )

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
        assert joint_positions_array.shape == (len(joint_name_ordering),)
        return joint_positions_array

    def waypoints_as_np_array(self, joint_name_ordering: List[str]) -> np.ndarray:
        """
        Return the joint positions as a numpy array. The shape of this array is
        (num_waypoints, num_joints).
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

        # Take transpose so we get array with shape (num_waypoints, num_joints)
        joint_positions_array = joint_positions_array.T
        assert joint_positions_array.shape == (
            self.num_waypoints,
            len(joint_name_ordering),
        )

        return joint_positions_array

    @classmethod
    def from_waypoints_np_array(
        cls,
        joint_positions_array: np.ndarray,
        joint_names: List[str],
        duration: Optional[float] = None,
        label: Optional[str] = None,
    ) -> JointSpacePath:
        """Create a JointSpacePath from an array of shape
        (num_waypoints, num_joints) and a list of joint names of length
        num_joints whose order corresponds to the array."""
        _, num_joints = joint_positions_array.shape
        assert len(joint_names) == num_joints
        waypoints = {
            name: joint_positions_array[:, i].tolist()
            for i, name in enumerate(joint_names)
        }
        return JointSpacePath(waypoints, duration, label)

    def get_reversed_path(self) -> "JointSpacePath":
        """Get the reversed joint space path."""
        return JointSpacePath(
            waypoints={
                joint_name: list(reversed(positions))
                for joint_name, positions in self.waypoints.items()
            },
            duration=self.duration,
            label=f"{self.label}_reversed",
        )

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

    @classmethod
    def _from_json_dict(cls, json_dict: Dict) -> "JointSpacePath":
        return cls(**json_dict)


class GripperPosition(Enum):
    open = "open"
    close = "close"


@dataclass(frozen=True)
class ActuateGripper(Command, type="ActuateGripper"):
    # Mapping of Gripper Joint Name to GripperPosition (i.e., open or close)
    configurations: Dict[JointName, GripperPosition]

    # Label to denote this gripper actuation by
    label: Optional[str] = None

    @property
    def joint_names(self) -> List[JointName]:
        return list(self.configurations.keys())

    def position_for_gripper_joint(self, gripper_joint: str) -> GripperPosition:
        if gripper_joint not in self.configurations:
            raise ValueError(f"Gripper joint {gripper_joint} not in configuration")

        return self.configurations[gripper_joint]

    def validate(self):
        if not self.configurations:
            raise ValueError(f"Empty configurations in {self.type}")

        for gripper_joint, gripper_position in self.configurations.items():
            if gripper_position not in [GripperPosition.open, GripperPosition.close]:
                raise ValueError(
                    f"Invalid gripper position {gripper_position} for "
                    f"gripper joint {gripper_joint}"
                )

    @classmethod
    def _from_json_dict(cls, json_dict: Dict) -> "ActuateGripper":
        # Overwrite the GripperPosition to their enum representations
        new_configurations = {
            joint_name: GripperPosition(gripper_position)
            for joint_name, gripper_position in json_dict["configurations"].items()
        }
        json_dict["configurations"] = new_configurations
        return cls(**json_dict)
