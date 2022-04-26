from abc import ABC, abstractmethod
from typing import List, Optional

import numpy as np

from lisdf.planner_output.command import GripperPosition


class Robot(ABC):
    """
    A Robot model for the plan execution.
    """

    def __init__(self, configuration: np.ndarray):
        # Full robot configuration which may be a superset of the joint configuration
        self.configuration = configuration

    @property
    @abstractmethod
    def joint_configuration(self) -> np.ndarray:
        """
        Robot joint configuration.
        """
        raise NotImplementedError

    @abstractmethod
    def set_joint_configuration(
        self, joint_configuration: np.ndarray, joint_names: Optional[List[str]] = None
    ) -> None:
        """
        Set the joint configuration of the robot. The supplied joint configuration may
        be a subset of the full robot configuration if `joint_names` is specified.

        Parameters
        ----------
        joint_configuration: joint configuration of the robot
        joint_names: joint names the configuration specifies. If None, then
            we assume the configuration specifies the entire robot.

        Returns
        -------
        None
        """
        raise NotImplementedError

    @property
    @abstractmethod
    def dimensionality(self) -> int:
        raise NotImplementedError

    @property
    @abstractmethod
    def joint_ordering(self) -> List[str]:
        raise NotImplementedError

    @property
    def joint_names(self) -> List[str]:
        return self.joint_ordering

    @property
    def num_joints(self) -> int:
        return len(self.joint_ordering)


class RobotWithGripper(Robot, ABC):
    @property
    def joint_configuration(self) -> np.ndarray:
        return self.configuration[: self.num_joints]

    def set_joint_configuration(
        self, joint_configuration: np.ndarray, joint_names: Optional[List[str]] = None
    ) -> None:
        if not joint_names:
            # Setting full configuration of the arm (not gripper)
            if len(joint_configuration) != self.num_joints:
                raise ValueError(
                    f"Expected {self.num_joints} joint configuration values, "
                    f"got {len(joint_configuration)} values"
                )
            self.configuration = np.concatenate(
                [joint_configuration, self.gripper_configuration]
            )
        else:
            # Setting subset of joints - run some sanity checks first
            if len(joint_configuration) != len(joint_names):
                raise ValueError(
                    "Length of joint configuration does not match number of joint names"
                )
            if len(joint_configuration) > self.num_joints:
                raise ValueError(
                    "Length of joint configuration is greater than number of joints."
                    "You shouldn't include gripper joints here."
                )

            # Get the indices of the joints we want to set
            for joint_name in joint_names:
                if joint_name not in self.joint_ordering:
                    raise ValueError(
                        f"Joint {joint_name} does not exist in {type(self).__name__}"
                    )

            joint_idxs = [
                self.joint_ordering.index(joint_name) for joint_name in joint_names
            ]
            self.configuration[joint_idxs] = joint_configuration

    @abstractmethod
    def gripper_configuration_for_position(
        self, position: GripperPosition
    ) -> np.ndarray:
        raise NotImplementedError

    @property
    @abstractmethod
    def gripper_configuration(self) -> np.ndarray:
        """
        Gripper configuration, which may be a subset of the full robot configuration.
        """
        raise NotImplementedError

    @abstractmethod
    def set_gripper_configuration(self, gripper_configuration: np.ndarray) -> None:
        """
        Set the Gripper configuration, which may be a subset of the full robot
        configuration.
        """
        raise NotImplementedError

    @property
    @abstractmethod
    def gripper_joint_ordering(self) -> List[str]:
        raise NotImplementedError
