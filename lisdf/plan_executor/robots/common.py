from abc import ABC, abstractmethod
from typing import List

import numpy as np

from lisdf.planner_output.command import GripperPosition


class Robot(ABC):
    """
    A Robot model for the plan execution.
    """

    def __init__(self, configuration: np.ndarray):
        self.configuration = configuration

    @property
    @abstractmethod
    def joint_configuration(self) -> np.ndarray:
        """
        Joint configuration, which may be a subset of the full robot configuration.
        """
        raise NotImplementedError

    @abstractmethod
    def set_joint_configuration(self, joint_configuration: np.ndarray) -> None:
        """
        Set the joint configuration, which may be a subset of the full robot
        configuration.
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


class RobotWithGripper(Robot, ABC):
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
