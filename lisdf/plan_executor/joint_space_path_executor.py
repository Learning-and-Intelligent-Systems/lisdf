import warnings
from abc import ABC, abstractmethod
from typing import ClassVar, Type

import numpy as np

from lisdf.plan_executor.executor import CommandExecutor
from lisdf.plan_executor.robot_state import RobotWithState
from lisdf.planner_output.command import JointSpacePath


class PathInterpolator(ABC):
    """Used so we can abstract simulator's interpolator out of the executor."""

    def __init__(self, t_all: np.ndarray, confs: np.ndarray):
        """
        Let n be the number of configurations and q be the number of joints
        in the configuration.

        Parameters
        ----------
        t_all: 1-d array of shape (n, ) with the times for each configuration point
        confs: 2-d array of shape (n, q) where each row represents a robot conf.
            Subsequently, there is a conf for each time in t_all.
        """
        if t_all.shape[0] != confs.shape[0]:
            raise ValueError("t_all and confs must be the same length.")

        self.t_all = t_all
        self.confs = confs

    @abstractmethod
    def value(self, t: float) -> np.ndarray:
        """
        Get the interpolated joint values for the given time t.
        Returns a 1-d array shape (q, ) where q is the number of joints.
        """
        raise NotImplementedError


class JointSpacePathExecutor(CommandExecutor):
    _DEFAULT_JSP_DURATION: ClassVar[float] = 5.0

    def __init__(
        self,
        robot: RobotWithState,
        path: JointSpacePath,
        start_time: float,
        interpolator_cls: Type[PathInterpolator],
    ):
        super().__init__(robot, path, start_time)
        # Determine duration if specified
        self._duration = (
            path.duration
            if path.duration
            else JointSpacePathExecutor._DEFAULT_JSP_DURATION
        )

        # Create interpolator
        confs = path.waypoints_as_np_array(joint_name_ordering=robot.joint_ordering)
        t_all = np.linspace(
            start_time, start_time + self.duration, num=path.num_waypoints
        )
        self._interpolator = interpolator_cls(t_all, confs)

        # For warning when outside duration
        self._warned_outside_time_limits = False

    @property
    def duration(self) -> float:
        return self._duration

    def execute(self, current_time: float) -> None:
        if (
            not (self.start_time <= current_time < self.end_time)
            and not self._warned_outside_time_limits
        ):
            warnings.warn(
                f"Current time {current_time} is not within the "
                "JointSpacePath time range"
            )
            self._warned_outside_time_limits = True

        # Use interpolator to get joint values
        q_joint = self._interpolator.value(current_time)

        # Update the joint configuration of the robot (not the gripper)
        self.robot.set_joint_configuration(q_joint)
