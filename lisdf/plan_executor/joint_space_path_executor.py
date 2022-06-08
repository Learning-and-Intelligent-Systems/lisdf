import warnings
from typing import ClassVar, Type

import numpy as np

from lisdf.plan_executor.executor import CommandExecutor
from lisdf.plan_executor.interpolator import PathInterpolator
from lisdf.plan_executor.robots.common import Robot
from lisdf.planner_output.command import JointSpacePath


class JointSpacePathExecutor(CommandExecutor):
    _DEFAULT_JSP_DURATION: ClassVar[float] = 5.0

    def __init__(
        self,
        robot: Robot,
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

        # For warning when provided time is outside duration
        self._warned_outside_time_limits = False

    @property
    def duration(self) -> float:
        return self._duration

    def execute(self, current_time: float) -> None:
        # Check if current time is within the bounds for duration. If it is outside,
        # we rely on interpolator to provide the best value.
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
