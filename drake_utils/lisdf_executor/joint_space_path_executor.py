import warnings
from typing import ClassVar

import numpy as np
from pydrake.trajectories import PiecewisePolynomial

from drake_utils.lisdf_executor.executor import CommandExecutor
from drake_utils.robot import DrakeRobot
from lisdf.planner_output.command import JointSpacePath


class JointSpacePathExecutor(CommandExecutor):
    _DEFAULT_JSP_DURATION: ClassVar[float] = 5.0

    def __init__(self, robot: DrakeRobot, path: JointSpacePath, start_time: float):
        super().__init__(robot, path, start_time)
        # Determine duration if specified
        self._duration = (
            path.duration
            if path.duration
            else JointSpacePathExecutor._DEFAULT_JSP_DURATION
        )

        # Using a FirstOrderHold for now so the code can later be generalized
        # to fancier interpolation schemes (as opposed to just solving the linear eq)
        confs = path.waypoints_as_np_array(joint_name_ordering=robot.joint_ordering)
        t_all = np.linspace(
            start_time, start_time + self.duration, num=path.num_waypoints
        )
        self._first_order_hold = PiecewisePolynomial.FirstOrderHold(t_all, confs.T)

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

        # Use FOH to determine configuration for joints
        q_joint = self._first_order_hold.value(current_time)
        q_joint = q_joint.reshape(-1)

        # Update the joint configuration of the robot (not the gripper)
        self.robot.set_joint_configuration(q_joint)
