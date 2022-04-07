import warnings

import numpy as np
from pydrake.systems.framework import BasicVector, LeafSystem

from drake_utils.interpolator_wrapper import DrakePiecewisePolynomialInterpolator
from lisdf.plan_executor.executor import CommandExecutor
from lisdf.plan_executor.gripper_executor import ActuateGripperExecutor
from lisdf.plan_executor.joint_space_path_executor import JointSpacePathExecutor
from lisdf.plan_executor.robot import RobotWithGripper, Robot
from lisdf.planner_output.command import ActuateGripper, Command, JointSpacePath
from lisdf.planner_output.plan import LISDFPlan


class LISDFPlanController(LeafSystem):
    def __init__(self, robot: Robot, plan: LISDFPlan):
        super().__init__()
        self.robot = robot
        self.plan = plan
        self.dimensionality = 2 * robot.dimensionality

        self._current_command_idx = 0
        self._current_executor = self._get_executor(
            command=self.plan.commands[self._current_command_idx], start_time=0.0
        )
        self._last_warning_printed = False

        # Declare output port
        self.DeclareVectorOutputPort(
            "robot_qv_des", BasicVector(self.dimensionality), self.CalcOutput
        )

    def _get_executor(self, command: Command, start_time: float) -> CommandExecutor:
        """Create command executor for the given command"""
        if command.type == JointSpacePath.type:
            command: JointSpacePath
            return JointSpacePathExecutor(
                self.robot,
                command,
                start_time,
                interpolator_cls=DrakePiecewisePolynomialInterpolator,
            )
        elif command.type == ActuateGripper.type:
            command: ActuateGripper
            self.robot: RobotWithGripper
            return ActuateGripperExecutor(self.robot, command, start_time)
        else:
            raise ValueError(f"Unsupported command type: {command.type}")

    def _update_current_executor(self, current_time: float):
        """Update the current CommandExecutor if required for the given time"""
        if not self._current_executor.finished(current_time):
            # Executor has not finished then we can keep using it
            return self._current_executor
        elif self._current_command_idx >= len(self.plan.commands) - 1:
            # We're using the last command and it has finished, so warn
            if not self._last_warning_printed:
                warnings.warn(
                    f"No more commands in plan after time {current_time}."
                    "Using last command."
                )
                self._last_warning_printed = True

            return self._current_executor
        else:
            # Previous executor has finished, so we need to move to the next executor
            self._current_command_idx += 1
            self._current_executor = self._get_executor(
                command=self.plan.commands[self._current_command_idx],
                start_time=current_time,
            )

    def CalcOutput(self, context, output):
        # Update the robot configuration by running the command executor
        current_time = context.get_time()
        self._update_current_executor(current_time)
        self._current_executor.execute(current_time)

        # Stack current robot state with 0 velocities
        q_des = self.robot.configuration
        v_des = np.zeros_like(q_des)
        qv_des = np.concatenate((q_des, v_des))
        output.SetFromVector(qv_des)
