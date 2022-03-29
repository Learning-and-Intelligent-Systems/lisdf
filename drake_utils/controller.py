from typing import Tuple

import numpy as np
from pydrake.systems.framework import BasicVector, LeafSystem

from drake_utils.lisdf_executor.joint_space_path_executor import JointSpacePathExecutor
from drake_utils.lisdf_executor.robot import DrakeRobot
from lisdf.planner_output.command import ActuateGripper, GripperPosition
from lisdf.planner_output.plan import LISDFPlan


class LISDFPlanController(LeafSystem):
    def __init__(self, robot: DrakeRobot, plan: LISDFPlan):
        super().__init__()
        self.robot = robot
        self.plan = plan
        self.dimensionality = 2 * robot.dimensionality

        # FIXME: init can be read from LISDF
        # if last_q_des is None and the first command is ActuateGripper, we crash)
        self.last_q_des = np.zeros((robot.dimensionality,))
        self.DeclareVectorOutputPort(
            "robot_qv_des", BasicVector(self.dimensionality), self.CalcOutput
        )

    def CalcOutput(self, context, output):
        current_time = context.get_time()

        # Get the current command and run it
        executor = self._get_executor(current_time)

        command = self.plan.commands[self._current_command_idx]
        q_des, completion = self._run_command(command, time)

        v_des = np.zeros_like(q_des)
        qv_des = np.concatenate((q_des, v_des))
        self.last_q_des = q_des

        if completion:
            if self._current_command_idx < len(self.plan.commands) - 1:
                self._current_command_idx += 1
                self.command_start_time = time
                print(
                    "Progressed to:",
                    self.plan.commands[self._current_command_idx].label,
                )

        output.SetFromVector(qv_des)

    def run_jsp(
        self, executor: JointSpacePathExecutor, current_time: float
    ) -> np.ndarray:
        q_des_joint = executor.configuration(current_time)
        q_des_gripper = self.last_q_des[7:]
        return np.concatenate([q_des_joint, q_des_gripper])

    def run_gripper(
        self, command: ActuateGripper, current_time: float
    ) -> Tuple[np.ndarray, bool]:
        configuration = list(command.configurations.values())[0]
        q_des_joint = self.last_q_des[:7]
        if configuration == GripperPosition.open:
            q_des_gripper = np.array([0.03, 0.03])
        elif configuration == GripperPosition.close:
            q_des_gripper = np.array([0.0, 0.0])
        return np.concatenate([q_des_joint, q_des_gripper]), True
