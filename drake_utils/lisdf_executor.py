import numpy as np
from pydrake.systems.framework import BasicVector, LeafSystem
from pydrake.trajectories import PiecewisePolynomial
from lisdf.planner_output.plan import LISDFPlan
from lisdf.planner_output.command import JointSpacePath, ActuateGripper, GripperPosition


class LISDFPlanExecutor(LeafSystem):
    def __init__(self, plan: LISDFPlan, dimensionality: int):
        super().__init__()
        self.dimensionality = 2 * dimensionality
        self.plan = plan

        self.current_command_idx = 0
        self.command_start_time = 0.0
        self.joint_ordering = ["panda_joint1", "panda_joint2", "panda_joint3",
                               "panda_joint4", "panda_joint5", "panda_joint6",
                               "panda_joint7"]

        # FIXME: init can be read from LISDF
        # if last_q_des is None and the first command is ActuateGripper, we crash)
        self.last_q_des = np.zeros((9, ))
        self.DeclareVectorOutputPort(
            "robot_qv_des", BasicVector(self.dimensionality), self.CalcOutput)

        self.command_executors = {JointSpacePath.type: self.run_jsp,
                                  ActuateGripper.type: self.run_gripper}
        print(plan.commands[self.current_command_idx].label)

    def CalcOutput(self, context, output):
        time = context.get_time()
        current_command = self.plan.commands[self.current_command_idx]
        q_des, completion = self.command_executors[current_command.type](
                current_command, time)
        v_des = np.zeros_like(q_des)
        qv_des = np.concatenate((q_des, v_des))
        self.last_q_des = q_des
        if completion:
            if self.current_command_idx < len(self.plan.commands) - 1:
                self.current_command_idx += 1
                self.command_start_time = time
                print(self.plan.commands[self.current_command_idx].label)
        output.SetFromVector(qv_des)

    def run_jsp(self, command: JointSpacePath, current_time: int) -> np.ndarray:
        command_duration = 5.0 if command.duration is None else command.duration
        # Using a FirstOrderHold for now so the code can later be generalized
        # to fancier interpolation schemes (as opposed to just solving the linear eq)
        confs = command.waypoints_as_np_array(self.joint_ordering)
        num_confs = confs.shape[0]
        t_all = np.linspace(0, command_duration, num=num_confs)
        FOH = PiecewisePolynomial.FirstOrderHold(t_all, confs.T)
        q_des_joint = FOH.value(current_time)
        q_des_joint = np.resize(q_des_joint, (7, ))
        q_des_gripper = self.last_q_des[7:]

        time_in_command = current_time - self.command_start_time
        timeout = time_in_command > command_duration
        return np.concatenate([q_des_joint, q_des_gripper]), timeout

    def run_gripper(self, command: ActuateGripper, current_time: int) -> np.ndarray:
        configuration = list(command.configurations.values())[0]
        q_des_joint = self.last_q_des[:7]
        if configuration == GripperPosition.open:
            q_des_gripper = np.array([0.03, 0.03])
        elif configuration == GripperPosition.close:
            q_des_gripper = np.array([0.0, 0.0])
        return np.concatenate([q_des_joint, q_des_gripper]), True
