import numpy as np
from pydrake.systems.framework import BasicVector, LeafSystem

from drake_utils.joint_space_path import JointSpacePaths


class RobotJointSpaceController(LeafSystem):
    def __init__(self, joint_space_paths: JointSpacePaths):
        """
        poses is a list of numpy.ndarray objects.
        Each object has shape (N, ) where N is the degrees of freedom of the robot.
        E.G: On systems with the Panda, there are 7 joints and 2 fingers -
        So the planner should output a list of poses with shape (9, )
        """
        super().__init__()
        self.joint_space_paths = joint_space_paths

        # Compute dimensionality and create output port
        self.dimensionality = 2 * joint_space_paths.joint_dimensionality()
        print("dim: ", self.dimensionality)

        self.last_qv_des = None
        self.DeclareVectorOutputPort(
            "robot_qv_des", BasicVector(self.dimensionality), self.CalcOutput
        )

    def CalcOutput(self, context, output):
        # Determine which joint space path to use
        time = context.get_time()
        joint_space_path = self.joint_space_paths.path_at_time(time)

        # q, v and set output port
        if joint_space_path:
            q_des = joint_space_path.conf_at_time(time)
            v_des = np.zeros_like(q_des)
            qv_des = np.concatenate((q_des, v_des))
        else:
            qv_des = self.last_qv_des

        self.last_qv_des = qv_des
        output.SetFromVector(qv_des)
