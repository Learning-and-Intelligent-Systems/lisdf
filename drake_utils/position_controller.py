import math
from typing import List

import numpy as np
from pydrake.systems.framework import BasicVector, LeafSystem


class RobotPositionController(LeafSystem):
    def __init__(self, poses: List[np.ndarray], time_step: float = 1.0):
        """
        poses is a list of numpy.ndarray objects.
        Each object has shape (N, ) where N is the degrees of freedom of the robot.
        E.G: On systems with the Panda, there are 7 joints and 2 fingers -
        So the planner should output a list of poses with shape (9, )
        """
        super().__init__()
        self.time_step = time_step

        # Check dimensionalities of poses are the same
        pose_dims = set(pose.shape for pose in poses)
        assert len(pose_dims) == 1, "Joint position shapes must be identical"
        pose_dim = next(iter(pose_dims))

        self.poses = poses

        # Compute dimensionality and create output port
        self.dimensionality = 2 * pose_dim[0]
        print("dim: ", self.dimensionality)

        self.DeclareVectorOutputPort(
            "robot_qv_des", BasicVector(self.dimensionality), self.CalcOutput
        )

    def CalcOutput(self, context, output):
        # Determine which pose to use
        pose_idx = min(
            len(self.poses) - 1, math.floor(context.get_time() / self.time_step)
        )

        # q, v and set output port
        q_des = self.poses[pose_idx]
        v_des = np.zeros_like(self.poses[pose_idx])
        qv_des = np.concatenate((q_des, v_des))
        output.SetFromVector(qv_des)
