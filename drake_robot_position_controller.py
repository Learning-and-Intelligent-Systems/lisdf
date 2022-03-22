from pydrake.systems.framework import LeafSystem, BasicVector
import numpy as np

class DrakeRobotPositionController(LeafSystem):
    def __init__(self, poses):
        """
        poses is a list of numpy.ndarray objects.
        Each object has shape (N, ) where N is the degrees of freedom of the robot.
        E.G: On systems with the Panda, there are 7 joints and 2 fingers - So the planner should
        output a list of poses with shape (9, )
        """
        LeafSystem.__init__(self)
        self.poses = poses
        self.dimensionality = 2*poses[0].shape[0]
        print("dim: ", self.dimensionality)
        self.DeclareVectorOutputPort("robot_qv_des", BasicVector(self.dimensionality), self.CalcOutput)

    def CalcOutput(self, context, output):
        pose_idx = max(len(self.poses) - 1, context.get_time()/2.0)
        q_des = self.poses[pose_idx]
        v_des = np.zeros_like(self.poses[pose_idx])
        qv_des = np.concatenate((q_des, v_des))
        output.SetFromVector(qv_des)


