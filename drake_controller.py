from pydrake.systems.framework import LeafSystem, BasicVector

class PandaControllerSystem(LeafSystem):
    def __init__(self, poses):
        LeafSystem.__init__(self)
        self.poses = poses
        self.DeclareVectorOutputPort("panda_qv_des", BasicVector(18), self.CalcOutput)

    def CalcOutput(self, context, output):
        pose_idx = max(len(self.poses) - 1, context.get_time()/2.0)
        q_des = self.poses[pose_idx]
        v_des = np.zeros(9)
        qv_des = np.concatenate((q_des, v_des))
        output.SetFromVector(qv_des)


