import numpy as np
from pydrake.multibody.plant import MultibodyPlant
from pydrake.systems.framework import BasicVector, Context, LeafSystem

from lisdf.plan_executor.robot import Robot


class LISDFPlanAnimator(LeafSystem):
    def __init__(
        self, robot: Robot, plant: MultibodyPlant, sim_context: Context
    ):
        super().__init__()
        self.plant = plant
        self.plant_robot_ref = plant.GetModelInstanceByName("robot")
        self.sim_context = sim_context
        self.dimensionality = robot.dimensionality

        self.qv_port = self.DeclareVectorInputPort(
            "desired_robot_qv", BasicVector(2 * robot.dimensionality)
        )

        # Declare output port
        self.DeclareVectorOutputPort(
            "torques", BasicVector(self.dimensionality), self.CalcOutput
        )

    def CalcOutput(self, context, output):
        desired_qv = self.qv_port.Eval(context)
        self.plant.SetPositionsAndVelocities(
            self.sim_context, self.plant_robot_ref, desired_qv
        )
        torques = np.zeros((self.dimensionality,))
        output.SetFromVector(torques)
