import warnings

import numpy as np
from pydrake.systems.framework import BasicVector, LeafSystem

from lisdf.plan_executor.lisdf_executor import LISDFPlanExecutor, NoExecutorFoundError


class LISDFPlanController(LeafSystem):
    def __init__(self, plan_executor: LISDFPlanExecutor):
        super().__init__()
        self.plan_executor = plan_executor
        self.robot = self.plan_executor.robot

        self._last_qv_des = None
        self._warned_no_executor = False

        # Declare output port
        self.dimensionality = 2 * self.robot.dimensionality
        self.DeclareVectorOutputPort(
            "robot_qv_des", BasicVector(self.dimensionality), self.CalcOutput
        )

    def CalcOutput(self, context, output):
        """
        Drake will compute updates linear in time (i.e., time only progresses so our
        check for NoExecutorFoundError is for when the simulation goes on for longer
        than the plan duration.
        """
        # Update the robot configuration by running the command executor
        current_time = context.get_time()
        try:
            self.plan_executor.execute(current_time)

            # Stack current robot state with 0 velocities
            q_des = self.robot.configuration
            v_des = np.zeros_like(q_des)
            qv_des = np.concatenate((q_des, v_des))
            self._last_qv_des = qv_des
        except NoExecutorFoundError:
            if not self._warned_no_executor:
                warnings.warn(
                    f"No executor found for the current time {current_time}. "
                    "The controller will output the last known desired "
                    "configuration and velocity."
                )
                self._warned_no_executor = True

            qv_des = self._last_qv_des
            if qv_des is None:
                raise RuntimeError("No executor found and no previous qv_des found")

        output.SetFromVector(qv_des)
