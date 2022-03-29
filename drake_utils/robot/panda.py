from typing import List

import numpy as np

from drake_utils.robot.robot import RobotWithGripper
from lisdf.planner_output.command import GripperPosition


class Panda(RobotWithGripper):
    @property
    def joint_configuration(self) -> np.array:
        return self.configuration[:7]

    def set_joint_configuration(self, joint_configuration: np.array) -> None:
        self.configuration = np.concat(
            (joint_configuration, self.gripper_configuration)
        )

    @property
    def gripper_configuration(self) -> np.array:
        return self.configuration[7:]

    def set_gripper_configuration(self, gripper_configuration: np.array) -> None:
        self.configuration = np.concat(
            (self.joint_configuration, gripper_configuration)
        )

    def gripper_configuration_for_position(self, position: GripperPosition) -> np.array:
        if position == GripperPosition.open:
            return np.array([0.03, 0.03])
        elif position == GripperPosition.close:
            return np.array([0.0, 0.0])
        else:
            raise ValueError(f"Unknown gripper position: {position}")

    @property
    def dimensionality(self) -> int:
        # 7 arm joints + 2 gripper joints
        return 9

    def joint_ordering(self) -> List[str]:
        return [
            "panda_joint1",
            "panda_joint2",
            "panda_joint3",
            "panda_joint4",
            "panda_joint5",
            "panda_joint6",
            "panda_joint7",
        ]
