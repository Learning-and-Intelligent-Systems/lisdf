from typing import ClassVar, List, Optional

import numpy as np

from lisdf.plan_executor.robots.common import RobotWithGripper
from lisdf.planner_output.command import GripperPosition


class Panda(RobotWithGripper):
    """
    Franka Emika Panda robot.

    First 7 entries of the configuration are the robot arm positions.
    Last 2 entries of the configuration are the gripper positions.
    Total = 9 joints.
    """

    INITIAL_CONFIGURATION: ClassVar[np.ndarray] = np.array(
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.04, 0.04]
    )

    @property
    def gripper_configuration(self) -> np.ndarray:
        return self.configuration[7:]

    def set_gripper_configuration(self, gripper_configuration: np.ndarray) -> None:
        self.configuration = np.concatenate(
            [self.joint_configuration, gripper_configuration]
        )

    def gripper_configuration_for_position(
        self, position: GripperPosition
    ) -> np.ndarray:
        if position == GripperPosition.open:
            return np.array([0.04, 0.04])
        elif position == GripperPosition.close:
            return np.array([0.0, 0.0])
        else:
            raise ValueError(f"Unknown gripper position: {position}")

    @property
    def dimensionality(self) -> int:
        # 7 arm joints + 2 gripper joints
        return 9

    @property
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

    @property
    def gripper_joint_ordering(self) -> List[str]:
        return ["panda_finger_joint1", "panda_finger_joint2"]
