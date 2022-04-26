from typing import ClassVar, List, Optional

import numpy as np

from lisdf.plan_executor.robots.common import RobotWithGripper
from lisdf.planner_output.command import GripperPosition


class PR2(RobotWithGripper):
    """PR2 robot."""

    INITIAL_CONFIGURATION: ClassVar[np.ndarray] = np.array(
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.04, 0.04]
    )

    @property
    def joint_configuration(self) -> np.ndarray:
        return self.configuration[:7]

    def set_joint_configuration(self, joint_configuration: np.ndarray, joint_names: Optional[List[str]]) -> None:
        self.configuration = np.concatenate(
            [joint_configuration, self.gripper_configuration]
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
            "torso_lift_joint",
            "l_shoulder_pan_joint",
            "l_shoulder_lift_joint",
            "l_upper_arm_roll_joint",
            "l_elbow_flex_joint",
            "l_forearm_roll_joint",
            "l_wrist_flex_joint"
            "l_wrist_roll_joint",
            "r_shoulder_pan_joint",
            "r_shoulder_lift_joint",
            "r_upper_arm_roll_joint",
            "r_elbow_flex_joint",
            "r_forearm_roll_joint",
            "r_wrist_flex_joint"
            "r_wrist_roll_joint",
        ]

