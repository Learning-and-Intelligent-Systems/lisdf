from abc import ABC
from dataclasses import dataclass
from functools import cached_property

import numpy as np

from lisdf.utils.transformations import euler_from_quaternion, quaternion_from_euler
from lisdf.utils.transformations_more import lookat_rpy
from lisdf.utils.typing import Vector3f, Vector4f, Vector6f


class StringConfigurable(ABC):
    # TODO(Jiayuan Mao @ 03/24): implement these methods for the child classes.
    def to_sdf(self) -> str:
        raise NotImplementedError()

    def to_urdf(self) -> str:
        raise NotImplementedError()

    def to_mjcf(self) -> str:
        raise NotImplementedError()

    def to_yaml(self) -> str:
        raise NotImplementedError()


@dataclass
class Pose(StringConfigurable):
    pos: Vector3f
    quat_wxyz: Vector3f

    @classmethod
    def from_rpy_6d(cls, a: Vector6f) -> "Pose":
        return cls.from_rpy(a[:3], a[3:])

    @classmethod
    def from_rpy(cls, pos: Vector3f, rpy: Vector3f) -> "Pose":
        return cls.from_quat_xyzw(pos, quaternion_from_euler(*rpy))  # type: ignore

    @classmethod
    def from_quat_xyzw(cls, pos: Vector3f, xyzw: Vector4f) -> "Pose":
        return cls(pos, np.array([xyzw[3], xyzw[0], xyzw[1], xyzw[2]]))

    @classmethod
    def from_lookat(cls, xyz: Vector3f, point_to: Vector3f) -> "Pose":
        """Construct the roll, pitch, yaw angles of a camera looking at a target.
        This function assumes that the camera is pointing to the z-axis ([0, 0, 1]),
        in the camera frame.

        Args:
            camera_pos: the position of the camera.
            target_pos: the target position.

        Returns:
            a Pose object.
        """
        return cls.from_rpy(xyz, lookat_rpy(xyz, point_to))

    @classmethod
    def identity(cls) -> "Pose":
        return cls(
            pos=np.zeros(3, dtype="float32"),
            quat_wxyz=np.array([1, 0, 0, 0], dtype="float32"),
        )

    @cached_property
    def quat_xyzw(self) -> Vector4f:
        return np.array(
            [self.quat_wxyz[1], self.quat_wxyz[2], self.quat_wxyz[3], self.quat_wxyz[0]]
        )

    @cached_property
    def rpy(self) -> Vector3f:
        return euler_from_quaternion(self.quat_xyzw)  # type: ignore

    def to_sdf(self) -> str:
        return (
            f"<pose>{self.pos[0]}, {self.pos[1]}, {self.pos[2]}, "
            f"{self.rpy[0]}, {self.rpy[1]}, {self.rpy[2]}</pose>"
        )
