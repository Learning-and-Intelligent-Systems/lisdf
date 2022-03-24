#! /usr/bin/env python3
# -*- coding: utf-8 -*-
# File   : model.py
# Author : Jiayuan Mao
# Email  : maojiayuan@gmail.com
# Date   : 03/23/2022
#
# Distributed under terms of the MIT license.

from dataclasses import dataclass, field
from functools import cached_property
from typing import List, Optional

import numpy as np

from lisdf.components.base import StringConfigurable
from lisdf.components.control import ControlInfo, JointInfo
from lisdf.components.sensor import Sensor
from lisdf.components.shape import ShapeInfo
from lisdf.components.visual import VisualInfo
from lisdf.utils.transformations import euler_from_quaternion, quaternion_from_euler
from lisdf.utils.typing import Vector3f, Vector4f, Vector6f


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


@dataclass
class Inertia(StringConfigurable):
    ixx: float
    ixy: float
    ixz: float
    iyy: float
    iyz: float
    izz: float

    @classmethod
    def zeros(cls) -> "Inertia":
        return cls(0, 0, 0, 0, 0, 0)

    @classmethod
    def from_diagonal(cls, ixx, iyy, izz) -> "Inertia":
        return cls(ixx, 0, 0, iyy, 0, izz)

    @property
    def matrix(self) -> np.ndarray:
        return np.array(
            [
                [self.ixx, self.ixy, self.ixz],
                [self.ixy, self.iyy, self.iyz],
                [self.ixz, self.iyz, self.izz],
            ],
            dtype=np.float32,
        )


@dataclass
class Inertial(StringConfigurable):
    mass: float
    pose: Pose
    inertia: Inertia

    @classmethod
    def zeros(cls) -> "Inertial":
        return cls(0, Pose.identity(), Inertia.zeros())


@dataclass
class SurfaceContact(StringConfigurable):
    pass


@dataclass
class SurfaceFriction(StringConfigurable):
    pass


@dataclass
class Surface(StringConfigurable):
    contact: Optional[SurfaceContact] = None
    friction: Optional[SurfaceFriction] = None


@dataclass
class Geom(StringConfigurable):
    name: str
    pose: Pose
    shape: ShapeInfo
    visual: Optional[VisualInfo] = None
    surface: Optional[Surface] = None

    @property
    def type(self):
        return self.shape.type


@dataclass
class Joint(StringConfigurable):
    name: str
    parent: str
    child: str
    pose: Pose
    joint_info: JointInfo
    control_info: Optional[ControlInfo] = None

    @property
    def type(self):
        return self.joint_info.type

    # TODO(Jiayuan Mao @ 03/24): add a link to the corresponding `model`,
    # so we can use joint.parent_link to access the corresponding link
    # object.


@dataclass
class Link(StringConfigurable):
    name: str
    parent: Optional[str]
    pose: Pose
    inertial: Optional[Inertial] = None
    collisions: List[Geom] = field(default_factory=list)
    visuals: List[Geom] = field(default_factory=list)
    sensors: List[Sensor] = field(default_factory=list)


@dataclass
class Model(StringConfigurable):
    name: str
    pose: Pose
    parent: Optional[str] = None
    static: Optional[bool] = False

    links: List[Link] = field(default_factory=list)
    joints: List[Joint] = field(default_factory=list)


@dataclass
class URDFModel(StringConfigurable):
    name: str
    uri: str
    size: Vector3f
    pose: Pose
    parent: Optional[str] = None
    static: Optional[bool] = False


@dataclass
class World(StringConfigurable):
    name: Optional[str] = None
    static: Optional[bool] = False
    models: List[Model] = field(default_factory=list)
