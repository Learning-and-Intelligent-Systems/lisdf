#! /usr/bin/env python3
# -*- coding: utf-8 -*-
# File   : model.py
# Author : Jiayuan Mao
# Email  : maojiayuan@gmail.com
# Date   : 03/23/2022
#
# Distributed under terms of the MIT license.

import numpy as np
from typing import Optional, List, Dict
from dataclasses import dataclass, field
from functools import cached_property
from lisdf.utils.typing import Vector3f
from lisdf.utils.transformations import euler_from_quaternion
from .base import StringConfigurable
from .control import ControlInfo, JointInfo
from .shape import ShapeInfo
from .visual import VisualInfo


@dataclass
class Pose(StringConfigurable):
    pos: Vector3f
    quat_wxyz: Vector3f

    @classmethod
    def from_quat_xyzw(cls, pos, xyzw):
        return cls(pos, np.array([xyzw[3], xyzw[0], xyzw[1], xyzw[2]]))

    @cached_property
    def quat_xyzw(self):
        return np.array([self.quat_wxyz[1], self.quat_wxyz[2], self.quat_wxyz[3], self.quat_wxyz[0]])

    @cached_property
    def rpy(self):
        return euler_from_quaternion(self.quat_xyzw)

    @classmethod
    def identity(cls):
        return cls(pos=np.zeros(3, dtype='float32'), quat_wxyz=np.array([1, 0, 0, 0], dtype='float32'))


@dataclass
class Inertia(StringConfigurable):
    ixx: float
    ixy: float
    ixz: float
    iyy: float
    iyz: float
    izz: float

    @classmethod
    def from_diagnal(cls, ixx, iyy, izz):
        return cls(ixx, 0, 0, iyy, 0, izz)

    @property
    def matrix(self):
        return np.array([
            [self.ixx, self.ixy, self.ixz],
            [self.ixy, self.iyy, self.iyz],
            [self.ixz, self.iyz, self.izz],
        ], dtype=np.float32)


@dataclass
class Inertial(StringConfigurable):
    mass: float
    pose: Vector3f
    inertia: Inertia


@dataclass
class Geom(object):
    name: str
    pose: Pose
    shape: ShapeInfo
    visual: VisualInfo
    mjcf_configs: Optional[Dict[str, str]] = None

    @property
    def type(self):
        return self.shape.type


@dataclass
class Joint(object):
    name: str
    parent: str
    child: str
    pose: Pose
    joint_info: JointInfo
    control: ControlInfo

    @property
    def type(self):
        return self.joint_info.type

    model: Optional['Model'] = None

    def set_model(self, model):
        self.model = model

    @property
    def parent_link(self):
        return self.model.links[self.parent]

    @property
    def child_link(self):
        return self.model.links[self.child]


@dataclass
class Link(object):
    name: str
    parent: str
    pose: Pose
    inertial: Inertial = None
    geometries: List[Geom] = field(default_factory=list)
    model: Optional['Model'] = None
    # sites: List[Site] = None

    def set_model(self, model):
        self.model = model

    def to_sdf(self):
        fmt = ''
        fmt += f'<link name={self.name}>\n'
        relative_to = '' if self.relative_to is None else f' relative_to="{self.relative_to}"'
        fmt += f'  <pose{relative_to}>{format_pose(self)}</pose>\n'
        for geom in self.geometries:
            fmt += jacinle.indent_text(geom.to_sdf(), 1).rstrip() + '\n'
        fmt += '</link>\n'
        return fmt


@property
class Model(object):
    name: str
    parent: str

    links: Dict[str, Link]
    joints: Dict[str, Joint]


class URDFModel(object):
    name: str
    uri: str
    pose: Pose
