#! /usr/bin/env python3
# -*- coding: utf-8 -*-
# File   : control.py
# Author : Jiayuan Mao
# Email  : maojiayuan@gmail.com
# Date   : 03/23/2022
#
# This file is part of lisdf.
# Distributed under terms of the MIT license.

from dataclasses import dataclass

import numpy as np

from lisdf.utils.typing import Vector2f, Vector3f

from .base import StringConfigurable

__all__ = [
    "ControlInfo",
    "JointInfo",
    "FixedJointInfo",
    "HingeJointInfo",
    "PrismaticJointInfo",
]


@dataclass
class ControlInfo(object):
    limited: bool
    range: Vector2f

    def __init__(self, limited: str, range: Vector2f):
        assert limited in ("true", "false")
        self.limited = limited == "true"
        self.range = range


@dataclass
class JointInfo(StringConfigurable):
    type_mapping = dict()  # type: ignore

    def __init_subclass__(cls, type: str, **kwargs):
        super().__init_subclass__(**kwargs)
        setattr(cls, "type", type)
        JointInfo.type_mapping[type] = cls

    @staticmethod
    def from_type(type, **kwargs) -> "JointInfo":
        return JointInfo.type_mapping[type](**kwargs)


@dataclass
class FixedJointInfo(JointInfo, type="fixed"):
    pass


@dataclass
class ControllableJointInfo(JointInfo, type="controllable"):
    limited: bool
    range: Vector2f
    damping: float
    armatrue: float

    def __init__(self, limited, range, damping, armature):
        assert limited in ("true", "false")
        self.limited = limited == "true"
        self.range = range
        self.damping = damping
        self.armatrue = armature


@dataclass
class HingeJointInfo(ControllableJointInfo, type="hinge"):
    axis: Vector3f

    def __init__(
        self,
        axis,
        limited="false",
        range=np.zeros(2, dtype="float32"),
        damping=0.0,
        armature=0.0,
    ):
        super().__init__(limited, range, damping, armature)
        self.axis = axis


@dataclass
class PrismaticJointInfo(ControllableJointInfo, type="prismatic"):
    axis: Vector3f

    def __init__(
        self,
        axis,
        limited="false",
        range=np.zeros(2, dtype="float32"),
        damping=0.0,
        armature=0.0,
    ):
        super().__init__(limited, range, damping, armature)
        self.axis = axis


@dataclass
class FreeJointInfo(JointInfo, type="free"):
    def __init__(self):
        super().__init__("false", np.zeros(2, dtype="float32"), 0, 0)
