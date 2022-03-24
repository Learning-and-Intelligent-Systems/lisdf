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
from typing import ClassVar, Dict, Type

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


@dataclass
class JointInfo(StringConfigurable):
    """
    When inherit from this class, child classes should pass type="XXX"
    as a keyword argument. This will register a new JointInfo type
    in this class. Then,

    >>> JointInfo.from_type('hinge', axis=np.array([0, 0, 1], dtype='float32'))

    will be equivalent to C.HingeJointInfo(axis=np.array([0, 0, 1], dtype='float32'))
    """

    type_mapping: ClassVar[Dict[str, Type["JointInfo"]]] = dict()

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
    # NB(Jiayuan Mao @ 03/24): intentionally wrote a explicit constructor.
    # Otherwise inheritance will be a disaster.
    # TODO(Jiayuan Mao @ 03/24): seems that the kw_only feature in Python 3.10 may help.
    # But that's a too new release.
    limited: bool
    range: Vector2f
    damping: float
    armatrue: float

    def __init__(
        self,
        limited: bool = False,
        range: Vector2f = np.zeros(2, dtype="float32"),
        damping: float = 0.0,
        armature: float = 0.0,
    ):
        self.limited = limited
        self.range = range
        self.damping = damping
        self.armatrue = armature


@dataclass
class FreeJointInfo(ControllableJointInfo, type="free"):
    pass


@dataclass
class HingeJointInfo(ControllableJointInfo, type="hinge"):
    axis: Vector3f

    def __init__(
        self,
        axis: Vector3f,
        limited: bool = False,
        range: Vector2f = np.zeros(2, dtype="float32"),
        damping: float = 0.0,
        armature: float = 0.0,
    ):
        super().__init__(limited, range, damping, armature)
        self.axis = axis


@dataclass
class PrismaticJointInfo(ControllableJointInfo, type="prismatic"):
    axis: Vector3f

    def __init__(
        self,
        axis: Vector3f,
        limited: bool = False,
        range: Vector2f = np.zeros(2, dtype="float32"),
        damping: float = 0.0,
        armature: float = 0.0,
    ):
        super().__init__(limited, range, damping, armature)
        self.axis = axis
