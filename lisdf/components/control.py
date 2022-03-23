#! /usr/bin/env python3
# -*- coding: utf-8 -*-
# File   : control.py
# Author : Jiayuan Mao
# Email  : maojiayuan@gmail.com
# Date   : 03/23/2022
#
# This file is part of lisdf.
# Distributed under terms of the MIT license.

import numpy as np
from dataclasses import dataclass
from lisdf.utils.typing import Vector2f, Vector3f
from .base import StringConfigurable

__all__ = ['ControlInfo', 'JointInfo', 'HingeJointInfo', 'PrismaticJointInfo']


@dataclass
class ControlInfo(object):
    limited: bool
    range: Vector2f

    def __init__(self, limited, range):
        assert limited in ('true', 'false')
        self.limited = (limited == 'true')
        self.range = range


@dataclass
class JointInfo(StringConfigurable):
    limited: bool
    range: Vector2f
    damping: float
    armatrue: float

    def __init__(self, limited, range, damping, armature):
        assert limited in ('true', 'false')
        self.limited = (limited == 'true')
        self.range = range
        self.damping = damping
        self.armatrue = armature

    type_mapping = dict()

    def __init_subclass__(cls, type, **kwargs):
        super().__init_subclass__(**kwargs)
        cls.type = type
        JointInfo.type_mapping[type] = cls

    @staticmethod
    def from_type(type, **kwargs):
        return JointInfo.type_mapping[type](**kwargs)


@dataclass
class HingeJointInfo(JointInfo, type='hinge'):
    axis: Vector3f

    def __init__(self, axis, limited, range, damping, armature):
        super().__init__(limited, range, damping, armature)
        self.axis = axis


@dataclass
class PrismaticJointInfo(JointInfo, type='prismatic'):
    axis: Vector3f

    def __init__(self, axis, limited, range, damping, armature):
        super().__init__(limited, range, damping, armature)
        self.axis = axis


@dataclass
class FreeJointInfo(JointInfo, type='free'):
    def __init__(self):
        super().__init__('false', np.zeros(2, dtype='float32'), 0, 0)
