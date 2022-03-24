#! /usr/bin/env python3
# -*- coding: utf-8 -*-
# File   : string_utils.py
# Author : Jiayuan Mao
# Email  : maojiayuan@gmail.com
# Date   : 03/23/2022
#
# This file is part of lisdf.
# Distributed under terms of the MIT license.

import numpy as np

from lisdf.utils.transformations import quaternion_from_euler as _quaternion_from_euler
from lisdf.utils.typing import Vector2f, Vector3f, Vector4f, Vector6f


def vector2f(string: str) -> Vector2f:
    rv = np.fromstring(string, sep=" ", dtype="float32")
    assert rv.shape == (2,)
    return rv


def vector3f(string: str) -> Vector3f:
    rv = np.fromstring(string, sep=" ", dtype="float32")
    if rv.shape == (1,):
        return np.repeat(rv, 3)
    assert rv.shape == (3,)
    return rv


def wxyz_from_euler(euler: str) -> Vector4f:
    eulerf = vector3f(euler)
    quat = _quaternion_from_euler(*eulerf)  # type: ignore
    return np.array([quat[3], quat[0], quat[1], quat[2]], dtype="float32")


def vector4f(string: str) -> Vector4f:
    rv = np.fromstring(string, sep=" ", dtype="float32")
    assert rv.shape == (4,)
    return rv


def vector6f(string: str) -> Vector6f:
    rv = np.fromstring(string, sep=" ", dtype="float32")
    assert rv.shape == (6,)
    return rv


def bool_string(string: str) -> bool:
    string = string.lower()
    assert string in ("true", "false", "0", "1")
    return string == "true" or string == "1"
