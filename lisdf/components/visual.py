#! /usr/bin/env python3
# -*- coding: utf-8 -*-
# File   : visual.py
# Author : Jiayuan Mao
# Email  : maojiayuan@gmail.com
# Date   : 03/23/2022
#
# This file is part of lisdf.
# Distributed under terms of the MIT license.

from dataclasses import dataclass

import numpy as np

from lisdf.utils.typing import Vector4f

from .base import StringConfigurable

__all__ = ["VisualInfo", "RGBA", "PhongMaterial", "MJCFMaterial"]


@dataclass
class VisualInfo(StringConfigurable):
    pass


@dataclass
class RGBA(VisualInfo):
    r: float
    g: float
    b: float
    a: float

    @classmethod
    def from_numpy(cls, a):
        if a.shape == (3,):
            return cls(a[0], a[1], a[2], 1)
        assert a.shape == (4,)
        return cls(a[0], a[1], a[2], a[3])


@dataclass
class PhongMaterial(VisualInfo):
    ambient: Vector4f
    diffuse: Vector4f
    specular: Vector4f
    emissive: Vector4f

    @classmethod
    def default(cls):
        return cls(
            np.fromstring("1 1 1 1", sep=" ", dtype="float32"),
            np.fromstring("1 1 1 1", sep=" ", dtype="float32"),
            np.fromstring("0 0 0 1", sep=" ", dtype="float32"),
            np.fromstring("0 0 0 1", sep=" ", dtype="float32"),
        )


@dataclass
class MJCFMaterial(VisualInfo):
    identifier: str
