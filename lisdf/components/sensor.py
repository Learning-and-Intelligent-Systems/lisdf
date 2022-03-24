#! /usr/bin/env python3
# -*- coding: utf-8 -*-
# File   : sensor.py
# Author : Jiayuan Mao
# Email  : maojiayuan@gmail.com
# Date   : 03/23/2022
#
# This file is part of lisdf.
# Distributed under terms of the MIT license.

from dataclasses import dataclass
from .base import StringConfigurable


@dataclass
class Sensor(StringConfigurable):
    name: str

    type_mapping = dict()

    def __init_subclass__(cls, type, **kwargs):
        super().__init_subclass__(**kwargs)
        cls.type = type
        Sensor.type_mapping[type] = cls

    @staticmethod
    def from_type(type, **kwargs):
        return Sensor.type_mapping[type](**kwargs)


@dataclass
class CameraSensor(Sensor, type='camera'):
    pass

