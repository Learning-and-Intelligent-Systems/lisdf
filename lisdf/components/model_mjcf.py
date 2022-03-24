#! /usr/bin/env python3
# -*- coding: utf-8 -*-
# File   : model_mjcf.py
# Author : Jiayuan Mao
# Email  : maojiayuan@gmail.com
# Date   : 03/24/2022
#
# This file is part of lisdf.
# Distributed under terms of the MIT license.

from dataclasses import dataclass
from typing import Optional

from lisdf.components.model import Geom


@dataclass
class MJCFGeom(Geom):
    inertial_group: Optional[int] = 3
    contact_type: Optional[int] = 0
    contact_affinity: Optional[int] = 0
    contact_dim: Optional[int] = 3
