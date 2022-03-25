#! /usr/bin/env python3
# -*- coding: utf-8 -*-
# File   : base.py
# Author : Jiayuan Mao
# Email  : maojiayuan@gmail.com
# Date   : 03/23/2022
#
# This file is part of lisdf.
# Distributed under terms of the MIT license.

from abc import ABC


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
