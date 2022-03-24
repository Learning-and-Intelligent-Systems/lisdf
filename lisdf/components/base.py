#! /usr/bin/env python3
# -*- coding: utf-8 -*-
# File   : base.py
# Author : Jiayuan Mao
# Email  : maojiayuan@gmail.com
# Date   : 03/23/2022
#
# This file is part of lisdf.
# Distributed under terms of the MIT license.


class StringConfigurable(object):
    def to_sdf(self) -> str:
        raise NotImplementedError()

    def to_urdf(self) -> str:
        raise NotImplementedError()

    def to_mjcf(self) -> str:
        raise NotImplementedError()

    def to_yaml(self) -> str:
        raise NotImplementedError()
