#! /usr/bin/env python3
# -*- coding: utf-8 -*-
# File   : scene.py
# Author : Jiayuan Mao
# Email  : maojiayuan@gmail.com
# Date   : 03/23/2022
#
# This file is part of lisdf.
# Distributed under terms of the MIT license.

from typing import List, Optional

from lisdf.components.model import Model, World


class LISDF:
    SUPPORTED_VERSIONS = {"1.5", "1.6", "1.7", "1.8", "1.9"}

    def __init__(self, sdf_version: str = "1.9"):
        self.sdf_version = ""
        self.set_sdf_version(sdf_version)

        self.model: Optional[Model] = None
        self.worlds: List[World] = list()

    def set_sdf_version(self, version: str) -> None:
        split = version.split(".")
        if len(split) != 2:
            raise ValueError("The version attribute should be in the form 'x.y'")

        if split[0] == "" or split[1] == "":
            raise ValueError("Empty major or minor number is not allowed")

        if int(split[0]) < 0 or int(split[1]) < 0:
            raise ValueError("Version number must be positive")

        if version not in self.SUPPORTED_VERSIONS:
            raise ValueError(
                "Invalid version; only %s is supported"
                % (",".join(self.SUPPORTED_VERSIONS))
            )
        self.sdf_version = version
