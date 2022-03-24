#! /usr/bin/env python3
# -*- coding: utf-8 -*-
# File   : test_parse_mjcf.py
# Author : Jiayuan Mao
# Email  : maojiayuan@gmail.com
# Date   : 03/24/2022
#
# This file is part of lisdf.
# Distributed under terms of the MIT license.

import os.path as osp


def test_sdf_parsing():
    from lisdf.parsing.mjcf import load_mjcf

    for filename in [
        osp.join(
            osp.dirname(osp.dirname(osp.dirname(__file__))),
            "models",
            "mjcf",
            "sawyer_assembly_peg.xml",
        )
    ]:
        _ = load_mjcf(filename)
