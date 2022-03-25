#! /usr/bin/env python3
# -*- coding: utf-8 -*-
# File   : test_parse_sdf_j.py
# Author : Jiayuan Mao
# Email  : maojiayuan@gmail.com
# Date   : 03/24/2022
#
# This file is part of lisdf.
# Distributed under terms of the MIT license.

"""
The SDF Parser already checks whether all attributes and nodes have been parsed.
So we only need to make sure it runs.
"""

import glob
import os.path as osp


def test_sdf_parsing():
    from lisdf.parsing.sdf_j import load_sdf

    for filename in glob.glob(
        osp.join(osp.dirname(osp.dirname(__file__)), "models", "*.sdf")
    ):
        _ = load_sdf(filename)
