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

import os.path as osp
from typing import Callable

from lisdf.parsing.sdf_j import load_sdf


def gen_test(filename_base: str) -> Callable[[], None]:
    def _test_file():
        filename = osp.join(
            osp.dirname(osp.dirname(osp.dirname(__file__))),
            "models",
            filename_base + ".sdf",
        )
        _ = load_sdf(
            filename
        )  # the parser already asserts all attributes and nodes are parsed.

    return _test_file


test_sdf_parsing_basic = gen_test("basic_test")
test_sdf_parsing_collision = gen_test("collision_test")
test_sdf_parsing_geometry = gen_test("geometry_test")
test_sdf_parsing_joint = gen_test("joint_test")
test_sdf_parsing_link = gen_test("link_test")
test_sdf_parsing_visual = gen_test("visual_test")
test_sdf_parsing_mud = gen_test("mud_test")
test_sdf_parsing_m0m_0 = gen_test("m0m_0")
test_sdf_parsing_m0m_0_test = gen_test("m0m_0_test")
