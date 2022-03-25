"""
The SDF Parser already checks whether all attributes and nodes have been parsed.
So we only need to make sure it runs.
"""

import os.path as osp

import pytest

from lisdf.parsing.sdf_j import load_sdf


@pytest.mark.parametrize(
    "filename_base",
    [
        "basic_test",
        "collision_test",
        "geometry_test",
        "joint_test",
        "link_test",
        "visual_test",
        "mud_test",
        "m0m_0",
        "m0m_0_test",
    ],
)
def test_sdf_parsing_j(filename_base: str):
    # TODO(Jiayuan Mao @ 03/24): add more assertions.
    filename = osp.join(
        osp.dirname(osp.dirname(osp.dirname(__file__))),
        "models",
        filename_base + "/model.sdf",
    )
    _ = load_sdf(
        filename
    )  # the parser already asserts all attributes and nodes are parsed.
