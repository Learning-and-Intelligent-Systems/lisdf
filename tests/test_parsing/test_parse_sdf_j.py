"""
The SDF Parser already checks whether all attributes and nodes have been parsed.
So we only need to make sure it runs.
"""

import os.path as osp

import pytest

from lisdf.parsing.sdf_j import load_sdf


@pytest.mark.parametrize(
    "test_dir",
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
def test_sdf_parsing_j(models_dir, test_dir: str):
    # TODO(Jiayuan Mao @ 03/24): add more assertions.
    filename = osp.join(models_dir, test_dir, "model.sdf")
    _ = load_sdf(
        filename
    )  # the parser already asserts all attributes and nodes are parsed.
