"""
The SDF Parser already checks whether all attributes and nodes have been parsed.
So we only need to make sure it runs.
"""

import os.path as osp

import pytest

from lisdf.parsing.sdf_j import load_sdf, load_sdf_string
from lisdf.parsing.urdf_j import load_urdf_string


@pytest.mark.parametrize(
    "test_dir",
    [
        "basic_test",
        "collision_test",
        "geometry_test",
        "joint_test",
        "link_test",
        "visual_test",
        "m0m_0",
        "m0m_0_test",
    ],
)
def test_sdf_parsing_j(models_dir, test_dir: str):
    # TODO(Jiayuan Mao @ 03/24): add more assertions.
    filename = osp.join(models_dir, test_dir, "model.sdf")
    node = load_sdf(
        filename
    )  # the parser already asserts all attributes and nodes are parsed.

    string = node.to_sdf()
    _ = load_sdf_string(string)

    if "m0m" not in test_dir:  # m0m test contains more than one objects.
        string = node.to_urdf()
        # print(string)
        _ = load_urdf_string(string)
