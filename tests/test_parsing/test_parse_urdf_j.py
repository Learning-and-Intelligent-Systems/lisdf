import os.path as osp

import pytest

from lisdf.parsing.sdf_j import load_sdf_string
from lisdf.parsing.urdf_j import load_urdf, load_urdf_string


@pytest.mark.parametrize(
    "test_dir",
    [
        "179",
        "46236",
        "47578",
        "48721",
    ],
)
def test_sdf_parsing_j(models_dir, test_dir: str):
    # TODO(Jiayuan Mao @ 03/31): add more assertions.
    filename = osp.join(models_dir, "partnet_mobility", test_dir, "mobility.urdf")
    node = load_urdf(
        filename
    )  # the parser already asserts all attributes and nodes are parsed.

    string = node.to_sdf()
    # print(string)
    _ = load_sdf_string(string)

    string = node.to_urdf()
    # print(string)
    _ = load_urdf_string(string)
