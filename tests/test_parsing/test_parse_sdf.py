import os
from collections import Counter
from typing import List

from lisdf.parsing.parse_sdf import load_sdf
from lisdf.parsing.sdf import Model
from lisdf.parsing.urdf import Robot


def _assert_in_any_order(l_1: List, l_2: List):
    """Assert two lists contain the same elements in any order"""
    # We can't use sort as can't expect elements to be sorted.
    assert Counter(l_1) == Counter(l_2)


def test_recursive_sdf_parsing(models_dir):
    parsed_sdf = load_sdf("mud_test/model.sdf", models_dir)
    world = parsed_sdf.aggregate_order[0]
    assert len(world.includes) == 5
    assert len(world.models) == 7


def test_m0m(models_dir):
    parsed_sdf = load_sdf("m0m/model_no_pr2.sdf", models_dir)
    world = parsed_sdf.aggregate_order[0]
    assert world.models[4].links[0].collisions[0].geometry.uri == os.path.join(
        models_dir, "m0m/../ycb/011_banana/textured.obj"
    )
    assert world.models[4].links[0].visuals[0].geometry.uri == os.path.join(
        models_dir, "m0m/../ycb/011_banana/textured.obj"
    )
    assert len(world.includes) == 0
    assert len(world.models) == 6
    _assert_in_any_order([type(m) for m in world.models], [Model for _ in range(6)])


def test_partnet_urdf(models_dir):
    parsed_sdf = load_sdf("partnet_mobility_test/model.sdf", models_dir)
    world = parsed_sdf.aggregate_order[0]

    assert len(world.includes) == 1
    assert len(world.models) == 3
    _assert_in_any_order(
        [type(m) for m in world.models],
        [
            Model,
            Model,
            Robot,
        ],
    )
