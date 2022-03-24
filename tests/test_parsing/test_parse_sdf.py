from lisdf.parsing.parse_sdf import load_sdf


def test_recursive_sdf_parsing():
    parsed_sdf = load_sdf("mud_test/model.sdf")
    world = parsed_sdf.aggregate_order[0]
    assert len(world.includes) == 5
    assert len(world.models) == 7


def test_m0m():
    parsed_sdf = load_sdf("m0m/model.sdf")
    world = parsed_sdf.aggregate_order[0]
    assert (
        world.models[4].links[0].collisions[0].geometry.uri
        == "models/m0m/../ycb/011_banana/textured.obj"
    )
    assert (
        world.models[4].links[0].visuals[0].geometry.uri
        == "models/m0m/../ycb/011_banana/textured.obj"
    )
    assert len(world.includes) == 0
    assert len(world.models) == 6
