from lisdf.parse_sdf import load_sdf


def test_recursive():
    parsed_sdf = load_sdf("mud_test.sdf")
    world = parsed_sdf.aggregate_order[0]
    assert len(world.includes) == 5
    assert len(world.models) == 7
