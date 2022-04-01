from lisdf.parsing.parse_pddl import parse_pddl


def test_parse_pddl():
    init_atoms, goal_atoms, type_to_obj_dict = parse_pddl(
        "examples/pddl/clutter.pddl", "examples/pddl/clutter_prob01.pddl"
    )
    for atom in init_atoms:
        # Test that the 0th element of the atoms, which should represent
        # the predicate names, is in the list of known predicate names
        predicate_names = ["on-surface", "arm-empty", "holding", "on"]
        assert atom[0] in predicate_names
        assert not any(elem in predicate_names for elem in atom[1:])

    # Test that a particular expected goal is in the goal_atoms
    assert ["on-surface", "gs3", "table4"] in goal_atoms

    # Test that the type_to_obj_dict has the expected number of objects
    # of a particular type
    assert len(type_to_obj_dict["surface"]) == 4
