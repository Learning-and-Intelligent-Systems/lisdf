from typing import Dict, List, Set, Tuple, Union

import pddlpy

Predicate = str
Type = str
Object = str

# First element in an atom is the predicate name, the rest are argument objects
Atom = List[Union[Predicate, Object]]
State = List[Atom]

def parse_pddl(
    domain_file_path: str, problem_file_path: str
) -> Tuple[State, State, Dict[Type, Set[Object]]]:
    """
    Parses the domain and problem PDDL files provided string paths
    to each of these.

    :return a tuple of the following:
        - init_atoms_strs: a list of list of strings representing
          the init atoms, where each inner list represents
          a ground atom. The first element of the inner list is the
          name of the predicate, and the following elements correspond
          to object names that are used to ground the predicate
        - goal_atoms_strs: same as above, but for the goal atoms
        - type_to_object_dict: a dictionary mapping a string representing
          each type to a set of strings representing the objects of that
          type
    """
    dom_prob = pddlpy.DomainProblem(domain_file_path, problem_file_path)
    init_atoms = dom_prob.initialstate()
    init_atoms_strs = [atom.predicate for atom in init_atoms]
    goal_atoms = dom_prob.goals()
    goal_atoms_strs = [atom.predicate for atom in goal_atoms]
    obj_to_type_dict = dom_prob.worldobjects()
    type_to_object_dict: Dict[str, Set[str]] = {}
    for obj in obj_to_type_dict.keys():
        if type_to_object_dict.get(obj_to_type_dict[obj]) is None:
            type_to_object_dict[obj_to_type_dict[obj]] = set([obj])
        else:
            type_to_object_dict[obj_to_type_dict[obj]].add(obj)

    return init_atoms_strs, goal_atoms_strs, type_to_object_dict


if __name__ == "__main__":
    entry_names = ("init_atoms_strs", "goal_atoms_strs", "type_to_object_dict")
    for entry_name, entry in zip(
        entry_names,
        parse_pddl("examples/pddl/clutter.pddl", "examples/pddl/clutter_prob01.pddl"),
    ):
        print(entry_name)
        print(entry)
