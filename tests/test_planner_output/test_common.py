import json
from dataclasses import dataclass
from typing import Dict

import yaml

from lisdf.planner_output.common import OutputElement


@dataclass
class _ConcreteOutputElement(OutputElement):
    my_dict: Dict
    validated: bool = False

    def validate(self):
        self.validated = True


def test_output_element():
    element = _ConcreteOutputElement({"lis": "mit", "counter": 999})

    # Check that the element automatically validates itself after init
    assert element.validated

    # Check to_json and to_yaml
    expected_dict_to_dump = {"my_dict": element.my_dict, "validated": True}
    assert element.to_dict() == expected_dict_to_dump
    assert element.to_json() == json.dumps(expected_dict_to_dump)
    assert element.to_yaml() == yaml.safe_dump(expected_dict_to_dump)
