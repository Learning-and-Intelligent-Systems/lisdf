import json
from dataclasses import dataclass
from typing import Dict

import yaml
from mock.mock import mock_open, patch

from lisdf.planner_output.common import OutputElement
from lisdf.planner_output.config import DEFAULT_JSON_INDENT


@dataclass
class _ConcreteOutputElement(OutputElement):
    my_dict: Dict
    validated: bool = False

    def validate(self):
        self.validated = True

    @classmethod
    def from_json_dict(cls, json_dict: Dict) -> "_ConcreteOutputElement":
        return cls(**json_dict)


def test_output_element():
    element = _ConcreteOutputElement({"lis": "mit", "counter": 999})

    # Check that the element automatically validates itself after init
    assert element.validated

    # Check to_dict, to_json, to_yaml
    expected_dict_to_dump = {"my_dict": element.my_dict, "validated": True}
    assert element.to_dict() == expected_dict_to_dump
    assert element.to_json() == json.dumps(
        expected_dict_to_dump, indent=DEFAULT_JSON_INDENT
    )
    assert element.to_yaml() == yaml.safe_dump(expected_dict_to_dump)

    # Check write_json, write_yaml. Mock open so we don't actually write to disk
    with patch("builtins.open", mock_open()) as mock_file:
        element.write_json("/tmp/test.json")
        mock_file.assert_called_with("/tmp/test.json", "w")
        mock_file().write.assert_called_with(
            element.to_json(),
        )

        element.write_yaml("/tmp/test.yaml")
        mock_file.assert_called_with("/tmp/test.yaml", "w")
        mock_file().write.assert_called_with(
            element.to_yaml(),
        )

        assert mock_file.call_count == 4

    # Check from_json_dict, from_json, from_yaml
    assert _ConcreteOutputElement.from_json_dict(element.to_dict()) == element
    assert _ConcreteOutputElement.from_json(element.to_json()) == element
    assert _ConcreteOutputElement.from_yaml(element.to_yaml()) == element
