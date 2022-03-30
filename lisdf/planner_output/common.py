import json
import warnings
from abc import ABC, abstractmethod
from enum import Enum
from typing import Dict

import yaml

from lisdf.planner_output.config import DEFAULT_JSON_INDENT


class _CustomJSONEncoder(json.JSONEncoder):
    """CustomEncoder that will call to_dict for our OutputElements"""

    def default(self, o):
        if isinstance(o, OutputElement):
            return o.to_dict()

        if isinstance(o, Enum):
            return o.value

        return super().default(o)


class OutputElement(ABC):
    def __post_init__(self):
        # IMPORTANT! Validate the object after it has been initialized
        self.validate()

    @abstractmethod
    def validate(self):
        """Validates the objects in this output element"""
        raise NotImplementedError

    def to_dict(self) -> Dict:
        """
        WARNING! DO NOT use dataclasses.asdict, because that will serialize all
        native python objects for us, which we may not want to do as subclasses
        may override to_dict()

        Our CustomJSONEncoder will handle recursively serializing all the
        OutputElements using their respective to_dict() methods.
        """
        return self.__dict__

    @classmethod
    @abstractmethod
    def from_json_dict(cls, json_dict: Dict) -> "OutputElement":
        """Convert a JSON dict with plain Python objects to an OutputElement"""
        raise NotImplementedError

    def to_json(self, indent=DEFAULT_JSON_INDENT, **json_kwargs) -> str:
        """Dump the current object as a JSON string"""
        return json.dumps(
            self.to_dict(), cls=_CustomJSONEncoder, indent=indent, **json_kwargs
        )

    def write_json(
        self, json_fname: str, indent=DEFAULT_JSON_INDENT, **json_kwargs
    ) -> None:
        """Write the current object to a file as a JSON"""
        if not json_fname.endswith(".json"):
            warnings.warn(f"Warning! {json_fname} does not end with .json")

        with open(json_fname, "w") as f:
            f.write(self.to_json(indent, **json_kwargs))

    @classmethod
    def from_json(cls, json_str: str) -> "OutputElement":
        """
        Load an OutputElement from a JSON string.
        There isn't really a need for subclasses to override this method.
        """
        return cls.from_json_dict(json.loads(json_str))

    def to_yaml(self, **yaml_kwargs) -> str:
        """Dump the current object as a YAML string"""
        # Convert JSON to YAML directly so we don't have to write another encoder
        json_as_dict = json.loads(self.to_json())
        return yaml.safe_dump(json_as_dict, **yaml_kwargs)

    def write_yaml(self, yaml_fname: str, **yaml_kwargs) -> None:
        """Write the current object to a file as a YAML"""
        if not yaml_fname.endswith(".yaml"):
            warnings.warn(f"Warning! {yaml_fname} does not end with .yaml")

        with open(yaml_fname, "w") as f:
            f.write(self.to_yaml(**yaml_kwargs))

    @classmethod
    def from_yaml(cls, yaml_str: str) -> "OutputElement":
        """
        Load an OutputElement from a YAML string.
        There isn't really a need for subclasses to override this method.
        """
        # YAML loads a plain old Python object dict, so we can use from_json_dict
        return cls.from_json_dict(yaml.safe_load(yaml_str))
