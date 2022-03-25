import json
from abc import ABC, abstractmethod
from enum import Enum
from typing import Dict

import yaml


class _CustomJSONEncoder(json.JSONEncoder):
    def default(self, o):
        if isinstance(o, OutputElement):
            return o.to_dict()

        if isinstance(o, Enum):
            return o.value

        return super().default(o)


class OutputElement(ABC):
    def __post_init__(self):
        # Validate the object after it has been initialized
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

    def to_json(self, **json_kwargs) -> str:
        return json.dumps(self.to_dict(), cls=_CustomJSONEncoder, **json_kwargs)

    def to_yaml(self, **yaml_kwargs) -> str:
        # Convert JSON to YAML directly so we don't have to write another encoder
        json_as_dict = json.loads(self.to_json())
        return yaml.safe_dump(json_as_dict, **yaml_kwargs)
