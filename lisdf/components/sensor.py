from abc import ABC
from dataclasses import dataclass
from typing import ClassVar, Dict, Type

from lisdf.components.base import (
    StringConfigurable,
    StringifyContext,
    unsupported_stringify,
)


@dataclass
class Sensor(StringConfigurable, ABC):
    name: str

    type: ClassVar[str] = "Sensor"
    type_mapping: ClassVar[Dict[str, Type["Sensor"]]] = dict()

    def __init_subclass__(cls, type: str, **kwargs):
        super().__init_subclass__(**kwargs)
        setattr(cls, "type", type)
        Sensor.type_mapping[type] = cls

    @staticmethod
    def from_type(type, **kwargs) -> "Sensor":
        return Sensor.type_mapping[type](**kwargs)


@dataclass
@unsupported_stringify(disable_urdf=True)
class CameraSensor(Sensor, type="camera"):
    def _to_sdf(self, ctx: StringifyContext) -> str:
        return f'<sensor name="{self.name}" type="{self.type}"></sensor>'
