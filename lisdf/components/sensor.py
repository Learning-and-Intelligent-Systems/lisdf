from dataclasses import dataclass
from typing import ClassVar, Dict, Type

from lisdf.components.base import StringConfigurable


@dataclass
class Sensor(StringConfigurable):
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
class CameraSensor(Sensor, type="camera"):
    pass
