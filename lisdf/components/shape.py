"""
This file defines the basic structures for shapes, including built-in shapes and meshes.

TODO(Jiayuan Mao @ 03/23): consider object and material mapping?
"""

from abc import ABC
from dataclasses import dataclass
from typing import ClassVar, Dict, Optional, Type

from lisdf.components.base import StringConfigurable, StringifyContext
from lisdf.utils.typing import Vector3f


@dataclass
class ShapeInfo(StringConfigurable, ABC):
    type: ClassVar[str] = "ShapeInfo"
    type_mapping: ClassVar[Dict[str, Type["ShapeInfo"]]] = dict()

    def __init_subclass__(cls, type: str, **kwargs):
        super().__init_subclass__(**kwargs)
        setattr(cls, "type", type)
        ShapeInfo.type_mapping[type] = cls

    @staticmethod
    def from_type(type, **kwargs) -> "ShapeInfo":
        return ShapeInfo.type_mapping[type](**kwargs)


@dataclass
class BoxShapeInfo(ShapeInfo, type="box"):
    size: Vector3f

    def _to_sdf(self, ctx: StringifyContext) -> str:
        return f"""<box>
  <size>{self.size[0]} {self.size[1]} {self.size[2]}</size>
</box>
"""

    def _to_urdf(self, ctx: StringifyContext) -> str:
        return f'<box size="{self.size[0]} {self.size[1]} {self.size[2]}"/>'


@dataclass
class SphereShapeInfo(ShapeInfo, type="sphere"):
    radius: float

    def _to_sdf(self, ctx: StringifyContext) -> str:
        return f"""<sphere>
  <radius>{self.radius}</radius>
</sphere>
"""

    def _to_urdf(self, ctx: StringifyContext) -> str:
        return f'<sphere radius="{self.radius}"/>'


@dataclass
class CylinderShapeInfo(ShapeInfo, type="cylinder"):
    radius: float
    half_height: float  # follows the mujoco standard.

    @property
    def length(self) -> float:
        return self.half_height * 2

    def _to_sdf(self, ctx: StringifyContext) -> str:
        return f"""<cylinder>
  <radius>{self.radius}</radius>
  <length>{self.length}</length>
</cylinder>
"""

    def _to_urdf(self, ctx: StringifyContext) -> str:
        return f'<cylinder radius="{self.radius}" length="{self.length}"/>'


@dataclass
class CapsuleShapeInfo(ShapeInfo, type="capsule"):
    radius: float
    half_height: float  # follows the mujoco standard.

    @property
    def length(self) -> float:
        return self.half_height * 2

    def _to_sdf(self, ctx: StringifyContext) -> str:
        return f"""<capsule>
  <radius>{self.radius}</radius>
  <length>{self.length}</length>
</capsule>
"""

    def _to_urdf(self, ctx: StringifyContext) -> str:
        return f'<capsule radius="{self.radius}" length="{self.length}"/>'


@dataclass
class MeshShapeInfo(ShapeInfo, type="mesh"):
    filename: str
    size: Vector3f

    def _to_sdf(self, ctx: StringifyContext) -> str:
        return f"""<mesh>
  <uri>{self.filename}</uri>
  <scale>{self.size[0]} {self.size[1]} {self.size[2]}</scale>
</mesh>
"""

    def _to_urdf(self, ctx: StringifyContext) -> str:
        return (
            "<mesh"
            f' filename="{self.filename}"'
            f' scale="{self.size[0]} {self.size[1]} {self.size[2]}"'
            " />"
        )


@dataclass
class PlaneShapeInfo(ShapeInfo, type="plane"):
    half_width: float  # follows the mujoco standard.
    half_height: float
    normal: Optional[Vector3f] = None

    @property
    def width(self) -> float:
        return self.half_width * 2

    @property
    def height(self) -> float:
        return self.half_height * 2

    def _to_sdf(self, ctx: StringifyContext) -> str:
        return f"""<plane>
  <size>{self.width} {self.height}</size>
</plane>
"""

    def _to_urdf(self, ctx: StringifyContext) -> str:
        return f'<plane size="{self.width} {self.height}"/>'
