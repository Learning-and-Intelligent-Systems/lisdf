from abc import ABC
from dataclasses import dataclass, field

import numpy as np

from lisdf.components.base import (
    StringConfigurable,
    StringifyContext,
    unsupported_stringify,
)
from lisdf.utils.typing import Vector4f


@dataclass(frozen=True)
class Material(StringConfigurable, ABC):
    pass


@dataclass(frozen=True)
class RGBA(Material):
    r: float
    g: float
    b: float
    a: float

    @classmethod
    def from_numpy(cls, a: Vector4f) -> "RGBA":
        if a.shape == (3,):
            return cls(a[0], a[1], a[2], 1)
        assert a.shape == (4,)
        return cls(a[0], a[1], a[2], a[3])

    def _to_sdf(self, ctx: StringifyContext) -> str:
        return f"""<material>
  <ambient>{self.r:.3f} {self.g:.3f} {self.b:.3f} {self.a:.3f}</ambient>
  <diffuse>{self.r:.3f} {self.g:.3f} {self.b:.3f} {self.a:.3f}</diffuse>
</material>"""

    def _to_urdf(self, ctx: StringifyContext) -> str:
        return f'<color rgba="{self.r:.3f} {self.g:.3f} {self.b:.3f} {self.a:.3f}" />'


@dataclass(frozen=True)
class PhongMaterial(Material):
    ambient: Vector4f = field(
        default_factory=lambda: np.array([1, 1, 1, 1], dtype="float32")
    )
    diffuse: Vector4f = field(
        default_factory=lambda: np.array([1, 1, 1, 1], dtype="float32")
    )
    specular: Vector4f = field(
        default_factory=lambda: np.array([0, 0, 0, 1], dtype="float32")
    )
    emissive: Vector4f = field(
        default_factory=lambda: np.array([0, 0, 0, 1], dtype="float32")
    )

    # flake8: noqa: E501
    def _to_sdf(self, ctx: StringifyContext) -> str:
        return f"""<material>
  <ambient>{self.ambient[0]} {self.ambient[1]} {self.ambient[2]} {self.ambient[3]}</ambient>
  <diffuse>{self.diffuse[0]} {self.diffuse[1]} {self.diffuse[2]} {self.diffuse[3]}</diffuse>
  <specular>{self.specular[0]} {self.specular[1]} {self.specular[2]} {self.specular[3]}</specular>
  <emissive>{self.emissive[0]} {self.emissive[1]} {self.emissive[2]} {self.emissive[3]}</emissive>
</material>"""

    # flake8: noqa: E501
    def _to_urdf(self, ctx: StringifyContext) -> str:
        ctx.warning(self, "PhongMaterial is not supported in URDF.")
        return f'<color rgba="{self.ambient[0]} {self.ambient[1]} {self.ambient[2]} {self.ambient[3]}" />'


@dataclass(frozen=True)
@unsupported_stringify(disable_sdf=True)
class Texture(Material):
    filename: str

    def _to_urdf(self, ctx: StringifyContext) -> str:
        return f'<texture filename="{self.filename}" />'


@dataclass(frozen=True)
@unsupported_stringify(disable_sdf=True, disable_urdf=True)
class MJCFMaterial(Material):
    identifier: str  # TODO

    def _to_sdf(self, ctx: StringifyContext) -> str:
        ctx.warning(self, "MJCF material is not supported in SDF.")
        return ""

    def _to_urdf(self, ctx: StringifyContext) -> str:
        ctx.warning(self, "MJCF material is not supported in URDF.")
        return ""
