from dataclasses import dataclass, field

import numpy as np

from lisdf.components.base import StringConfigurable
from lisdf.utils.typing import Vector4f


@dataclass(frozen=True)
class VisualInfo(StringConfigurable):
    pass


@dataclass(frozen=True)
class RGBA(VisualInfo):
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


@dataclass(frozen=True)
class PhongMaterial(VisualInfo):
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


@dataclass(frozen=True)
class MJCFMaterial(VisualInfo):
    identifier: str
