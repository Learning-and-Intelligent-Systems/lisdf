from typing import Optional, Tuple

import numpy as np

from lisdf.utils.transformations import quaternion_from_euler as _quaternion_from_euler
from lisdf.utils.typing import Vector2f, Vector3f, Vector4f, Vector6f


def safe_float(string: Optional[str]) -> Optional[float]:
    if string is None:
        return None
    return float(string)


def vector2f(string: str) -> Vector2f:
    rv = np.fromstring(string, sep=" ", dtype="float32")
    assert rv.shape == (2,)
    return rv


def vector3f(string: str) -> Vector3f:
    rv = np.fromstring(string, sep=" ", dtype="float32")
    if rv.shape == (1,):
        return np.repeat(rv, 3)
    assert rv.shape == (3,)
    return rv


def vector3f_or_float(string: str) -> Tuple[Optional[float], Vector3f]:
    if string.count(" ") == 0:
        return float(string), vector3f(string)
    return None, vector3f(string)


def wxyz_from_euler(euler: str) -> Vector4f:
    eulerf = vector3f(euler)
    quat = _quaternion_from_euler(*eulerf)  # type: ignore
    return np.array([quat[3], quat[0], quat[1], quat[2]], dtype="float32")


def vector4f(string: str) -> Vector4f:
    rv = np.fromstring(string, sep=" ", dtype="float32")
    assert rv.shape == (4,)
    return rv


def vector6f(string: str) -> Vector6f:
    rv = np.fromstring(string, sep=" ", dtype="float32")
    assert rv.shape == (6,)
    return rv


def bool_string(string: str) -> bool:
    string = string.lower()
    assert string in ("true", "false", "0", "1")
    return string == "true" or string == "1"
