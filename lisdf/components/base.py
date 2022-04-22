from abc import ABC
from collections import defaultdict
from dataclasses import dataclass
from functools import cached_property
from typing import (
    Any,
    Callable,
    ClassVar,
    DefaultDict,
    Dict,
    List,
    Optional,
    Tuple,
    Type,
    TypeVar,
)

import numpy as np

from lisdf.utils.transformations import (
    euler_from_quaternion,
    quaternion_from_euler,
    quaternion_multiply,
)
from lisdf.utils.transformations_more import lookat_rpy
from lisdf.utils.typing import Vector3f, Vector4f, Vector6f

T = TypeVar("T")

NAME_SCOPE_SEP: Optional[str] = "::"


def set_name_scope_sep(sep: Optional[str]) -> None:
    """Set the name scope seperator to None to disable name scoping."""
    global NAME_SCOPE_SEP
    NAME_SCOPE_SEP = sep


class StringifyContext(object):
    def __init__(self, **kwargs) -> None:
        self.stacks: DefaultDict[str, List[Any]] = defaultdict(list)
        self.warnings: List[Tuple[StringConfigurable, str]] = list()
        self.options = kwargs

    def warning(self, obj: "StringConfigurable", message: str) -> None:
        self.warnings.append((obj, message))

    def st_push(self, name: str, value: Any) -> None:
        self.stacks[name].append(value)

    def st_pop(self, name: str, default: Any = None) -> Any:
        return self.stacks[name].pop() if len(self.stacks[name]) > 0 else default

    def st_top(self, name: str, default: Any = None) -> Any:
        return self.stacks[name][-1] if len(self.stacks[name]) > 0 else default

    def get_scoped_name(self, name: str) -> str:
        """
        Prepend the current scope to the name.

        Args:
            name: the input name.

        Returns:
            When the use_scoped_name option is True, the name is prepended with
            the current scope. For example, when we are stringifying a model,
            the to_sdf method will do seomthing like this:
            ```
                ctx.push_scoped_name(self.model_name)
                for link in self.links:
                    link.to_sdf()
                ctx.pop_scoped_name()
            ```

            Then, inside the link.to_sdf method, users can call the function
            get_scoped_name to get the name of the link. It will return:
            `model_name + "::" + link.name`.
        """
        if not self.options["use_scoped_name"]:
            return name

        parent_name = self.st_top("model_name", None)
        if parent_name is None:
            return name
        if NAME_SCOPE_SEP is None:
            return name

        return f"{parent_name}{NAME_SCOPE_SEP}{name}"

    def push_scoped_name(self, name: str) -> None:
        self.st_push("model_name", self.get_scoped_name(name))

    def pop_scoped_name(self) -> None:
        self.st_pop("model_name")

    def get_scoped_pose(self, pose: "Pose") -> "Pose":
        """
        In SDF files we constantly allow nested definition of poses.
        For example, we can specify the pose of a model then all links under
        this model will be transformed. This helper function maintains a stack
        of poses from the root node. The usage is similar to the `get_scoped_name`.
        """
        parent_pose = self.st_top("pose", None)
        if parent_pose is None:
            return pose
        return parent_pose * pose

    def push_scoped_pose(self, pose: "Pose") -> None:
        self.st_push("pose", self.get_scoped_pose(pose))

    def pop_scoped_pose(self) -> None:
        self.st_pop("pose")


class StringConfigurable(ABC):
    # TODO(Jiayuan Mao @ 03/24): implement these methods for the child classes.

    DEFAULT_LISDF_STRINGIFY_OPTIONS: ClassVar[Dict[str, Any]] = {}
    DEFAULT_SDF_STRINGIFY_OPTIONS: ClassVar[Dict[str, Any]] = {}
    DEFAULT_URDF_STRINGIFY_OPTIONS: ClassVar[Dict[str, Any]] = {
        # The URDF standard supports defining the material for a visual element
        # inside the visual element itself. However, this is not supported by
        # some URDF parsers. Set this option to False to enforce all material
        # definitions to be defined at the root level.
        "allow_embedded_material": False,
        # When export the URDF model, whether to use the scoped name (i.e. names
        # that is composed of the parent name and the child name).
        "use_scoped_name": False,
    }

    def to_lisdf(self, ctx: Optional[StringifyContext] = None, **kwargs) -> str:
        if ctx is None:
            for k, v in type(self).DEFAULT_LISDF_STRINGIFY_OPTIONS.items():
                kwargs.setdefault(k, v)
            ctx = StringifyContext(**kwargs)
        return self._to_lisdf(ctx)

    def to_sdf(self, ctx: Optional[StringifyContext] = None, **kwargs) -> str:
        if ctx is None:
            for k, v in type(self).DEFAULT_SDF_STRINGIFY_OPTIONS.items():
                kwargs.setdefault(k, v)
            ctx = StringifyContext(**kwargs)
        return self._to_sdf(ctx)

    def to_urdf(self, ctx: Optional[StringifyContext] = None, **kwargs) -> str:
        if ctx is None:
            for k, v in type(self).DEFAULT_URDF_STRINGIFY_OPTIONS.items():
                kwargs.setdefault(k, v)
            ctx = StringifyContext(**kwargs)
        return self._to_urdf(ctx)

    def _to_lisdf(self, ctx: StringifyContext) -> str:
        return self._to_sdf(ctx)

    def _to_sdf(self, ctx: StringifyContext) -> str:
        raise NotImplementedError()

    def _to_urdf(self, ctx: StringifyContext) -> str:
        raise NotImplementedError()


def unsupported_stringify(
    *, disable_sdf: bool = False, disable_urdf: bool = False
) -> Callable[[Type[T]], Type[T]]:
    # TODO (Jiayuan Mao @ 04/03): find a more checker/IDE-friendly way to inject.
    def decorator(cls: Type[T]) -> Type[T]:
        if not disable_sdf:

            def _to_sdf(self: StringConfigurable, ctx: StringifyContext) -> str:
                ctx.warning(
                    self, "Unsupported SDF stringification for {}".format(cls.__name__)
                )
                return ""

            setattr(cls, "to_sdf", _to_sdf)

        if not disable_urdf:

            def _to_urdf(self: StringConfigurable, ctx: StringifyContext) -> str:
                ctx.warning(
                    self,
                    "Unsupported URDF stringification for {}.".format(cls.__name__),
                )
                return ""

            setattr(cls, "to_urdf", _to_urdf)
        return cls

    return decorator


@dataclass
class Pose(StringConfigurable):
    pos: Vector3f
    quat_wxyz: Vector4f

    @classmethod
    def from_rpy_6d(cls, a: Vector6f) -> "Pose":
        """Construct a Pose object from a 6-dimensional vector: x, y, z, r, p, y."""
        return cls.from_rpy(a[:3], a[3:])

    @classmethod
    def from_rpy(cls, pos: Vector3f, rpy: Vector3f) -> "Pose":
        """Construct a Pose object two 3-dimensional vector."""
        return cls.from_quat_xyzw(pos, quaternion_from_euler(*rpy))  # type: ignore

    @classmethod
    def from_quat_xyzw(cls, pos: Vector3f, xyzw: Vector4f) -> "Pose":
        """Construct a Pose object from a 3d position vector and a
        4d quaternion vector, expressed in xyzw."""
        return cls(pos, np.array([xyzw[3], xyzw[0], xyzw[1], xyzw[2]]))

    @classmethod
    def from_lookat(cls, xyz: Vector3f, point_to: Vector3f) -> "Pose":
        """Construct the roll, pitch, yaw angles of a camera looking at a target.
        This function assumes that the camera is pointing to the z-axis ([0, 0, 1]),
        in the camera frame.

        Args:
            camera_pos: the position of the camera.
            target_pos: the target position.

        Returns:
            a Pose object.
        """
        return cls.from_rpy(xyz, lookat_rpy(xyz, point_to))

    @classmethod
    def identity(cls) -> "Pose":
        """Construct the identity pose (x=y=z=r=p=y=0)."""
        return cls(
            pos=np.zeros(3, dtype="float32"),
            quat_wxyz=np.array([1, 0, 0, 0], dtype="float32"),
        )

    @cached_property
    def quat_xyzw(self) -> Vector4f:
        return np.array(
            [self.quat_wxyz[1], self.quat_wxyz[2], self.quat_wxyz[3], self.quat_wxyz[0]]
        )

    @cached_property
    def rpy(self) -> Vector3f:
        return euler_from_quaternion(self.quat_xyzw)  # type: ignore

    def __mul__(self, other: "Pose") -> "Pose":
        quat = quaternion_multiply(self.quat_xyzw, other.quat_xyzw)  # type: ignore
        return Pose(self.pos + other.pos, quat)

    def _to_sdf(self, ctx: StringifyContext) -> str:
        return (
            f"<pose>{self.pos[0]} {self.pos[1]} {self.pos[2]} "
            f"{self.rpy[0]} {self.rpy[1]} {self.rpy[2]}</pose>"
        )

    def _to_urdf(self, ctx: StringifyContext) -> str:
        urdf_pose = ctx.get_scoped_pose(self)
        return (
            f'<origin xyz="{urdf_pose.pos[0]} {urdf_pose.pos[1]} {urdf_pose.pos[2]}" '
            f'rpy="{urdf_pose.rpy[0]} {urdf_pose.rpy[1]} {urdf_pose.rpy[2]}" />'
        )
