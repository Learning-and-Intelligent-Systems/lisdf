from abc import ABC
from dataclasses import dataclass
from typing import ClassVar, Dict, Optional, Type

from lisdf.components.base import (
    StringConfigurable,
    StringifyContext,
    unsupported_stringify,
)
from lisdf.utils.printing import indent_text
from lisdf.utils.typing import Vector3f

"""A series of data structures for joints."""


@dataclass
class JointDynamics(StringConfigurable):
    damping: float = 0
    friction: float = 0
    armature: float = 0  # used by MJCF only.

    def _to_sdf(self, ctx: StringifyContext) -> str:
        if self.armature != 0:
            ctx.warning(self, "armature is not supported in SDF.")
        return f"""<dynamics>
 <damping>{self.damping}</damping>
 <friction>{self.friction}</friction>
</dynamics>"""

    def _to_urdf(self, ctx: StringifyContext) -> str:
        if self.armature != 0:
            ctx.warning(self, "armature is not supported in SDF.")
        return f'<dynamics damping="{self.damping}" friction="{self.friction}" />'


@dataclass
class JointLimit(StringConfigurable):
    lower: Optional[float] = None
    upper: Optional[float] = None
    effort: Optional[float] = None
    velocity: Optional[float] = None

    def _to_sdf(self, ctx: StringifyContext) -> str:
        fmt = "<limit>\n"
        if self.lower is not None:
            fmt += f"  <lower>{self.lower}</lower>\n"
        if self.upper is not None:
            fmt += f"  <upper>{self.upper}</upper>\n"
        if self.effort is not None:
            fmt += f"  <effort>{self.effort}</effort>\n"
        if self.velocity is not None:
            fmt += f"  <velocity>{self.velocity}</velocity>\n"
        return fmt + "</limit>"

    def _to_urdf(self, ctx: StringifyContext) -> str:
        fmt = "<limit"
        if self.lower is not None:
            fmt += f' lower="{self.lower}"'
        if self.upper is not None:
            fmt += f' upper="{self.upper}"'
        if self.effort is not None:
            fmt += f' effort="{self.effort}"'
        if self.velocity is not None:
            fmt += f' velocity="{self.velocity}"'
        return fmt + " />"


@dataclass
@unsupported_stringify(disable_sdf=True)
class JointCalibration(StringConfigurable):
    falling: float = 0
    rising: float = 0

    def _to_urdf(self, ctx: StringifyContext) -> str:
        return f'<calibration falling="{self.falling}" rising="{self.rising}" />'


@dataclass
@unsupported_stringify(disable_sdf=True)
class JointMimic(StringConfigurable):
    joint: str
    multiplier: float = 1
    offset: float = 0

    # flake8: noqa: E501
    def _to_urdf(self, ctx: StringifyContext) -> str:
        return f'<mimic joint="{self.joint}" multiplier="{self.multiplier}" offset="{self.offset}" />'


@dataclass
@unsupported_stringify(disable_sdf=True)
class JointControlInfo(StringConfigurable):
    lower: Optional[float] = None
    upper: Optional[float] = None
    velocity: Optional[float] = None
    position: Optional[float] = None

    def _to_urdf(self, ctx: StringifyContext) -> str:
        fmt = "<safety_controller"
        if self.lower is not None:
            fmt += f' lower_velocity="{self.lower}"'
        if self.upper is not None:
            fmt += f' upper_velocity="{self.upper}"'
        if self.velocity is not None:
            fmt += f' velocity="{self.velocity}"'
        if self.position is not None:
            fmt += f' position="{self.position}"'
        return fmt + "/>"


@dataclass
class JointInfo(StringConfigurable, ABC):
    """
    When inherit from this class, child classes should pass type="XXX"
    as a keyword argument. This will register a new JointInfo type
    in this class. Then,

    >>> JointInfo.from_type('hinge', axis=np.array([0, 0, 1], dtype='float32'))

    will be equivalent to C.HingeJointInfo(axis=np.array([0, 0, 1], dtype='float32'))
    """

    type: ClassVar[str] = "JointInfo"
    type_mapping: ClassVar[Dict[str, Type["JointInfo"]]] = dict()

    def __init_subclass__(cls, type: str, **kwargs):
        super().__init_subclass__(**kwargs)
        setattr(cls, "type", type)
        JointInfo.type_mapping[type] = cls

    @staticmethod
    def from_type(type, **kwargs) -> "JointInfo":
        return JointInfo.type_mapping[type](**kwargs)


@dataclass
class FixedJointInfo(JointInfo, type="fixed"):
    """A fixed joint does not have any degrees of freedom."""

    def _to_sdf(self, ctx: StringifyContext) -> str:
        return ""

    def _to_urdf(self, ctx: StringifyContext) -> str:
        return ""


@dataclass
class SingleAxisJointInfo(JointInfo, type="controllable"):
    axis: Vector3f
    limit: Optional[JointLimit] = None
    dynamics: Optional[JointDynamics] = None
    calibration: Optional[JointCalibration] = None
    mimic: Optional[JointMimic] = None

    def _to_sdf(self, ctx: StringifyContext) -> str:
        return f"""<axis>
  <xyz>{self.axis[0]} {self.axis[1]} {self.axis[2]}</xyz>
  {indent_text(self.limit.to_sdf(ctx)) if self.limit is not None else ""}
  {indent_text(self.dynamics.to_sdf(ctx)) if self.dynamics is not None else ""}
  {indent_text(self.calibration.to_sdf(ctx))
  if self.calibration is not None else ""}
  {indent_text(self.mimic.to_sdf(ctx)) if self.mimic is not None else ""}
</axis>"""

    def _to_urdf(self, ctx: StringifyContext) -> str:
        return f"""<axis xyz="{self.axis[0]} {self.axis[1]} {self.axis[2]}" />
  {indent_text(self.limit.to_urdf(ctx)) if self.limit is not None else ""}
  {indent_text(self.dynamics.to_urdf(ctx))
  if self.dynamics is not None else ""}
  {indent_text(self.calibration.to_urdf(ctx))
  if self.calibration is not None else ""}
  {indent_text(self.mimic.to_urdf(ctx)) if self.mimic is not None else ""}"""


@dataclass
class ContinuousJointInfo(SingleAxisJointInfo, type="continuous"):
    pass


@dataclass
class RevoluteJointInfo(SingleAxisJointInfo, type="revolute"):
    pass


@dataclass
class PrismaticJointInfo(SingleAxisJointInfo, type="prismatic"):
    pass
