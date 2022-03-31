from dataclasses import dataclass
from typing import ClassVar, Dict, Optional, Type

from lisdf.components.base import StringConfigurable
from lisdf.utils.typing import Vector3f


@dataclass
class JointDynamics(object):
    damping: float = 0
    friction: float = 0
    armature: float = 0


@dataclass
class JointLimit(object):
    lower: Optional[float] = None
    upper: Optional[float] = None
    effort: Optional[float] = None
    velocity: Optional[float] = None


@dataclass
class JointCalibration(object):
    falling: float = 0
    rising: float = 0


@dataclass
class JointMimic(object):
    joint: str
    multiplier: float = 1
    offset: float = 0


@dataclass
class JointInfo(StringConfigurable):
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
    pass


@dataclass
class SingleAxisJointInfo(JointInfo, type="controllable"):
    axis: Vector3f
    limit: Optional[JointLimit] = None
    dynamics: Optional[JointDynamics] = None
    calibration: Optional[JointCalibration] = None
    mimic: Optional[JointMimic] = None

    def to_sdf(self) -> str:
        return f"<axis>{self.axis[0]} {self.axis[1]} {self.axis[2]}</axis>"


@dataclass
class ContinuousJointInfo(SingleAxisJointInfo, type="continuous"):
    pass


@dataclass
class RevoluteJointInfo(SingleAxisJointInfo, type="revolute"):
    pass


@dataclass
class PrismaticJointInfo(SingleAxisJointInfo, type="prismatic"):
    pass


@dataclass
class JointControlInfo(object):
    lower: Optional[float] = None
    upper: Optional[float] = None
    velocity: Optional[float] = None
    position: Optional[float] = None
