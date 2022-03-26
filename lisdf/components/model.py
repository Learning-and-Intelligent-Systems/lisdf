from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np

from lisdf.components.base import StringConfigurable, Pose
from lisdf.components.control import ControlInfo, JointInfo
from lisdf.components.sensor import Sensor
from lisdf.components.shape import ShapeInfo
from lisdf.components.visual import VisualInfo
from lisdf.utils.printing import indent_text
from lisdf.utils.typing import Vector3f


@dataclass
class Inertia(StringConfigurable):
    ixx: float
    ixy: float
    ixz: float
    iyy: float
    iyz: float
    izz: float

    @classmethod
    def zeros(cls) -> "Inertia":
        return cls(0, 0, 0, 0, 0, 0)

    @classmethod
    def from_diagonal(cls, ixx, iyy, izz) -> "Inertia":
        return cls(ixx, 0, 0, iyy, 0, izz)

    @property
    def matrix(self) -> np.ndarray:
        return np.array(
            [
                [self.ixx, self.ixy, self.ixz],
                [self.ixy, self.iyy, self.iyz],
                [self.ixz, self.iyz, self.izz],
            ],
            dtype=np.float32,
        )

    def to_sdf(self) -> str:
        return f"""<inertia>
  <ixx>{self.ixx}</ixx>
  <ixy>{self.ixy}</ixy>
  <ixz>{self.ixz}</ixz>
  <iyy>{self.iyy}</iyy>
  <iyz>{self.iyz}</iyz>
  <izz>{self.izz}</izz>
</inertia>"""


@dataclass
class Inertial(StringConfigurable):
    mass: float
    pose: Pose
    inertia: Inertia

    @classmethod
    def zeros(cls) -> "Inertial":
        return cls(0, Pose.identity(), Inertia.zeros())

    def to_sdf(self) -> str:
        return f"""<inertial>
  <mass>{self.mass}</mass>
  {self.pose.to_sdf()}
  {indent_text(self.inertia.to_sdf()).strip()}
</inertial>"""


@dataclass
class SurfaceContact(StringConfigurable):
    pass


@dataclass
class SurfaceFriction(StringConfigurable):
    pass


@dataclass
class Surface(StringConfigurable):
    contact: Optional[SurfaceContact] = None
    friction: Optional[SurfaceFriction] = None


@dataclass
class Geom(StringConfigurable):
    name: str
    pose: Optional[Pose]
    shape: ShapeInfo
    visual: Optional[VisualInfo] = None
    surface: Optional[Surface] = None

    @property
    def type(self):
        return self.shape.type

    def to_sdf(self) -> str:
        return f"""<geometry>
  {self.pose.to_sdf() if self.pose is not None else ""}
  {indent_text(self.shape.to_sdf()).strip()}
</geometry>"""


@dataclass
class Joint(StringConfigurable):
    name: str
    parent: str
    child: str
    pose: Pose
    joint_info: JointInfo
    control_info: Optional[ControlInfo] = None

    @property
    def type(self):
        return self.joint_info.type

    # TODO(Jiayuan Mao @ 03/24): add a link to the corresponding `model`,
    # so we can use joint.parent_link to access the corresponding link
    # object.

    def to_sdf(self) -> str:
        return f"""<joint name="{self.name}" type="{self.type}">
  <parent>{self.parent}</parent>
  <child>{self.child}</child>
  {self.pose.to_sdf()}
  {indent_text(self.joint_info.to_sdf()).strip()}
</joint>"""


@dataclass
class Link(StringConfigurable):
    name: str
    parent: Optional[str]
    pose: Pose
    inertial: Optional[Inertial] = None
    collisions: List[Geom] = field(default_factory=list)
    visuals: List[Geom] = field(default_factory=list)
    sensors: List[Sensor] = field(default_factory=list)

    def to_sdf(self) -> str:
        if len(self.collisions) > 0:
            collision_str = "\n".join([c.to_sdf() for c in self.collisions])
            collision_str = f"""<collision>
  {collision_str}
</collision>"""
        else:
            collision_str = ""
        if len(self.visuals) > 0:
            visual_str = "\n".join([v.to_sdf() for v in self.visuals])
            visual_str = f"""<visual>
  {visual_str}
</visual>"""
        else:
            visual_str = ""
        return f"""<link name="{self.name}">
  {self.pose.to_sdf()}
  {indent_text(self.inertial.to_sdf()).strip() if self.inertial is not None else ""}
  {indent_text(collision_str).strip()}
  {indent_text(visual_str).strip()}
</link>
"""


@dataclass
class Model(StringConfigurable):
    name: str
    pose: Pose
    parent: Optional[str] = None
    static: bool = False

    links: List[Link] = field(default_factory=list)
    joints: List[Joint] = field(default_factory=list)

    def to_sdf(self) -> str:
        link_str = "\n".join([link.to_sdf() for link in self.links])
        joint_str = "\n".join([joint.to_sdf() for joint in self.joints])
        return f"""<model name="{self.name}">
  <static>{self.static}</static>
  {self.pose.to_sdf()}
  {indent_text(link_str).strip()}
  {indent_text(joint_str).strip()}
</model>
"""

    def to_sdf_xml(self) -> str:
        return f"""<?xml version="1.0"?>
<sdf version="1.9">
{self.to_sdf()}
</sdf>
"""


@dataclass
class SDFInclude(StringConfigurable):
    name: Optional[str]
    uri: str
    scale: Vector3f
    pose: Optional[Pose]
    parent: Optional[str] = None
    static: bool = False

    _scale1d: Optional[float] = None
    _content: Optional[StringConfigurable] = None

    @property
    def scale_1d(self):
        assert self._scale1d is not None, "scale_1d is not allowed. Use scale instead."
        return self._scale1d

    @property
    def content(self):
        assert self._content is not None, "content has not been parsed."
        return self._content


@dataclass
class URDFInclude(StringConfigurable):
    name: Optional[str]
    uri: str
    scale: Vector3f
    pose: Pose
    parent: Optional[str] = None
    static: bool = False

    _scale1d: Optional[float] = None
    _content: Optional[StringConfigurable] = None

    @property
    def scale_1d(self):
        assert self._scale1d is not None, "scale_1d is not allowed. Use scale instead."
        return self._scale1d

    @property
    def content(self):
        assert self._content is not None, "content has not been parsed."
        return self._content
