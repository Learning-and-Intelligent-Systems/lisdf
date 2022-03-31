from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np

from lisdf.components.base import Pose, StringConfigurable
from lisdf.components.control import JointControlInfo, JointInfo
from lisdf.components.material import RGBA, Material
from lisdf.components.sensor import Sensor
from lisdf.components.shape import ShapeInfo
from lisdf.utils.printing import indent_text
from lisdf.utils.typing import Vector3f, Vector4f


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

    def to_urdf(self) -> str:
        return (
            f'<inertia ixx="{self.ixx}"'
            f' ixy="{self.ixy}" ixz="{self.ixz}"'
            f' iyy="{self.iyy}" iyz="{self.iyz}"'
            f' izz="{self.izz}" />'
        )


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

    def to_urdf(self) -> str:
        return f"""<inertial>
  <mass>{self.mass}</mass>
  {self.pose.to_urdf()}
  {indent_text(self.inertia.to_urdf()).strip()}
</inertial>"""


@dataclass
class SurfaceContact(StringConfigurable):
    pass


@dataclass
class SurfaceFriction(StringConfigurable):
    pass


@dataclass
class SurfaceInfo(StringConfigurable):
    contact: Optional[SurfaceContact] = None
    friction: Optional[SurfaceFriction] = None

    def to_sdf(self) -> str:
        return f"""<surface>
  {indent_text(self.contact.to_sdf()).strip() if self.contact else ""}
  {indent_text(self.friction.to_sdf()).strip() if self.friction else ""}
</surface>"""

    def to_urdf(self) -> str:
        return ""


@dataclass
class _Geom(StringConfigurable):
    """Shared base class for collision and visual."""

    name: str
    pose: Optional[Pose]
    shape: ShapeInfo


@dataclass
class Collision(_Geom):
    surface: Optional[SurfaceInfo] = None

    @property
    def type(self):
        return self.shape.type

    def to_sdf(self) -> str:
        return f"""<collision name="{self.name}">>
  {self.pose.to_sdf() if self.pose is not None else ""}
  <geometry>
    {indent_text(self.shape.to_sdf(), 2).strip()}
  </geometry>
  {indent_text(self.surface.to_sdf()).strip() if self.surface is not None else ""}
</collision>"""

    def to_urdf(self) -> str:
        return f"""<collision name="{self.name}">
  {self.pose.to_urdf() if self.pose is not None else ""}
  <geometry>
    {indent_text(self.shape.to_urdf(), 2).strip()}
  </geometry>
  {indent_text(self.surface.to_urdf()).strip() if self.surface is not None else ""}
</collision>"""


@dataclass
class Visual(_Geom):
    material: Optional[Material] = None

    @property
    def type(self):
        return self.shape.type

    def to_sdf(self) -> str:
        return f"""<visual name="{self.name}">
  {self.pose.to_sdf() if self.pose is not None else ""}
  <geometry>
    {indent_text(self.shape.to_sdf(), 2).strip()}
  </geometry>
  {indent_text(self.material.to_sdf()).strip() if self.material is not None else ""}
</visual>"""

    def to_material_urdf(self) -> str:
        if self.material is not None:
            return f"""<material name="{self.name}_material">
  {indent_text(self.material.to_urdf()).strip() if self.material is not None else ""}
</material>"""
        return ""

    def to_urdf(self) -> str:
        material_str = ""
        if self.material is not None:
            material_str = f"""<material name="{self.name}_material">"""

        return f"""<visual name="{self.name}">
  {self.pose.to_urdf() if self.pose is not None else ""}
  <geometry>
    {indent_text(self.shape.to_urdf(), 2).strip()}
  </geometry>
  {material_str}
</visual>"""


@dataclass
class Link(StringConfigurable):
    name: str
    parent: Optional[str]
    pose: Optional[Pose] = None
    inertial: Optional[Inertial] = None
    collisions: List[Collision] = field(default_factory=list)
    visuals: List[Visual] = field(default_factory=list)
    sensors: List[Sensor] = field(default_factory=list)

    @classmethod
    def from_simple_geom(
        cls,
        name: str,
        pose: Pose,
        shape_type: str,
        rgba: Vector4f,
        inertial: Optional[Inertial] = None,
        **kwargs,
    ):
        if inertial is None:
            inertial = Inertial.zeros()
        shape = ShapeInfo.from_type(shape_type, **kwargs)
        material = RGBA(*rgba)
        return cls(
            name,
            pose=None,
            parent=None,
            inertial=inertial,
            collisions=[Collision(name + "_collision", pose, shape)],
            visuals=[Visual(name + "_visual", pose, shape, material)],
        )

    def to_sdf(self) -> str:
        collision_str = "\n".join([c.to_sdf() for c in self.collisions])
        visual_str = "\n".join([v.to_sdf() for v in self.visuals])
        return f"""<link name="{self.name}">
  {self.pose.to_sdf() if self.pose is not None else ""}
  {indent_text(self.inertial.to_sdf()).strip() if self.inertial is not None else ""}
  {indent_text(collision_str).strip()}
  {indent_text(visual_str).strip()}
</link>
"""

    def to_urdf(self) -> str:
        assert self.pose is None, "URDF does not support link pose."
        material_str = "\n".join([v.to_material_urdf() for v in self.visuals])
        collision_str = "\n".join([c.to_urdf() for c in self.collisions])
        visual_str = "\n".join([v.to_urdf() for v in self.visuals])
        return f"""{material_str}
<link name="{self.name}">
  {indent_text(self.inertial.to_urdf()).strip() if self.inertial is not None else ""}
  {indent_text(collision_str).strip()}
  {indent_text(visual_str).strip()}
</link>"""

    def to_sdf_xml(self) -> str:
        model = Model(self.name)
        model.links.append(self)
        return model.to_sdf_xml()

    def to_urdf_xml(self) -> str:
        model = Model(self.name)
        model.links.append(self)
        return model.to_urdf_xml()


@dataclass
class Joint(StringConfigurable):
    name: str
    parent: str
    child: str
    pose: Pose
    joint_info: JointInfo
    control_info: Optional[JointControlInfo] = None

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
  {indent_text(self.control_info.to_sdf()).strip()
  if self.control_info is not None else ""}
</joint>"""

    def to_urdf(self) -> str:
        return f"""<joint name="{self.name}" type="{self.type}">
  <parent link="{self.parent}"/>
  <child link="{self.child}"/>
  {self.pose.to_urdf()}
  {indent_text(self.joint_info.to_urdf()).strip()}
  {indent_text(self.control_info.to_urdf()).strip()
  if self.control_info is not None else ""}
</joint>"""


@dataclass
class Model(StringConfigurable):
    name: str
    pose: Optional[Pose] = None
    parent: Optional[str] = None
    static: bool = False

    links: List[Link] = field(default_factory=list)
    joints: List[Joint] = field(default_factory=list)

    def to_sdf(self) -> str:
        link_str = "\n".join([link.to_sdf() for link in self.links])
        joint_str = "\n".join([joint.to_sdf() for joint in self.joints])
        return f"""<model name="{self.name}">
  <static>{self.static}</static>
  {self.pose.to_sdf() if self.pose is not None else ""}
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

    def to_urdf(self) -> str:
        link_str = "\n".join([link.to_urdf() for link in self.links])
        joint_str = "\n".join([joint.to_urdf() for joint in self.joints])
        return f"""
<robot name="{self.name}">
  {indent_text(link_str).strip()}
  {indent_text(joint_str).strip()}
</robot>"""

    def to_urdf_xml(self) -> str:
        return f"""<?xml version="1.0"?>
{self.to_urdf()}
</robot>
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

    def to_sdf(self) -> str:
        if self._scale1d is not None:
            scale_str = f"<scale>{self._scale1d}</scale>"
        else:
            scale_str = (
                f"<scale>{self.scale[0]} {self.scale[1]} {self.scale[2]}</scale>"
            )
        return f"""<include name="{self.name}">
  <uri>{self.uri}</uri>
  <static>{self.static}</static>
  {scale_str}
  {self.pose.to_sdf() if self.pose is not None else ""}
</include>"""


@dataclass
class URDFInclude(SDFInclude):
    pass
