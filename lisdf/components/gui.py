from dataclasses import dataclass
from typing import Optional

from lisdf.components.base import (
    Pose,
    StringConfigurable,
    StringifyContext,
    unsupported_stringify,
)
from lisdf.utils.printing import indent_text
from lisdf.utils.typing import Vector3f


@dataclass
@unsupported_stringify(disable_urdf=True)
class GUICamera(StringConfigurable):
    name: Optional[str]
    pose: Pose
    projection_type: str = "perspective"

    # TODO(Jiayuan Mao @ 03/26): Add track_visual.

    @classmethod
    def from_lookat(
        cls,
        xyz: Vector3f,
        point_to: Vector3f,
        name: str = "camera",
        projection_type: str = "perspective",
    ) -> "GUICamera":
        pose = Pose.from_lookat(xyz, point_to)
        return cls(name, pose, projection_type)

    def __post_init__(self) -> None:
        assert self.projection_type in ["perspective", "orthographic"]

    def _to_sdf(self, ctx: StringifyContext) -> str:
        name_str = f' name="{self.name}"' if self.name else ""
        return f"""<camera{name_str}>
  <pose>{self.pose.to_sdf(ctx)}</pose>
  <projection_type>{self.projection_type}</projection_type>
</camera>"""


@dataclass
@unsupported_stringify(disable_urdf=True)
class GUI(StringConfigurable):
    camera: GUICamera

    def _to_sdf(self, ctx: StringifyContext) -> str:
        return f"""<gui>
  {indent_text(self.camera.to_sdf(ctx))}
</gui>"""
