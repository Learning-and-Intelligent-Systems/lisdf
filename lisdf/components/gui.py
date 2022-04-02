from dataclasses import dataclass
from typing import Optional

from lisdf.components.base import Pose, StringConfigurable, StringifyContext
from lisdf.utils.printing import indent_text
from lisdf.utils.typing import Vector3f


@dataclass
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
        fmt = f"""<camera{name_str}>"""
        fmt += f"""<pose>{self.pose.to_sdf(ctx)}</pose>"""
        fmt += f"""<projection_type>{self.projection_type}</projection_type>"""
        fmt += """</camera>"""
        return fmt

    def _to_urdf(self, ctx: StringifyContext) -> str:
        ctx.warning(self, "GUI Camera is not supported in URDF.")
        return ""


@dataclass
class GUI(StringConfigurable):
    camera: GUICamera

    def _to_sdf(self, ctx: StringifyContext) -> str:
        return f"""<gui>
  {indent_text(self.camera.to_sdf(ctx)).strip()}
</gui>"""

    def _to_urdf(self, ctx: StringifyContext) -> str:
        ctx.warning(self, "GUI is not supported in URDF.")
        return ""
