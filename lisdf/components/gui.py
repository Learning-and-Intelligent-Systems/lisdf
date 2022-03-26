from dataclasses import dataclass
from typing import Optional

from lisdf.components.base import Pose, StringConfigurable
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
        name: str,
        projection_type: str = "perspective",
    ) -> "GUICamera":
        pose = Pose.from_lookat(xyz, point_to)
        return cls(name, pose, projection_type)

    def __post_init__(self) -> None:
        assert self.projection_type in ["perspective", "orthographic"]


@dataclass
class GUI(StringConfigurable):
    camera: GUICamera
