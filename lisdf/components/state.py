from dataclasses import dataclass, field
from typing import List, Optional

from lisdf.components.base import (
    Pose,
    StringConfigurable,
    StringifyContext,
    unsupported_stringify,
)
from lisdf.utils.printing import indent_text


@dataclass
@unsupported_stringify(disable_urdf=True)
class JointAxisState(StringConfigurable):
    axis: int
    value: float

    def _to_sdf(self, ctx: StringifyContext) -> str:
        return f'<angle axis="{self.axis}">{self.value}</angle>'


@dataclass
@unsupported_stringify(disable_urdf=True)
class JointState(StringConfigurable):
    name: str
    axis_states: List[JointAxisState] = field(default_factory=list)

    def _to_sdf(self, ctx: StringifyContext) -> str:
        fmt = f'<joint name="{self.name}">\n'
        for axis_state in self.axis_states:
            fmt += indent_text(axis_state.to_sdf(ctx)) + "\n"
        fmt += "</joint>"
        return fmt


@dataclass
@unsupported_stringify(disable_urdf=True)
class LinkState(StringConfigurable):
    name: str
    pose: Optional[Pose] = None

    def _to_sdf(self, ctx: StringifyContext) -> str:
        fmt = f'<link name="{self.name}">\n'
        if self.pose is not None:
            fmt += indent_text(self.pose.to_sdf(ctx)) + "\n"
        fmt += "</link>"
        return fmt


@dataclass
@unsupported_stringify(disable_urdf=True)
class ModelState(StringConfigurable):
    name: str
    parent: Optional[str] = None
    pose: Optional[Pose] = None
    joint_states: List[JointState] = field(default_factory=list)
    link_states: List[LinkState] = field(default_factory=list)

    def _to_sdf(self, ctx: StringifyContext) -> str:
        fmt = f'<model name="{self.name}">\n'
        if self.pose is not None:
            fmt += indent_text(self.pose.to_sdf(ctx)) + "\n"
        for joint_state in self.joint_states:
            fmt += indent_text(joint_state.to_sdf(ctx)) + "\n"
        for link_state in self.link_states:
            fmt += indent_text(link_state.to_sdf(ctx)) + "\n"
        fmt += "</model>"
        return fmt


@dataclass
@unsupported_stringify(disable_urdf=True)
class WorldState(StringConfigurable):
    name: str
    model_states: List[ModelState] = field(default_factory=list)

    def _to_sdf(self, ctx: StringifyContext) -> str:
        fmt = f'<state world_name="{self.name}">\n'
        for model_state in self.model_states:
            fmt += indent_text(model_state.to_sdf(ctx)) + "\n"
        fmt += "</state>"
        return fmt
