from dataclasses import dataclass, field
from typing import List, Optional

from lisdf.components.base import (
    StringConfigurable,
    StringifyContext,
    unsupported_stringify,
)
from lisdf.utils.printing import indent_text


@dataclass
@unsupported_stringify(disable_sdf=True, disable_urdf=True)
class JointIdentifier(StringConfigurable):
    name: str

    def _to_lisdf(self, ctx: StringifyContext) -> str:
        return f'<joint name="{self.name}" />'


@dataclass
@unsupported_stringify(disable_sdf=True, disable_urdf=True)
class LinkIdentifier(StringConfigurable):
    name: str

    def _to_lisdf(self, ctx: StringifyContext) -> str:
        return f'<link name="{self.name}" />'


@dataclass
@unsupported_stringify(disable_sdf=True, disable_urdf=True)
class ChainIdentifier(StringConfigurable):
    base_link_name: str
    tip_link_name: str

    def _to_lisdf(self, ctx: StringifyContext) -> str:
        return f'<chain base_link="{self.base_link_name}" tip_link="{self.tip_link_name}" />'  # noqa: E501


@dataclass
@unsupported_stringify(disable_sdf=True, disable_urdf=True)
class GroupIdentifier(StringConfigurable):
    name: str

    def _to_lisdf(self, ctx: StringifyContext) -> str:
        return f'<group name="{self.name}" />'


@dataclass
@unsupported_stringify(disable_sdf=True, disable_urdf=True)
class Group(StringConfigurable):
    name: str
    joints: List[JointIdentifier] = field(default_factory=list)
    links: List[LinkIdentifier] = field(default_factory=list)
    chains: List[ChainIdentifier] = field(default_factory=list)
    sub_groups: List[GroupIdentifier] = field(default_factory=list)

    def _to_lisdf(self, ctx: StringifyContext) -> str:
        fmt = f'<group name="{self.name}">\n'
        for joint in self.joints:
            fmt += indent_text(joint.to_lisdf(ctx), strip=False)
        for link in self.links:
            fmt += indent_text(link.to_lisdf(ctx), strip=False)
        for chain in self.chains:
            fmt += indent_text(chain.to_lisdf(ctx), strip=False)
        for sub_group in self.sub_groups:
            fmt += indent_text(sub_group.to_lisdf(ctx), strip=False)
        fmt += "</group>"
        return fmt


@dataclass
@unsupported_stringify(disable_sdf=True, disable_urdf=True)
class DisableCollisions(StringConfigurable):
    link1_name: str
    link2_name: str
    reason: Optional[str] = None

    def _to_lisdf(self, ctx: StringifyContext) -> str:
        reason_str = f' reason="{self.reason}"' if self.reason else ""
        return f'<disable_collisions link1="{self.link1_name}" link2="{self.link2_name}"{reason_str} />'  # noqa: E501
