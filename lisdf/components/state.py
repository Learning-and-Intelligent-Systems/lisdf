from dataclasses import dataclass, field
from typing import List, Optional

from lisdf.components.base import Pose, StringConfigurable


@dataclass
class JointAxisState(StringConfigurable):
    axis: int
    value: float


@dataclass
class JointState(StringConfigurable):
    name: str
    axis_states: List[JointAxisState] = field(default_factory=list)


@dataclass
class LinkState(StringConfigurable):
    name: str
    pose: Optional[Pose] = None


@dataclass
class ModelState(StringConfigurable):
    name: str
    parent: Optional[str] = None
    pose: Optional[Pose] = None
    joint_states: List[JointState] = field(default_factory=list)
    link_states: List[LinkState] = field(default_factory=list)


@dataclass
class WorldState(StringConfigurable):
    name: str
    model_states: List[ModelState] = field(default_factory=list)
