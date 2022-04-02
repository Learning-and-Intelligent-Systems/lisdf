from dataclasses import dataclass, field
from typing import List, Optional

from lisdf.components.base import StringConfigurable, StringifyContext
from lisdf.components.gui import GUI
from lisdf.components.model import Model
from lisdf.components.state import WorldState
from lisdf.utils.printing import indent_text


@dataclass
class World(StringConfigurable):
    name: Optional[str] = None
    static: bool = False
    models: List[Model] = field(default_factory=list)
    states: List[WorldState] = field(default_factory=list)
    gui: Optional[GUI] = None

    def _to_sdf(self, ctx: StringifyContext) -> str:
        name_str = f' name="{self.name}"' if self.name is not None else ""
        fmt = ""
        fmt += f"<world{name_str}>\n"
        fmt += f"  <static>{self.static}</static>\n"
        for model in self.models:
            fmt += f"  {indent_text(model.to_sdf(ctx)).strip()}\n"
        for state in self.states:
            fmt += f"  {indent_text(state.to_sdf(ctx)).strip()}\n"
        if self.gui is not None:
            fmt += f"  {indent_text(self.gui.to_sdf(ctx)).strip()}\n"
        fmt += "</world>\n"
        return fmt

    def _to_urdf(self, ctx: StringifyContext) -> str:
        assert len(self.models) == 1, "URDF only supports one model in a world."
        return self.models[0].to_urdf(ctx)


class LISDF(StringConfigurable):
    SUPPORTED_VERSIONS = {"1.5", "1.6", "1.7", "1.8", "1.9"}

    def __init__(self, sdf_version: str = "1.9"):
        self.sdf_version = ""
        self.set_sdf_version(sdf_version)

        self.model: Optional[Model] = None
        self.worlds: List[World] = list()

    def set_sdf_version(self, version: str) -> None:
        split = version.split(".")

        if len(split) != 2:
            raise ValueError("The version attribute should be in the form 'x.y'")

        if split[0] == "" or split[1] == "":
            raise ValueError("Empty major or minor number is not allowed")

        if int(split[0]) < 0 or int(split[1]) < 0:
            raise ValueError("Version number must be positive")

        if version not in self.SUPPORTED_VERSIONS:
            raise ValueError(
                "Invalid version; only %s is supported"
                % (",".join(self.SUPPORTED_VERSIONS))
            )
        self.sdf_version = version

    def _to_sdf(self, ctx: StringifyContext) -> str:
        fmt = '<?xml version="1.0" ?>\n'
        fmt += f'<sdf version="{self.sdf_version}">\n'
        if self.model is not None:
            fmt += f"  {indent_text(self.model.to_sdf(ctx)).strip()}\n"
        for world in self.worlds:
            fmt += f"  {indent_text(world.to_sdf(ctx)).strip()}\n"
        fmt += "</sdf>\n"
        return fmt

    def _to_urdf(self, ctx: StringifyContext) -> str:
        assert (
            len(self.worlds) == 0 and self.model is not None
        ), "URDF only supports one model in a definition file."
        return self.model.to_urdf(ctx)
