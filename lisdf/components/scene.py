from dataclasses import dataclass, field
from typing import Dict, List, Optional, Union

from lisdf.components.base import NAME_SCOPE_SEP, StringConfigurable, StringifyContext
from lisdf.components.gui import GUI
from lisdf.components.model import Joint, Link, Model, SDFInclude, URDFInclude
from lisdf.components.state import WorldState
from lisdf.utils.printing import indent_text


@dataclass
class World(StringConfigurable):
    name: Optional[str] = None
    static: bool = False
    models: List[Union[Model, SDFInclude, URDFInclude]] = field(default_factory=list)
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

        self.link_dict: Dict[str, Link] = dict()
        self.joint_dict: Dict[str, Joint] = dict()
        self.model_dict: Dict[str, Model] = dict()

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

    def build_lookup_tables(self):
        assert len(self.worlds) <= 1, "Only one world is supported."

        def add_model(model: Model, model_name: Optional[str] = None):
            if model_name is None:
                model_name = model.name
            self.model_dict[model_name] = model

            if NAME_SCOPE_SEP is not None:
                for link in model.links:
                    self.link_dict[model_name + NAME_SCOPE_SEP + link.name] = link
                for joint in model.joints:
                    self.joint_dict[model_name + NAME_SCOPE_SEP + joint.name] = joint
            else:
                for link in model.links:
                    self.link_dict[link.name] = link
                for joint in model.joints:
                    self.joint_dict[joint.name] = joint

        if self.model is not None:
            add_model(self.model)
        else:
            for model in self.worlds[0].models:
                if isinstance(model, URDFInclude):
                    add_model(model.content, model.name)
                elif isinstance(model, SDFInclude):
                    if model.content.model is not None:
                        add_model(model.content.model, model.name)
                    else:
                        for model in model.content.worlds[0].models:
                            add_model(model, model.name)
                elif isinstance(model, Model):
                    add_model(model)

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
