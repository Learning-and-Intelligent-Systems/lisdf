from dataclasses import dataclass

from lisdf.components.base import StringifyContext
from lisdf.components.model import Link, SurfaceContact, SurfaceFriction, Visual


@dataclass
class SDFSurfaceContact(SurfaceContact):
    collide_bitmask: int = 0xFFFF
    collide_without_contact: bool = False

    def _to_sdf(self, ctx: StringifyContext) -> str:
        return f"""<contact>
  <collide_bitmask>{self.collide_bitmask}</collide_bitmask>
  <collide_without_contact>{self.collide_without_contact}</collide_without_contact>
</contact>"""

    def _to_urdf(self, ctx: StringifyContext) -> str:
        ctx.warning(self, "SDFSurfaceContact is not supported in URDF.")
        return ""


@dataclass
class SDFSurfaceFriction(SurfaceFriction):
    ode_mu: float = 0.0
    ode_mu2: float = 0.0

    def _to_sdf(self, ctx: StringifyContext) -> str:
        return f"""<friction>
  <ode>
    <mu>{self.ode_mu}</mu>
    <mu2>{self.ode_mu2}</mu2>
  </ode>
</friction>"""

    def _to_urdf(self, ctx: StringifyContext) -> str:
        ctx.warning(self, "SDFSurfaceContact is not supported in URDF.")
        return ""


@dataclass
class SDFVisual(Visual):
    cast_shadows: bool = True

    def _to_sdf(self, ctx: StringifyContext) -> str:
        fmt = super()._to_sdf(ctx).split("\n")
        fmt.insert(-1, f"  <cast_shadows>{self.cast_shadows}</cast_shadows>")
        return "\n".join(fmt)

    def _to_urdf(self, ctx: StringifyContext) -> str:
        if not self.cast_shadows:
            ctx.warning(self, "Visual::cast_shadows is not supported in URDF")
        return super()._to_urdf(ctx)


@dataclass
class SDFLink(Link):
    self_collide: bool = True

    def _to_sdf(self, ctx: StringifyContext) -> str:
        fmt = super()._to_sdf(ctx).split("\n")
        fmt.insert(-1, f"  <self_collide>{self.self_collide}</self_collide>")
        return "\n".join(fmt)

    def _to_urdf(self, ctx: StringifyContext) -> str:
        if not self.self_collide:
            ctx.warning(self, "Visual::self_collide is not supported in URDF")
        return super()._to_urdf(ctx)
