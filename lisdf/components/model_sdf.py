from dataclasses import dataclass

from lisdf.components.model import Geom, Link, SurfaceContact, SurfaceFriction


@dataclass
class SDFSurfaceContact(SurfaceContact):
    collide_bitmask: int = 0xFFFF
    collide_without_contact: bool = False


@dataclass
class SDFSurfaceFriction(SurfaceFriction):
    ode_mu: float = 0.0
    ode_mu2: float = 0.0


@dataclass
class SDFGeom(Geom):
    cast_shadows: bool = True


@dataclass
class SDFLink(Link):
    self_collide: bool = True
