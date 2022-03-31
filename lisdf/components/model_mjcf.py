from dataclasses import dataclass

from lisdf.components.model import Geom, Model


@dataclass
class MJCFGeom(Geom):
    inertial_group: int = 3
    contact_type: int = 0
    contact_affinity: int = 0
    contact_dim: int = 3


@dataclass
class MJCFModel(Model):
    pass
