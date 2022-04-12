from dataclasses import dataclass

from lisdf.components.base import StringifyContext
from lisdf.components.model import Collision, Model, Visual


@dataclass
class MJCFCollision(Collision):
    inertial_group: int = 3
    contact_type: int = 0
    contact_affinity: int = 0
    contact_dim: int = 3

    def _to_sdf(self, ctx: StringifyContext) -> str:
        ctx.warning(
            self,
            "Contact and inertial features in MJCFCollision is not supported in SDF.",
        )
        return super()._to_sdf(ctx)

    def _to_urdf(self, ctx: StringifyContext) -> str:
        ctx.warning(
            self,
            "Contact and inertial features in MJCFCollision is not supported in URDF.",
        )
        return super()._to_urdf(ctx)


@dataclass
class MJCFVisual(Visual):
    pass


@dataclass
class MJCFModel(Model):
    pass
