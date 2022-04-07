import numpy as np
from pydrake.trajectories import PiecewisePolynomial

from lisdf.plan_executor.joint_space_path_executor import PathInterpolator


class DrakePiecewisePolynomialInterpolator(PathInterpolator):
    def __init__(self, t_all, confs):
        super().__init__(t_all, confs)

        # Using a FirstOrderHold for now so the code can later be generalized
        # to fancier interpolation schemes (as opposed to just solving the linear eq)
        self._first_order_hold = PiecewisePolynomial.FirstOrderHold(
            self.t_all, self.confs.T  # take transpose as that's what Drake likes
        )

    def value(self, t: float) -> np.ndarray:
        q = self._first_order_hold.value(t)
        q = q.reshape(-1)
        return q
