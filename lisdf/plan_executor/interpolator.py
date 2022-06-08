from abc import ABC, abstractmethod

import numpy as np


class PathInterpolator(ABC):
    """Used so we can abstract simulator's interpolator out of the executor."""

    def __init__(self, t_all: np.ndarray, confs: np.ndarray):
        """
        Let n be the number of configurations and q be the number of joints
        in the configuration.

        Parameters
        ----------
        t_all: 1-d array of shape (n, ) with the times for each configuration point.
            This array must be sorted.
        confs: 2-d array of shape (n, q) where each row represents a robot conf.
            Subsequently, there is a conf for each time in t_all.
        """
        if t_all.shape[0] != confs.shape[0]:
            raise ValueError("t_all and confs must be the same length.")
        elif not np.allclose(np.sort(t_all), t_all):
            raise ValueError("t_all must be sorted")

        self.t_all = t_all
        self.confs = confs

    @abstractmethod
    def value(self, t: float) -> np.ndarray:
        """
        Get the interpolated joint values for the given time t.
        Returns a 1-d array shape (q, ) where q is the number of joints.
        """
        raise NotImplementedError


class NearestTimeInterpolator(PathInterpolator):
    def value(self, t: float) -> np.ndarray:
        # Find the index of the value with closest time to t.
        # Could improve to O(log n) but less readable.
        shifted_t_all = np.abs(self.t_all - t)
        closest_t_idx = np.argmin(shifted_t_all)
        return self.confs[closest_t_idx]


class LinearInterpolator(PathInterpolator):
    """
    Simple implementation of linear interpolation.
    https://en.wikipedia.org/wiki/Linear_interpolation

    If the time provided is:
    - before the first time, we return the first configuration.
    - after last time, we return the last configuration.
    - otherwise, we use linear interpolation
    """

    def __init__(self, t_all: np.ndarray, confs: np.ndarray):
        super().__init__(t_all, confs)
        intervals = zip(t_all, t_all[1:], confs, confs[1:])
        self.slopes = [(y2 - y1) / (x2 - x1) for x1, x2, y1, y2 in intervals]
        assert len(self.slopes) == len(t_all) - 1

    def value(self, t: float) -> np.ndarray:
        if t < self.t_all[0]:
            # Return first configuration if time earlier
            return self.confs[0]
        elif t >= self.t_all[-1]:
            # Return last configuration if time later
            return self.confs[-1]

        # Find index of t_x, where t_x <= t < t_y
        # i.e. the configuration and slope we should use for interpolating
        # https://numpy.org/doc/stable/reference/generated/numpy.searchsorted.html
        idx = np.searchsorted(self.t_all, t, side="right") - 1

        # Use the standard linear interpolation formula
        conf = self.confs[idx] + self.slopes[idx] * (t - self.t_all[idx])
        return conf
