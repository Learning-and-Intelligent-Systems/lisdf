from typing import Dict, List, NamedTuple

import numpy as np
from pydrake.trajectories import PiecewisePolynomial


class JointSpacePathWithTime:
    def __init__(self, confs: np.ndarray, start_time: float, end_time: float):
        self.confs = confs  # (N, dimensionality of robot confs)
        self.start_time = start_time  # in seconds. Robot expected to be at path[0]
        self.end_time = end_time  # in seconds. Robot expected to be at path[-1]
        self.validate()

        # Create piecewise polynomial trajectory
        num_confs = self.confs.shape[0]
        t_all = np.linspace(self.start_time, self.end_time, num_confs)
        self._traj = PiecewisePolynomial.CubicShapePreserving(t_all, self.confs.T)

    def conf_at_time(self, t: float) -> np.ndarray:
        return self._traj.value(t)

    def validate(self):
        assert self.end_time > self.start_time


class JointSpacePaths(NamedTuple):

    paths: List[JointSpacePathWithTime]

    def joint_dimensionality(self) -> int:
        pose_dims = set([path.confs[0].shape[0] for path in self.paths])
        pose_dim = next(iter(pose_dims))
        return pose_dim

    def path_at_time(self, time: float) -> JointSpacePathWithTime:
        paths_with_time = [
            path for path in self.paths if path.start_time <= time < path.end_time
        ]
        assert len(paths_with_time) in {0, 1}
        return paths_with_time[0] if paths_with_time else None

    def validate(self):
        for path in self.paths:
            path.validate()

        # Check dimensionalities of poses are the same
        pose_dims = set([path.confs.shape[0] for path in self.paths])
        assert len(pose_dims) == 1, "Joint position shapes must be identical"

        # TODO: check start and end times don't overlap
        # TODO: check list of JointSpacePathWithTime is sorted by start_time/end_time

    def simulation_end_time(self) -> float:
        return max([path.end_time for path in self.paths])


class DrakeJointSpacePath(NamedTuple):
    robot_to_paths: Dict[str, JointSpacePaths]

    def validate(self):
        for paths in self.robot_to_paths.values():
            paths.validate()

    def simulation_end_time(self) -> float:
        return max(
            [paths.simulation_end_time() for paths in self.robot_to_paths.values()]
        )
