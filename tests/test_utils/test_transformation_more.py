import numpy as np
import pytest

from lisdf.utils.transformations import euler_matrix
from lisdf.utils.transformations_more import lookat_rpy


@pytest.mark.parametrize(
    "camera_pos, target_pos",
    [
        [[3, 8, 3], [0, 8, 1]],
        [[3, 8, 3], [0, 9, 1]],
        [[3, 8, 3], [0, 7, 1]],
        [[0, 8, 1], [3, 8, 3]],
        [[0, 9, 1], [3, 8, 3]],
        [[0, 7, 1], [3, 8, 3]],
    ],
)
def test_lookat_rpy(camera_pos, target_pos):
    camera_pos = np.array(camera_pos, dtype="float32")
    target_pos = np.array(target_pos, dtype="float32")
    delta = target_pos - camera_pos
    delta = delta / np.linalg.norm(delta)

    rpy = lookat_rpy(camera_pos, target_pos)
    mat = euler_matrix(*rpy)
    z = np.array([0, 0, 1, 1], dtype="float32")  # the unit-z vector.
    # the unit-z vector of the camera frame in the world frame.
    z_in_world = (mat @ z)[:-1]

    assert np.allclose(
        z_in_world / np.linalg.norm(z_in_world),
        delta / np.linalg.norm(delta),
        atol=1e-6,
    )
