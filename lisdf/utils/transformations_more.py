import math

import numpy as np

from lisdf.utils.typing import Vector3f


def lookat_rpy(camera_pos: Vector3f, target_pos: Vector3f) -> np.ndarray:
    """Construct the roll, pitch, yaw angles of a camera looking at a target.
    This function assumes that the camera is pointing to the z-axis ([0, 0, 1]),
    in the camera frame.

    Args:
        camera_pos: the position of the camera.
        target_pos: the target position.

    Returns:
        a numpy array of the roll, pitch, yaw angles.
    """
    delta = target_pos - camera_pos
    pitch = math.atan2(-np.linalg.norm(delta[:2]), delta[2])
    yaw = math.atan2(-delta[1], -delta[0])
    return np.array([0, pitch, yaw], dtype="float32")
