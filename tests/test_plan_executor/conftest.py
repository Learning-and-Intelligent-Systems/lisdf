from typing import Dict, List

import pytest

from lisdf.plan_executor.robots.panda import Panda


@pytest.fixture
def panda() -> Panda:
    # Robot at zero with gripper initially open
    return Panda(configuration=Panda.INITIAL_CONFIGURATION)


@pytest.fixture
def panda_waypoints() -> Dict[str, List[float]]:
    return {
        "panda_joint1": [0.0, 0.05, 0.10, 0.15, 0.20, 0.25, 0.30],
        "panda_joint2": [0.0, 0.01, 0.02, 0.03, 0.04, 0.05, 0.06],
        "panda_joint3": [0.0, -0.1, -0.2, -0.3, -0.4, -0.5, -0.6],
        "panda_joint4": [0.0, 0.15, 0.25, 0.35, 0.45, 0.55, 0.65],
        "panda_joint5": [0.0, -0.2, -0.4, -0.6, -0.8, -1.0, -1.2],
        "panda_joint6": [0.0, 0.22, 0.44, 0.66, 0.88, 1.10, 1.32],
        "panda_joint7": [0.0, 0.11, 0.22, 0.33, 0.44, 0.55, 0.66],
    }
