import numpy as np
import pytest

from lisdf.plan_executor.interpolator import NearestTimeInterpolator
from lisdf.plan_executor.joint_space_path_executor import JointSpacePathExecutor
from lisdf.planner_output.command import JointSpacePath
from tests.test_plan_executor.test_interpolator import (
    NEAREST_TIME_INTERPOLATOR_TEST_CASES,
)


@pytest.fixture
def joint_space_path(panda_waypoints) -> JointSpacePath:
    return JointSpacePath(
        waypoints=panda_waypoints,
        duration=6.0,
        label="complex_joint_space_path",
    )


@pytest.fixture
def executor(panda, joint_space_path) -> JointSpacePathExecutor:
    return JointSpacePathExecutor(
        panda, joint_space_path, 0.0, interpolator_cls=NearestTimeInterpolator
    )


def test_joint_space_path_executor_parses_duration(executor, joint_space_path):
    assert executor.duration == joint_space_path.duration


def test_joint_space_path_executor_creates_interpolator(
    executor, panda, joint_space_path
):
    interpolator = executor._interpolator
    assert np.allclose(
        interpolator.t_all, np.array([0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0])
    )
    assert np.allclose(
        interpolator.confs, joint_space_path.waypoints_as_np_array(panda.joint_ordering)
    )


@pytest.mark.parametrize(
    "time, expected_configuration_idx",
    # Since we use the NearestTimeInterpolator, we can reuse test cases here
    NEAREST_TIME_INTERPOLATOR_TEST_CASES,
)
def test_joint_space_path_executor_execute(
    time, expected_configuration_idx, executor, panda, joint_space_path
):
    executor.execute(time)
    assert np.allclose(
        panda.joint_configuration,
        joint_space_path.waypoint_as_np_array(
            expected_configuration_idx, panda.joint_ordering
        ),
    )
