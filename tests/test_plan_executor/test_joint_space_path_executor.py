import numpy as np
import pytest

from lisdf.plan_executor.joint_space_path_executor import (
    JointSpacePathExecutor,
    NearestTimeInterpolator,
)
from lisdf.planner_output.command import JointSpacePath


@pytest.fixture
def joint_space_path(panda_waypoints) -> JointSpacePath:
    return JointSpacePath(
        waypoints=panda_waypoints,
        duration=6.0,
        label="complex_joint_space_path",
    )


_TIME_AND_EXPECTED_CONFIGURATIONS = [
    # (time, expected configuration index)
    pytest.param(-1.0, 0, id="time_before_start"),
    pytest.param(0.4999, 0, id="initial_configuration"),
    pytest.param(0.5, 0, id="tie_breaking"),
    pytest.param(0.5001, 1, id="edge_case_second_configuration"),
    pytest.param(0.999, 1, id="second_configuration"),
    pytest.param(5.501, -1, id="edge_case_last_configuration"),
    pytest.param(6.0, -1, id="last_configuration"),
    pytest.param(999, -1, id="time_beyond_duration"),
]


@pytest.fixture
def executor(panda, joint_space_path) -> JointSpacePathExecutor:
    return JointSpacePathExecutor(
        panda, joint_space_path, 0.0, interpolator_cls=NearestTimeInterpolator
    )


def test_interpolator_raises_error():
    with pytest.raises(ValueError):
        # Number of timesteps does not equal number of configurations
        NearestTimeInterpolator(
            t_all=np.array([0.0, 1.0]), confs=np.array([[0.0, 0.0, 0.1]])
        )


@pytest.mark.parametrize(
    "time, expected_configuration_idx",
    _TIME_AND_EXPECTED_CONFIGURATIONS,
)
def test_nearest_time_interpolator(
    time, expected_configuration_idx, panda, joint_space_path
):
    # NearestTimeInterpolator interpolates to the waypoint with the nearest time.
    # Ties (i.e., at 0.5) are broken by choosing the waypoint that came first in time.
    interpolator = NearestTimeInterpolator(
        t_all=np.linspace(
            0.0, joint_space_path.duration, joint_space_path.num_waypoints
        ),
        confs=joint_space_path.waypoints_as_np_array(panda.joint_ordering),
    )
    assert np.allclose(
        interpolator.value(time),
        joint_space_path.waypoint_as_np_array(
            expected_configuration_idx, panda.joint_ordering
        ),
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
    _TIME_AND_EXPECTED_CONFIGURATIONS,
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
