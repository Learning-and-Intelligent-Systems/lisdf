import numpy as np
import pytest

from lisdf.plan_executor.interpolator import LinearInterpolator, NearestTimeInterpolator
from lisdf.planner_output.command import JointSpacePath


@pytest.fixture
def joint_space_path(panda_waypoints) -> JointSpacePath:
    return JointSpacePath(
        waypoints=panda_waypoints,
        duration=6.0,
        label="complex_joint_space_path",
    )


NEAREST_TIME_INTERPOLATOR_TEST_CASES = [
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


@pytest.mark.parametrize(
    "interpolator_cls", [LinearInterpolator, NearestTimeInterpolator]
)
def test_interpolator_raises_error(interpolator_cls):
    # Number of timesteps does not equal number of configurations
    with pytest.raises(ValueError):
        interpolator_cls(t_all=np.array([0.0, 1.0]), confs=np.array([[0.0, 0.0, 0.1]]))

    # Timesteps are not sorted
    with pytest.raises(ValueError):
        interpolator_cls(
            t_all=np.array([0.2, 0.5, 0.3]), confs=np.array([[0.0], [0.1], [0.2]])
        )


@pytest.mark.parametrize(
    "time, expected_configuration_idx",
    NEAREST_TIME_INTERPOLATOR_TEST_CASES,
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


def test_linear_interpolator(joint_space_path, panda):
    interpolator = LinearInterpolator(
        t_all=np.linspace(
            0.0, joint_space_path.duration, joint_space_path.num_waypoints
        ),
        confs=joint_space_path.waypoints_as_np_array(panda.joint_ordering),
    )

    def waypoint_at(idx_):
        return joint_space_path.waypoint_as_np_array(idx_, panda.joint_ordering)

    # At time 0.0 or before, the configuration is the first waypoint.
    assert np.allclose(interpolator.value(0.0), waypoint_at(0))
    assert np.allclose(interpolator.value(-999), waypoint_at(0))

    # In between first and second waypoint (i.e., idx = 0 and 1),
    # check that it linearly interpolates
    slope = (waypoint_at(1) - waypoint_at(0)) / (interpolator.t_all[1])
    for time in np.linspace(0.0, interpolator.t_all[1], 50):
        assert np.allclose(interpolator.value(time), waypoint_at(0) + slope * time)

    # In between second and third waypoint (i.e., idx = 1 and 2),
    # check that it linearly interpolates
    slope = (waypoint_at(2) - waypoint_at(1)) / (
        interpolator.t_all[2] - interpolator.t_all[1]
    )
    for time in np.linspace(interpolator.t_all[1], interpolator.t_all[2], 50):
        assert np.allclose(
            interpolator.value(time),
            waypoint_at(1) + slope * (time - interpolator.t_all[1]),
        )

    # At end of command (and beyond), the configuration is the last waypoint.
    assert np.allclose(interpolator.value(joint_space_path.duration), waypoint_at(-1))
    assert np.allclose(interpolator.value(999), waypoint_at(-1))

    # Check that interpolator returns us the configuration at each given timestep
    # i.e., we want the robot to be at the configuration within the split duration
    for idx, time in enumerate(interpolator.t_all):
        assert np.allclose(interpolator.value(time), interpolator.confs[idx])
