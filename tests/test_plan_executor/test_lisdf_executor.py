import pytest
from mock import Mock, call

from lisdf.plan_executor.gripper_executor import ActuateGripperExecutor
from lisdf.plan_executor.interpolator import NearestTimeInterpolator
from lisdf.plan_executor.joint_space_path_executor import JointSpacePathExecutor
from lisdf.plan_executor.lisdf_executor import LISDFPlanExecutor, NoExecutorFoundError
from lisdf.planner_output.command import ActuateGripper, GripperPosition, JointSpacePath
from lisdf.planner_output.plan import LISDFPlan


@pytest.fixture
def plan(panda_waypoints) -> LISDFPlan:
    reversed_waypoints = {
        joint: list(reversed(confs)) for joint, confs in panda_waypoints.items()
    }

    return LISDFPlan(
        lisdf_problem="my_cool_problem",
        commands=[
            JointSpacePath(
                waypoints=panda_waypoints,
                duration=3.0,
                label="move_to_pick",
            ),
            ActuateGripper(
                configurations={"gripper_1": GripperPosition.close}, label="pick"
            ),
            JointSpacePath(
                waypoints=reversed_waypoints,
                duration=6.0,
                label="move_to_place",
            ),
            ActuateGripper(
                configurations={"gripper_1": GripperPosition.open}, label="place"
            ),
        ],
    )


@pytest.fixture
def executor(panda, plan) -> LISDFPlanExecutor:
    return LISDFPlanExecutor(
        robot=panda,
        plan=plan,
        path_interpolator_cls=NearestTimeInterpolator,
        start_time=0.0,
    )


def test_lisdf_plan_executor(executor, plan, panda):
    # 3.0 + 6.0 + 2 * 1.0 (for gripper)
    assert ActuateGripperExecutor.DEFAULT_DURATION == 1.0  # test relies on this for now
    assert executor.duration == 3.0 + 6.0 + 2.0

    # Check executors created conform to plan
    for idx, ((expected_type, expected_start_time), command_executor) in enumerate(
        zip(
            [
                (JointSpacePathExecutor, 0.0),
                (ActuateGripperExecutor, 3.0),
                (JointSpacePathExecutor, 4.0),
                (ActuateGripperExecutor, 10.0),
            ],
            executor._executors,
        )
    ):
        assert type(command_executor) == expected_type
        assert command_executor.command == plan.commands[idx]
        assert command_executor.start_time == expected_start_time
        assert (
            command_executor.end_time == expected_start_time + command_executor.duration
        )

        command_executor.execute = Mock()

    # Execute the LISDFPlanExecutor for given times
    expected_times = [
        [0.0, 0.2, 1.0, 2.0, 2.99],  # move to pick
        [3.0, 3.2, 3.99],  # pick
        [4.0, 5.0, 7.0, 8.5, 9.95],  # move to place
        [10.0, 11.0],  # place
    ]
    for idx, times in enumerate(expected_times):
        for current_time in times:
            executor.execute(current_time)

        # Check calls were made to underlying execute method in expected order
        command_executor = executor._executors[idx]
        command_executor.execute.assert_has_calls([call(t) for t in times])

    # Check no additional calls
    for idx, command_executor in enumerate(executor._executors):
        assert command_executor.execute.call_count == len(expected_times[idx])

    # Error raised when time goes backwards
    with pytest.raises(RuntimeError):
        executor.execute(1.0)

    # Raise error when we try execute for time beyond the plan
    with pytest.raises(NoExecutorFoundError):
        executor.execute(999)
