from abc import ABC
from typing import Dict, Type, cast

from lisdf.plan_executor.executor import CommandExecutor
from lisdf.plan_executor.gripper_executor import ActuateGripperExecutor
from lisdf.plan_executor.interpolator import PathInterpolator
from lisdf.plan_executor.joint_space_path_executor import JointSpacePathExecutor
from lisdf.plan_executor.robots.common import Robot
from lisdf.planner_output.command import ActuateGripper, Command, JointSpacePath
from lisdf.planner_output.plan import LISDFPlan


class NoExecutorFoundError(ValueError):
    pass


class _EmptyCommand(Command, type="_EmptyCommand"):
    """To keep the typing happy."""

    @classmethod
    def _from_json_dict(cls, json_dict: Dict) -> "Command":
        raise NotImplementedError

    def validate(self):
        raise NotImplementedError


class LISDFPlanExecutor(CommandExecutor, ABC):
    """
    This class provides the functionality to execute a LISDF Plan by creating and
    getting the executor for given time steps and executing them.

    Consumers of this class need to connect it up to a simulator such as PyBullet
    or Drake.
    """

    def __init__(
        self,
        robot: Robot,
        plan: LISDFPlan,
        path_interpolator_cls: Type[PathInterpolator],
        start_time: float = 0.0,
    ):
        """
        Parameters
        ----------
        robot: the Robot to execute the plan on
        plan: the LISDF plan to execute
        path_interpolator_cls: the class to use for interpolating a joint space path
        start_time: the time to start executing the plan
        """
        super().__init__(robot=robot, command=_EmptyCommand(), start_time=start_time)
        self.plan = plan
        self._path_interpolator_cls = path_interpolator_cls

        # Note: we assume that the execute method is called with increasing time
        self._last_execute_time = self.start_time

        # Create all the executors with their respective start times based on
        # a command's duration.
        current_time = start_time
        self._executors = []
        for command in plan.commands:
            self._executors.append(self._create_executor(command, current_time))
            # Increase start time by duration of the executor
            current_time += self._executors[-1].duration

        self._current_executor_idx = 0
        self._run_sanity_checks(start_time)

    def _run_sanity_checks(self, start_time: float) -> None:
        # Sanity checks that none of the executor start and end times overlap
        prev_end_time = start_time
        for executor in self._executors:
            assert executor.end_time > executor.start_time
            assert executor.end_time == executor.start_time + executor.duration
            assert executor.start_time >= prev_end_time
            prev_end_time = executor.end_time
        assert prev_end_time == self.end_time == start_time + self.duration

    @property
    def duration(self) -> float:
        return sum(executor.duration for executor in self._executors)

    def _create_executor(self, command: Command, start_time: float) -> CommandExecutor:
        """Create command executor for the given command"""
        if command.type == JointSpacePath.type:
            return JointSpacePathExecutor(
                self.robot,
                cast(JointSpacePath, command),  # cast the type as mypy can't infer it
                start_time,
                interpolator_cls=self._path_interpolator_cls,
            )
        elif command.type == ActuateGripper.type:
            return ActuateGripperExecutor(self.robot, command, start_time)
        else:
            # You should add support for new command types here
            raise ValueError(f"Unsupported command type: {command.type}")

    def _get_executor_at_time(self, time: float) -> CommandExecutor:
        """
        Get the executor that should be executed at the given time.
        We assume this method is called with time increasing.
        """
        if self._current_executor_idx >= len(self._executors):
            raise NoExecutorFoundError(f"Time {time} is after the end of the plan")
        elif time == self.end_time:
            # Edge case where we are at the end of the plan exactly (but not beyond it)
            return self._executors[self._current_executor_idx]

        current_executor = self._executors[self._current_executor_idx]
        if time >= current_executor.end_time:
            self._current_executor_idx += 1
            # This won't cause infinite recursion because `time` stays fixed
            return self._get_executor_at_time(time)

        return current_executor

    def execute(self, current_time: float) -> None:
        """
        Execute the plan at the given time. This will grab the relevant executor and
        execute it to update the robot state.
        """
        if current_time < self._last_execute_time:
            raise RuntimeError(
                "Execute time must be increasing - i.e., time only progresses"
            )

        executor = self._get_executor_at_time(current_time)
        executor.execute(current_time)
        self._last_execute_time = current_time
