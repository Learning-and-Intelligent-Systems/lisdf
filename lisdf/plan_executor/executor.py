from abc import ABC, abstractmethod
from typing import TypeVar

from lisdf.plan_executor.robot_state import RobotWithState
from lisdf.planner_output.command import Command

# Use TypeVar so we can infer types in subclasses
RobotType = TypeVar("RobotType", bound=RobotWithState)
CommandType = TypeVar("CommandType", bound=Command)


class CommandExecutor(ABC):
    """
    A CommandExecutor takes a LISDF Plan Command, and executes it on a robot by
    updating the robot's configuration for given times in the `execute` method.
    """

    def __init__(
        self,
        robot: RobotWithState,
        command: Command,
        start_time: float,
    ):
        self.robot = robot
        self.command = command
        self.start_time = start_time

    @property
    @abstractmethod
    def duration(self) -> float:
        raise NotImplementedError

    @property
    def end_time(self) -> float:
        return self.start_time + self.duration

    def finished(self, current_time: float) -> bool:
        """Whether this executor has finished executing"""
        return current_time >= self.end_time

    def execute(self, current_time: float) -> None:
        """
        Execute the command at the given time to update the robot configurations
        """
        raise NotImplementedError
