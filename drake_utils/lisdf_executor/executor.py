from abc import ABC, abstractmethod
from typing import Generic, TypeVar

from drake_utils.lisdf_executor.robot import DrakeRobot
from lisdf.planner_output.command import Command

# Use TypeVar so we can infer types in subclasses
RobotType = TypeVar("RobotType", bound=DrakeRobot)
CommandType = TypeVar("CommandType", bound=Command)


class CommandExecutor(ABC):
    def __init__(
        self,
        robot: Generic[RobotType],
        command: Generic[CommandType],
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
