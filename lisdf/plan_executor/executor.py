from abc import ABC, abstractmethod

from lisdf.plan_executor.robots.common import Robot
from lisdf.planner_output.command import Command


class CommandExecutor(ABC):
    """
    A CommandExecutor takes a LISDF Plan Command, and executes it on a robot by
    updating the robot's configuration for given times in the `execute` method.
    """

    def __init__(
        self,
        robot: Robot,
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

    @abstractmethod
    def execute(self, current_time: float) -> None:
        """
        Execute the command at the given time to update the robot configurations
        """
        raise NotImplementedError
