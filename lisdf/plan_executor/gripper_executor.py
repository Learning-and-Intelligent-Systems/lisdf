
from lisdf.plan_executor.executor import CommandExecutor
from lisdf.plan_executor.robot import RobotWithGripper
from lisdf.planner_output.command import ActuateGripper, GripperPosition


class ActuateGripperExecutor(CommandExecutor):
    robot: RobotWithGripper
    command: ActuateGripper

    @property
    def duration(self) -> float:
        # Gripper command takes 1 second to execute in our simulated world for now
        return 1.0

    def execute(self, current_time: float) -> None:
        # TODO: check gripper name matches link name?
        gripper_positions = set(self.command.configurations.values())
        if len(gripper_positions) > 1:
            raise ValueError("We only support one gripper at the moment")

        gripper_position: GripperPosition = next(iter(gripper_positions))
        gripper_configuration = self.robot.gripper_configuration_for_position(
            gripper_position
        )

        # Update robot configuration
        self.robot.set_gripper_configuration(gripper_configuration)
