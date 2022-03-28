#!/usr/bin/env python3
from lisdf.planner_output.command import ActuateGripper, JointSpacePath
from lisdf.planner_output.plan import LISDFPlan
from tests.test_planner_output.conftest import generate_complex_commands


def main() -> None:
    # Check the generate_complex_commands() method for examples on how to
    # generate commands
    lisdf_plan = LISDFPlan(lisdf_problem=".", commands=generate_complex_commands())
    # We can convert the plan into json, the indent just makes it human-readable
    print("=== JSON ===")
    print(lisdf_plan.to_json())

    # We can also convert the plan into YAML (which is a superset of JSON)
    print("\n=== YAML ===")
    print(lisdf_plan.to_yaml())

    # We can load a plan from JSON or YAML
    print("\n=== Loading plans from JSON and YAML ===")
    print(
        "Deserialize from JSON is identical?",
        LISDFPlan.from_json(lisdf_plan.to_json()) == lisdf_plan,
    )
    print(
        "Deserialize from YAML is identical?",
        LISDFPlan.from_yaml(lisdf_plan.to_yaml()) == lisdf_plan,
    )

    # We can do cool things with a JointSpacePath
    joint_space_path: JointSpacePath = lisdf_plan.commands[0]  # type: ignore
    print("\n=== JointSpacePath ===")
    print("Dict Representation:", joint_space_path.to_dict())
    print("Joint Names:", joint_space_path.joint_names)
    print("Dimensionality:", joint_space_path.dimensionality)
    print("Number of Waypoints:", joint_space_path.num_waypoints)

    print("Waypoints for joint_2:", joint_space_path.waypoints_for_joint("joint_2"))
    print("Last waypoint as a Dict[str, float]:", joint_space_path.waypoint(-1))

    # We can choose the order in which the np.array is ordered. Let's just use
    # the default expected order for now
    joint_name_ordering = ["joint_1", "joint_2", "joint_3"]
    print(
        "Last waypoint as a np.array:",
        joint_space_path.waypoint_as_np_array(-1, joint_name_ordering),
    )
    print(
        "Waypoints as np.array:\n",
        joint_space_path.waypoints_as_np_array(joint_name_ordering),
    )

    # We can do not less cool things with a ActuateGripper
    print("\n=== ActuateGripper ===")
    actuate_gripper: ActuateGripper = lisdf_plan.commands[1]  # type: ignore
    print("Dict Representation:", actuate_gripper.to_dict())
    print("Joint Names:", actuate_gripper.joint_names)
    print(
        "gripper_1 position:", actuate_gripper.position_for_gripper_joint("gripper_1")
    )


if __name__ == "__main__":
    main()
