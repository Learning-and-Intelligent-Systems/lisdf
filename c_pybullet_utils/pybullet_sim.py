import os
import time

import numpy as np
import pybullet as p
from pybullet_tools.utils import (
    TEMP_DIR,
    connect,
    disable_gravity,
    set_joint_positions,
    wait_for_user,
)
from pydrake.multibody.tree import Joint  # noqa: F401

from drake_utils.interpolator_wrapper import DrakePiecewisePolynomialInterpolator
from drake_utils.panda import Panda
from lisdf.parsing.parse_sdf import load_sdf
from lisdf.plan_executor.lisdf_executor import LISDFPlanExecutor
from lisdf.planner_output.plan import LISDFPlan

"""
Note: you need to clone pybullet-planning locally for this to work
https://github.com/caelan/pybullet-planning
"""


def from_lisdf():
    scene_dir = "lisdf-models/models/"
    sdf_struct = load_sdf("m0m_panda/model.sdf", scene_dir)

    p.setAdditionalSearchPath(scene_dir)
    panda = p.loadURDF(
        "lisdf-models/models/panda/robots/panda_spheres.urdf",
        useFixedBase=True
        # "lisdf-models/models/panda/robots/panda_arm_hand.urdf", useFixedBase=True
    )

    world = sdf_struct.aggregate_order[0]
    objects = []
    for mi, model in enumerate(world.models):
        intermediate_path = os.path.join(TEMP_DIR, "{}.sdf".format(str(mi)))
        print(intermediate_path)
        with open(intermediate_path, "w") as f:
            xml_string = "\n".join(model.to_xml_string().split("\n")[1:])
            f.write("<sdf>\n{}\n</sdf>".format(xml_string))
        body_id = p.loadSDF(intermediate_path)
        objects.append(body_id)

    print(objects)
    return objects, panda


def main(plan: LISDFPlan):
    connect(use_gui=True)
    objects, panda = from_lisdf()
    disable_gravity()
    panda_init_conf = np.concatenate([np.zeros((7,)), np.array([0.05, 0.05])])

    jnt_to_id = {}
    non_fixed_jnt_names = []
    robot_id = panda

    for i in range(p.getNumJoints(robot_id)):
        info = p.getJointInfo(robot_id, i)
        jnt_name = info[1].decode("UTF-8")
        jnt_to_id[jnt_name] = info[0]
        if info[2] != p.JOINT_FIXED:
            non_fixed_jnt_names.append(jnt_name)

    arm_jnt_ids = [jnt_to_id[jnt] for jnt in non_fixed_jnt_names]

    #################

    plan_executor = LISDFPlanExecutor(
        robot=Panda(configuration=panda_init_conf),
        plan=plan,
        path_interpolator_cls=DrakePiecewisePolynomialInterpolator,
    )
    robot = plan_executor.robot

    start_time = time.perf_counter()
    current_time = 0
    itt = 0
    try:
        while current_time < plan_executor.end_time:

            plan_executor.execute(current_time)

            plan_executor.execute(current_time)

            set_joint_positions(robot_id, arm_jnt_ids, robot.configuration)
            # doesn't work with physics
            # p.setJointMotorControlArray(
            #     panda,
            #     arm_jnt_ids,
            #     p.POSITION_CONTROL,
            #     targetPositions=list(robot.configuration),
            # )
            states = p.getJointStates(robot_id, arm_jnt_ids)
            pos = [state[0] for state in states]

            if itt % 10 == 0:
                print(current_time, "des:", robot.configuration)
                print("actual:", pos)
                print("======================")

            itt += 1
            # p.stepSimulation()
            time.sleep(0.01)
            current_time = time.perf_counter() - start_time
    except Exception as e:
        print(e)

    states = p.getJointStates(robot_id, arm_jnt_ids)
    pos = [state[0] for state in states]
    print("current time:", current_time)
    print("actual:", pos)
    wait_for_user("Finish")


if __name__ == "__main__":
    plan_json = open("lisdf_plan.json").read()
    plan_ = LISDFPlan.from_json(plan_json)
    main(plan_)
