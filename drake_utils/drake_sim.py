import time

import numpy as np
from pydrake.geometry.render import (
    ClippingRange,
    DepthRange,
    DepthRenderCamera,
    MakeRenderEngineVtk,
    RenderCameraCore,
    RenderEngineVtkParams,
)
from pydrake.math import RigidTransform
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.multibody.tree import Joint  # noqa: F401
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.meshcat_visualizer import ConnectMeshcatVisualizer
from pydrake.systems.sensors import CameraInfo, RgbdSensor

from drake_utils.interpolator_wrapper import DrakePiecewisePolynomialInterpolator
from drake_utils.lisdf_animator import LISDFPlanAnimator
from drake_utils.lisdf_controller import LISDFPlanController
from drake_utils.panda import Panda
from drake_utils.utils import make_robot_controller, xyz_rpy_deg
from lisdf.plan_executor.lisdf_executor import LISDFPlanExecutor
from lisdf.planner_output.plan import LISDFPlan
from lisdf.parsing.parse_sdf import load_sdf

def dump_xml(fname, xml_string):
    with open(fname, "w") as f:
        f.write(xml_string)

def main(plan: LISDFPlan):
    builder = DiagramBuilder()
    # if simulate_physics=False then 1 min of sim time = ~5 min of realtime
    simulate_physics = True
    if simulate_physics:
        plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 0.001)
    else:
        plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 0)

    parser = Parser(plant)
    parser.package_map().Add("franka_description", "lisdf-models/models/panda/")
    parser.package_map().Add("assets", "assets/")
    problem_sdf = load_sdf("m0m_panda/model.sdf", "lisdf-models/models/")
    for world in problem_sdf.worlds:
        for model in world.models:
            xml_string = "\n".join(model.to_xml_string().split("\n")[1:])
            if not "</robot>" in xml_string:
                xml_string = "<?xml version=\"1.0\" ?>\n<sdf version=\"1.4\">\n{}\n</sdf>".format(xml_string)
                fname = "model.sdf"
                dump_xml(fname, xml_string)
                parser.AddModelFromFile(fname)
            else:
                fname = "model.urdf"
                xml_string = "<?xml version=\"1.0\" encoding=\"utf-8\"?>\n{}\n".format(xml_string)
                dump_xml(fname, xml_string)
                robot = parser.AddModelFromFile(fname, model_name="robot")
    #robot = parser.AddModelFromFile("assets/panda_arm_hand.urdf", model_name="robot")

    # Weld robot to the world
    plant.WeldFrames(
        frame_on_parent_P=plant.world_frame(),
        frame_on_child_C=plant.GetFrameByName("panda_link0", robot),
        X_PC=xyz_rpy_deg([0.0, 0, 0], [0, 0, 0]),
    )

    renderer_name = "renderer"
    scene_graph.AddRenderer(renderer_name, MakeRenderEngineVtk(RenderEngineVtkParams()))
    depth_camera = DepthRenderCamera(
        RenderCameraCore(
            renderer_name,
            CameraInfo(width=640, height=480, fov_y=np.pi / 4),
            ClippingRange(0.01, 10.0),
            RigidTransform(),
        ),
        DepthRange(0.01, 10.0),
    )
    world_id = plant.GetBodyFrameIdOrThrow(plant.world_body().index())
    X_WB = xyz_rpy_deg([4, 0, 0], [-90, 0, 90])
    sensor = RgbdSensor(world_id, X_PB=X_WB, depth_camera=depth_camera)
    builder.AddSystem(sensor)
    builder.Connect(
        scene_graph.get_query_output_port(), sensor.query_object_input_port()
    )
    meshcat_vis = ConnectMeshcatVisualizer(
        builder, scene_graph, zmq_url="new", open_browser=False
    )

    plant.Finalize()

    panda_init_conf = np.concatenate([np.zeros((7,)), np.array([0.05, 0.05])])
    plan_executor = LISDFPlanExecutor(
        robot=Panda(configuration=panda_init_conf),
        plan=plan,
        path_interpolator_cls=DrakePiecewisePolynomialInterpolator,
    )

    if simulate_physics:
        # FIXME: init can be read from LISDF
        # if last_q_des is None and the first command is ActuateGripper, we crash)
        # gripper open initially
        lisdf_controller = LISDFPlanController(plan_executor)
        joint_controller = builder.AddSystem(lisdf_controller)

        # Connect the robot controller to the plant
        torque_controller = builder.AddSystem(
            make_robot_controller("assets/panda_arm_hand.urdf")
        )
        builder.Connect(
            joint_controller.get_output_port(),
            torque_controller.get_input_port_desired_state(),
        )
        builder.Connect(
            plant.get_state_output_port(robot),
            torque_controller.get_input_port_estimated_state(),
        )
        builder.Connect(
            torque_controller.get_output_port_control(),
            plant.get_actuation_input_port(robot),
        )
    else:
        lisdf_controller = LISDFPlanController(plan_executor)
        joint_controller = builder.AddSystem(lisdf_controller)

        robot_animator = builder.AddSystem(
            LISDFPlanAnimator(
                robot=Panda(configuration=panda_init_conf),
                plant=plant,
                sim_context=None,
            )
        )
        builder.Connect(
            joint_controller.get_output_port(), robot_animator.get_input_port()
        )
        builder.Connect(
            robot_animator.get_output_port(), plant.get_actuation_input_port(robot)
        )

    diagram = builder.Build()

    simulator = Simulator(diagram)
    simulator.Initialize()
    # simulator.set_target_realtime_rate(1.0)

    simulator.get_mutable_context().SetTime(0.0)
    plant_context = plant.GetMyContextFromRoot(  # noqa: F841
        simulator.get_mutable_context()
    )

    if not simulate_physics:
        plant.mutable_gravity_field().set_gravity_vector(np.array([0, 0, 0.0]))
        robot_animator.sim_context = plant_context
        # plant.get_actuation_input_port().FixValue(plant_context, np.zeros(9))

    # Set initial state
    meshcat_vis.reset_recording()
    meshcat_vis.start_recording()

    # Execute for plan duration and add 1 second for good measure
    total_time = plan_executor.duration + 1.0
    print(f"Advancing for {total_time}s")

    # Measure how long the simulation actually took given we set realtime rate
    simulate_start_time = time.perf_counter()
    simulator.AdvanceTo(total_time)

    simulate_end_time = time.perf_counter()
    simulate_duration = simulate_end_time - simulate_start_time
    print(f"Simulating {total_time}s took {simulate_duration}s")

    meshcat_vis.publish_recording()
    meshcat_vis.vis.render_static()
    input("press enter to end\n")


if __name__ == "__main__":
    plan_json = open("lisdf_plan.json").read()
    plan_ = LISDFPlan.from_json(plan_json)
    main(plan_)
