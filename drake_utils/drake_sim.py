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
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.meshcat_visualizer import ConnectMeshcatVisualizer
from pydrake.systems.sensors import CameraInfo, RgbdSensor

from drake_utils.lisdf_controller import LISDFPlanController
from drake_utils.robot.panda import Panda
from drake_utils.utils import make_robot_controller, xyz_rpy_deg
from lisdf.planner_output.plan import LISDFPlan


def main(plan: LISDFPlan):
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 0.01)

    parser = Parser(plant)
    parser.package_map().Add("assets", "assets/")
    parser.AddAllModelsFromFile("lisdf-models/models/m0m_panda/model.sdf")
    robot = parser.AddModelFromFile("assets/panda_arm_hand.urdf", model_name="robot")

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
    simulate_physics = True

    if simulate_physics:
        # FIXME: init can be read from LISDF
        # if last_q_des is None and the first command is ActuateGripper, we crash)
        # gripper open initially
        panda_init_conf = np.concatenate([np.zeros((7,)), np.array([0.03, 0.03])])
        lisdf_controller = LISDFPlanController(
            robot=Panda(configuration=panda_init_conf), plan=plan
        )
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

    diagram = builder.Build()

    simulator = Simulator(diagram)
    simulator.Initialize()
    simulator.get_mutable_context().SetTime(0.0)
    plant_context = plant.GetMyContextFromRoot(  # noqa: F841
        simulator.get_mutable_context()
    )

    if not simulate_physics:
        plant.get_actuation_input_port().FixValue(plant_context, np.zeros(9))
        plant.mutable_gravity_field().set_gravity_vector(np.array([0, 0, 0.0]))

    # Set initial state
    meshcat_vis.reset_recording()
    meshcat_vis.start_recording()
    total_time = 60.0

    if simulate_physics:
        simulator.AdvanceTo(total_time)
    else:
        t = 0.0
        while t < total_time:
            # See commit dc919111d on the sdrake branch for and example of how
            # this used to be implemented before the outputspec was formalized
            raise NotImplementedError

    meshcat_vis.publish_recording()
    meshcat_vis.vis.render_static()
    input("press enter to end\n")


if __name__ == "__main__":
    plan_json = open("lisdf_plan.json").read()
    plan = LISDFPlan.from_json(plan_json)
    main(plan)
