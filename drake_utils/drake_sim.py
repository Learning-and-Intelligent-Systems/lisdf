import numpy as np
from position_controller import RobotPositionController
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
from pydrake.trajectories import PiecewisePolynomial

from drake_utils.utils import make_robot_controller, xyz_rpy_deg


def main():
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 0.001)

    parser = Parser(plant)
    parser.package_map().Add("assets", "assets/")
    world = parser.AddAllModelsFromFile("assets/world.sdf")  # noqa: F841
    # robot_path = "assets/panda_arm_hand.urdf"
    robot_path = "assets/pr2_description/urdf/pr2_simplified.urdf"
    robot = parser.AddModelFromFile(robot_path, model_name="robot")

    # Weld robot to the world
    plant.WeldFrames(
        frame_on_parent_P=plant.world_frame(),
        frame_on_child_C=plant.GetFrameByName("world_link", robot),
        X_PC=xyz_rpy_deg([0, 0, 0], [0, 0, 0]),
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

    # Make a traj that goes from 0 to home
    simulation_time = 7.0
    panda_home = np.array([-0.19, 0.08, 0.23, -2.43, 0.03, 2.52, 0.86, 0.0, 0.0])
    # trajs = np.array([np.zeros((9,)), panda_home, panda_home])
    trajs = np.array([np.zeros(28,), np.zeros(28,), np.zeros(28,)])

    t_all = np.array([0, simulation_time/2.0, simulation_time])
    q_traj = PiecewisePolynomial.CubicShapePreserving(t_all, trajs.T)

    qs_for_controller = [q_traj.value(t) for t in np.linspace(0, 5.0, 100)]
    print("num robot confs q in traj:", len(qs_for_controller))

    time_step = simulation_time / len(qs_for_controller)

    joint_controller = builder.AddSystem(
        RobotPositionController(qs_for_controller, time_step=time_step)
    )

    torque_controller = builder.AddSystem(
        make_robot_controller(robot_path)
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
    #plant.get_actuation_input_port(robot).FixValue(plant_context, np.zeros((28,1)))

    # Set initial state
    meshcat_vis.reset_recording()
    meshcat_vis.start_recording()
    simulator.AdvanceTo(simulation_time)
    meshcat_vis.publish_recording()
    meshcat_vis.vis.render_static()
    input("press enter to end\n")


if __name__ == "__main__":
    main()
