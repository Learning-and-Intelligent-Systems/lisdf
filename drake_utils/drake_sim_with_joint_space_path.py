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
from pydrake.trajectories import PiecewisePolynomial

from drake_utils.joint_space_controller import RobotJointSpaceController
from drake_utils.joint_space_path import JointSpacePaths, JointSpacePathWithTime
from drake_utils.utils import make_robot_controller, xyz_rpy_deg

_PANDA_HOME = np.array([-0.19, 0.08, 0.23, -2.43, 0.03, 2.52, 0.86, 0.0, 0.0])


def main(joint_space_paths: JointSpacePaths):
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 0.001)

    parser = Parser(plant)
    parser.package_map().Add("assets", "assets/")
    world = parser.AddAllModelsFromFile("assets/world.sdf")  # noqa: F841
    robot = parser.AddModelFromFile("assets/panda_arm_hand.urdf", model_name="robot")

    # Weld robot to the world so dimensionalites become cool for some reason
    plant.WeldFrames(
        frame_on_parent_P=plant.world_frame(),
        frame_on_child_C=plant.GetFrameByName("panda_link0", robot),
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
    panda_home = np.array([-0.19, 0.08, 0.23, -2.43, 0.03, 2.52, 0.86, 0.0, 0.0])
    trajs = np.array([np.zeros((9,)), panda_home, panda_home])

    t_all = np.array([0, 5.0, 10.0])

    joint_controller = builder.AddSystem(RobotJointSpaceController(joint_space_paths))

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

    # Set initial state
    meshcat_vis.reset_recording()
    meshcat_vis.start_recording()
    simulator.AdvanceTo(joint_space_paths.simulation_end_time() + 1.0)
    meshcat_vis.publish_recording()
    meshcat_vis.vis.render_static()
    input("press enter to end\n")


if __name__ == "__main__":

    paths = JointSpacePaths(
        paths=[
            JointSpacePathWithTime(
                confs=np.array([np.zeros((9,)), _PANDA_HOME / 2, _PANDA_HOME]),
                start_time=0.0,
                end_time=10.0,
            ),
            # FIXME: grippers are messed up as they are last 2 dimensionalities
            JointSpacePathWithTime(
                confs=np.array([_PANDA_HOME, 0.6 * np.ones((9,)), 0.1 * np.ones((9,))]),
                start_time=10.0,
                end_time=20.0,
            ),
        ]
    )
    main(paths)
