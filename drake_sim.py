from pydrake.geometry.render import (
    ClippingRange,
    DepthRange,
    DepthRenderCamera,
    RenderCameraCore,
    MakeRenderEngineVtk,
    RenderEngineVtkParams,
)

from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.multibody.parsing import Parser
from pydrake.systems.meshcat_visualizer import ConnectMeshcatVisualizer
from pydrake.systems.sensors import (
    CameraInfo,
    RgbdSensor,
)
from pydrake.math import RigidTransform, RotationMatrix, RollPitchYaw
from utils.drake_utils import xyz_rpy_deg, make_robot_controller
from drake_robot_position_controller import DrakeRobotPositionController
import numpy as np

builder = DiagramBuilder()
plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 0.001)

parser = Parser(plant)
parser.package_map().Add('assets', 'assets/')
robot = parser.AddModelFromFile('assets/panda_arm_hand.urdf', model_name="robot")
world = parser.AddAllModelsFromFile('assets/world.sdf')

renderer_name = "renderer"
scene_graph.AddRenderer(
    renderer_name, MakeRenderEngineVtk(RenderEngineVtkParams()))
depth_camera = DepthRenderCamera(
    RenderCameraCore(
        renderer_name,
        CameraInfo(width=640, height=480, fov_y=np.pi/4),
        ClippingRange(0.01, 10.0),
        RigidTransform()),
    DepthRange(0.01, 10.0))
world_id = plant.GetBodyFrameIdOrThrow(plant.world_body().index())
X_WB = xyz_rpy_deg([4, 0, 0], [-90, 0, 90])
sensor = RgbdSensor(
    world_id, X_PB=X_WB,
    depth_camera=depth_camera)
builder.AddSystem(sensor)
builder.Connect(
    scene_graph.get_query_output_port(),
    sensor.query_object_input_port())
meshcat_vis = ConnectMeshcatVisualizer(
    builder, scene_graph, zmq_url="new", open_browser=False)

plant.Finalize()

joint_controller = builder.AddSystem(DrakeRobotPositionController([np.zeros((9,))]))
torque_controller = builder.AddSystem(make_robot_controller("assets/panda_arm_hand.urdf"))
builder.Connect(joint_controller.get_output_port(), torque_controller.get_input_port_desired_state())
builder.Connect(plant.get_state_output_port(panda_1), torque_controller.get_input_port_estimated_state())
builder.Connect(torque_controller.get_output_port_control(), plant.get_actuation_input_port(robot))
diagram = builder.Build()

simulator = Simulator(diagram)
simulator.Initialize()
simulator.get_mutable_context().SetTime(.0)
plant_context = plant.GetMyContextFromRoot(simulator.get_mutable_context())

meshcat_vis.reset_recording()
meshcat_vis.start_recording()
simulator.AdvanceTo(5.0)
meshcat_vis.publish_recording()
meshcat_vis.vis.render_static()
input("press enter to end\n")
