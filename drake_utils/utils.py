import numpy as np
from pydrake.math import RigidTransform, RollPitchYaw
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.controllers import InverseDynamicsController
from pydrake.systems.framework import DiagramBuilder


def xyz_rpy_deg(xyz, rpy_deg):
    """Shorthand for defining a pose."""
    rpy_deg = np.asarray(rpy_deg)
    return RigidTransform(RollPitchYaw(rpy_deg * np.pi / 180), xyz)


def make_robot_controller(robot_model):
    cbuilder = DiagramBuilder()
    cplant, _ = AddMultibodyPlantSceneGraph(cbuilder, 0.001)
    cparser = Parser(cplant)
    cparser.package_map().Add("assets", "assets/")
    robot = cparser.AddModelFromFile(robot_model)
    # TODO: the location of the weldjoint should be read from the URDF?
    # TODO: the base link must be read from the URDF
    cplant.WeldFrames(
        frame_on_parent_P=cplant.world_frame(),
        frame_on_child_C=cplant.GetFrameByName("panda_link0", robot),
        X_PC=xyz_rpy_deg([0, 0, 0], [0, 0, 0]),
    )
    cplant.Finalize()
    kp = 1200 * np.ones((9, 1))
    ki = 10 * np.ones((9, 1))
    kd = (2 * np.sqrt(1200)) * np.ones((9, 1))
    """
    kp = 200.0*np.ones((28, 1))
    ki = 2.0*np.ones((28, 1))
    kd = 20.0*np.ones((28, 1))
    """
    torque_controller = InverseDynamicsController(cplant, kp, ki, kd, False)
    return torque_controller
