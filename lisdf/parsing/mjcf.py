import os.path as osp
from copy import deepcopy

import numpy as np

import lisdf.components as C
from lisdf.parsing.string_utils import (
    bool_string,
    vector2f,
    vector3f,
    vector4f,
    wxyz_from_euler,
)
from lisdf.parsing.xml_j.visitor import XMLVisitor, check_done
from lisdf.parsing.xml_j.xml import XMLNode


class MJCFVisitorFlatten(XMLVisitor):
    """A simple MJCF reader that expands all include commands."""

    def include(self, node: XMLNode) -> XMLNode:
        filename = osp.join(
            osp.dirname(self.filename_stack[-1]), node.attributes["file"]
        )
        filename = osp.normpath(filename)
        node = self.load_file(filename)
        node.attributes["filename"] = filename
        return node


class MJCFVisitor(XMLVisitor):
    """
    A minimal MJCF reader that handles a subset of all attributes.
    """

    # TODO(Jiayuan Mao @ 03/24): think about a better way to unify
    # the parsing of sdf and mjcf files.

    # TODO(Jiayuan Mao @ 03/24): write better docs for check_done.

    """Defaults"""

    def default_init(self, node: XMLNode):
        st = self._st["default"]
        if node.attributes.get("class", None) is None:
            assert len(st) == 0
            st.append(dict(geom=dict(), joint=dict(), position=dict()))
            self._data.setdefault("defaults", dict())
        else:
            st.append(deepcopy(st[-1]))

    def default(self, node: XMLNode):
        st = self._st["default"]
        if node.attributes.get("class", None) is None:
            st.pop()
            assert len(st) == 0
        else:
            classname = node.attributes["class"]
            self._data["defaults"][classname] = self._st["default"][-1]
            st.pop()
        node.attributes.pop("class", None)
        return check_done(node)

    """Asset"""

    def asset(self, node: XMLNode):
        return check_done(node)

    def mesh(self, node: XMLNode):
        file = node.attributes.pop("file")
        name = node.attributes.pop("name")
        scale = node.attributes.pop("scale", "1 1 1")
        data = self._data["mesh"]
        assert name not in data
        data[name] = dict(filename=file, scale=scale)
        return check_done(node)

    def texture(self, node: XMLNode):
        name = node.attributes.pop("name", "skybox")
        if name == "skybox":
            assert node.attributes["type"] == "skybox"
        data = self._data["texture"]
        assert name not in data
        data[name] = node.attributes.copy()
        return check_done(node, attr=False)

    def material(self, node: XMLNode):
        name = node.attributes.pop("name")
        data = self._data["material"]
        assert name not in data
        data[name] = node.attributes.copy()
        return check_done(node, attr=False)

    def include(self, node: XMLNode) -> XMLNode:
        filename = osp.join(
            osp.dirname(self.filename_stack[-1]), node.attributes["file"]
        )
        filename = osp.normpath(filename)
        node = self.load_file(filename)
        if node is not None:
            node.attributes["filename"] = filename
        return node

    def mujocoinclude(self, node: XMLNode):
        """<mujocoinclude> are used only as the root tag for mujoco-included files."""
        if len(node.children) == 0:
            return check_done(node)
        return node

    def body_init(self, node: XMLNode):
        name = node.attributes.get("name", None)
        pose = self._find_pose(node)
        parent = None
        if len(self._st["body"]):
            parent = self._st["body"][-1].name

        data = self._data["body"]
        if name is not None:
            assert name not in data
            data[name] = C.Link(name, parent, pose)
            self._st["body"].append(data[name])
        if "childclass" in node.attributes:
            self._st["class"].append(node.attributes["childclass"])

    def body(self, node: XMLNode):
        name = node.attributes.pop("name", None)
        if name is not None:
            self._st["body"].pop()

        if "childclass" in node.attributes:
            self._st["class"].pop()
            node.attributes.pop("childclass")

        # TODO(Jiayuan Mao @ 03/23): fix this.
        node.attributes.pop("mocap", None)
        return check_done(node)

    def camera(self, node: XMLNode):
        # TODO(Jiayuan Mao @ 03/23): fix this.
        return None

    def inertial(self, node: XMLNode):
        pose = self._find_pose(node)
        mass = node.attributes.pop("mass")
        diaginertia = node.attributes.pop("diaginertia")

        inertial = C.Inertial(
            float(mass), pose, C.Inertia.from_diagonal(*vector3f(diaginertia))
        )
        body = self._st["body"][-1]
        assert body.inertial is None
        body.inertial = inertial
        return check_done(node)

    def site(self, node: XMLNode):
        name = node.attributes.pop("name")
        type = node.attributes.pop("type", "sphere")
        pose = self._find_pose(node)
        size = node.attributes.pop("size", "0.0005")
        rgba = node.attributes.pop("rgba", "0.5 0.5 0.5 1")

        if len(self._st["body"]) == 0:
            assert name is not None
            body = C.Link(name, None, pose)
            self._data.setdefault("body", dict())[name] = body
            pose = C.Pose.identity()
            name = None
        else:
            body = self._st["body"][-1]

        assert type == "sphere"
        shape = C.ShapeInfo.from_type(type, radius=float(size))

        data = self._data["site"]
        assert name not in data
        data[name] = C.MJCFVisual(name, pose, shape, material=C.RGBA(*vector4f(rgba)))
        body.visuals.append(data[name])

        return check_done(node)

    def geom(self, node: XMLNode):
        st = self._st["default"]
        if len(st):
            st[-1]["geom"].update(node.attributes)
            assert len(node.children) == 0
            return

        cls = node.attributes.pop("class", None)
        if cls is None and len(self._st["class"]):
            cls = self._st["class"][-1]
        if cls is not None:
            for k, v in self._data["defaults"][cls]["geom"].items():
                node.attributes.setdefault(k, v)

        name = node.attributes.pop("name", None)
        type = node.attributes.pop("type", "plane")
        pose = self._find_pose(node)

        shape: C.ShapeInfo
        if type == "box":
            size = node.attributes.pop("size", "0 0 0")
            shape = C.BoxShapeInfo(vector3f(size))
        elif type == "mesh":
            size = node.attributes.pop("size", None)
            mesh = node.attributes.pop("mesh")
            filename = self._data["mesh"][mesh]["filename"]
            if size is None:
                size = self._data["mesh"][mesh]["scale"]
            shape = C.MeshShapeInfo(filename, vector3f(size))
        elif type == "plane":
            size = node.attributes.pop("size", "0 0 0")
            shape = C.PlaneShapeInfo(
                *vector3f(size)[:2]
            )  # uses only the half width and the half height
        elif type == "sphere":
            size = node.attributes.pop("size", "0")
            shape = C.SphereShapeInfo(float(size))
        elif type == "cylinder":
            r, hh = node.attributes.pop("size", "0 0").split()
            shape = C.CylinderShapeInfo(float(r), float(hh))
        elif type == "capsule":
            r, hh = node.attributes.pop("size", "0 0").split()
            shape = C.CapsuleShapeInfo(float(r), float(hh))
        else:
            raise ValueError("Unknown type: {}.".format(type))

        if len(self._st["body"]) == 0:
            assert name is not None
            body = C.Link(name, None, pose)
            self._data.setdefault("body", dict())[name] = body
            pose = C.Pose.identity()
            name = None
        else:
            body = self._st["body"][-1]

        material = node.attributes.pop("material", None)
        rgba = node.attributes.pop("rgba", "0.5 0.5 0.5 1")

        visual: C.Material
        if material is not None:
            visual = C.MJCFMaterial(material)
        else:
            visual = C.RGBA(*vector4f(rgba))

        # TODO(Jiayuan Mao @ 03/23): fix this.
        node.attributes.pop("mass", None)
        node.attributes.pop("density", None)
        node.attributes.pop("friction", None)
        node.attributes.pop("margin", None)

        node.attributes.pop("solimp", None)
        node.attributes.pop("solref", None)
        node.attributes.pop("user", None)

        inertial_group = node.attributes.pop("group", 0)
        contact_type = node.attributes.pop("contype", 0)
        contact_affinity = node.attributes.pop("conaffinity", 0)
        contact_dim = node.attributes.pop("condim", 3)  # frictional

        visual_geom = C.MJCFVisual(
            name,
            pose,
            shape=shape,
            material=visual,
        )
        body.visuals.append(visual_geom)
        collision_geom = C.MJCFCollision(
            name,
            pose,
            shape=shape,
            inertial_group=inertial_group,
            contact_type=contact_type,
            contact_affinity=contact_affinity,
            contact_dim=contact_dim,
        )
        body.collisions.append(collision_geom)
        if name is not None:
            self._data["visual"][name] = visual_geom
            self._data["collision"][name] = collision_geom

        return check_done(node)

    def joint(self, node: XMLNode):
        st = self._st["default"]
        if len(st):
            st[-1]["joint"].update(node.attributes)
            assert len(node.children) == 0
            return

        cls = node.attributes.pop("class", None)
        if cls is None and len(self._st["class"]):
            cls = self._st["class"][-1]
        if cls is not None:
            for k, v in self._data["defaults"][cls]["joint"].items():
                node.attributes.setdefault(k, v)

        name = node.attributes.pop("name", None)
        type = node.attributes.pop("type", "hinge")
        pose = self._find_pose(node)

        axis = node.attributes.pop("axis", "0 0 1")
        limited = bool_string(node.attributes.pop("limited", "false"))
        range = vector2f(node.attributes.pop("range", "0 0"))
        damping = node.attributes.pop("damping", 0)
        armature = node.attributes.pop("armature", 0)

        dynamics = C.JointDynamics(damping=damping, armature=armature)
        limit = C.JointLimit(lower=range[0], upper=range[1])
        control = self._find_control(node)

        if type == "slide":
            type = "prismatic"
        elif type == "hinge":
            if limited:
                type = "revolute"
            else:
                type = "continuous"

        joint = C.Joint(
            name,
            "",
            "",
            pose,
            C.JointInfo.from_type(
                type,
                axis=vector3f(axis),
                limit=limit,
                dynamics=dynamics,
            ),
            control,
        )
        if name is not None:
            self._data["joint"][name] = joint

        body_child = self._st["body"][-1]
        body_parent = self._st["body"][-2]
        joint.parent = body_parent.name
        joint.child = body_child.name
        return check_done(node)

    def freejoint(self, node: XMLNode):
        name = node.attributes.pop("name", None)
        type = "free"
        control = self._find_control(node)

        joint = C.Joint(
            name, "", "", C.Pose.identity(), C.JointInfo.from_type(type), control
        )
        if name is not None:
            self._data["joint"][name] = joint

        body_child = self._st["body"][-1]
        body_parent = self._st["body"][-2]
        joint.parent = body_parent.name
        joint.child = body_child.name
        return check_done(node)

    def worldbody(self, node: XMLNode):
        if len(node.children) == 0:
            return check_done(node)
        return node

    def actuator(self, node: XMLNode):
        return check_done(node)

    def position(self, node: XMLNode):
        st = self._st["default"]
        if len(st):
            st[-1]["joint"].update(node.attributes)
            assert len(node.children) == 0
            return

        cls = node.attributes.pop("class", None)
        if cls is None and len(self._st["class"]):
            cls = self._st["class"][-1]
        if cls is not None:
            for k, v in self._data["defaults"][cls]["position"].items():
                node.attributes.setdefault(k, v)

        joint = node.attributes.pop("joint")
        kp = node.attributes.pop("kp")
        control = self._find_control(node)

        # TODO(Jiayuan Mao @ 03/23): fix this.
        node.attributes.pop("user", None)

        data = self._data["actuator"]
        assert joint not in data
        # TODO(Jiayuan Mao @ 03/23): fix this.
        self._data["actuator"][joint] = (joint, float(kp), control)
        return check_done(node)

    def _find_rotation(self, node: XMLNode):
        found = set()
        for name in ["quat", "axisangle", "euler", "xyzaxes", "zaxis"]:
            if name in node.attributes:
                found.add(name)
        assert len(found) <= 1
        if len(found) == 0:
            return np.array([1, 0, 0, 0], dtype="float32")
        name = tuple(found)[0]
        if name == "quat":
            return vector4f(node.attributes.pop(name))
        elif name == "euler":
            return wxyz_from_euler(node.attributes.pop(name))
        else:
            raise NotImplementedError()

    def _find_pose(self, node: XMLNode):
        return C.Pose(
            pos=node.attributes.pop("pos", "0 0 0"), quat_wxyz=self._find_rotation(node)
        )

    def _find_control(self, node: XMLNode):
        limited = bool_string(node.attributes.pop("ctrllimited", "false"))
        range = node.attributes.pop("ctrlrange", "0 0")
        if limited:
            return C.JointControlInfo(lower=range[0], upper=range[1])
        return None

    def as_model(self):
        model = C.MJCFModel("mjcf_model")
        for link in self._data["body"].values():
            model.links.append(link)
        for joint in self._data["joint"].values():
            model.joints.append(joint)
        return model


def load_mjcf(filename: str, flatten_only: bool = False) -> C.MJCFModel:
    visitor: XMLVisitor

    if flatten_only:
        visitor = MJCFVisitorFlatten()
        return visitor.load_file(filename)

    visitor = MJCFVisitor()
    visitor.load_file(filename)
    return visitor.as_model()
