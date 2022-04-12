import lisdf.components as C
from lisdf.parsing.srdf import SRDFParserMixin
from lisdf.parsing.string_utils import (
    bool_string,
    safe_float,
    vector2f,
    vector3f,
    vector3f_or_float,
    vector4f,
    vector6f,
)
from lisdf.parsing.urdf_j import load_urdf
from lisdf.parsing.xml_j.visitor import XMLVisitor, check_done_decorator


class SDFVisitor(XMLVisitor, SRDFParserMixin):
    DEFAULT_WORLD_NAME = "default_world"
    DEFAULT_MODEL_NAME = "default_model"

    def include(self, node):
        uri = node.pop("uri", required=True)

        if uri.endswith(".sdf"):
            content = self.load_file(self._resolve_path(uri))
            scale_f, scale = vector3f_or_float(node.pop("scale", default="1"))
            # does not allow worlds definitions.
            assert content.model is not None and len(content.worlds) == 0
            return node.set_data(
                C.SDFInclude(
                    name=node.attributes.pop("name", None),
                    uri=node.attributes.pop("uri", None),
                    scale=scale,
                    pose=node.pop(
                        "pose", return_type="data", default=C.Pose.identity()
                    ),
                    static=bool_string(node.pop("static", default="false")),
                    _scale1d=scale_f,
                    _content=content,
                )
            )
        elif uri.endswith(".urdf"):
            content = load_urdf(self._resolve_path(uri))
            scale_f, scale = vector3f_or_float(node.pop("scale", default="1"))
            return node.set_data(
                C.URDFInclude(
                    name=node.attributes.pop("name", None),
                    uri=uri,
                    scale=scale,
                    pose=node.pop(
                        "pose", return_type="data", default=C.Pose.identity()
                    ),
                    static=bool_string(node.pop("static", default="false")),
                    _scale1d=scale_f,
                    _content=content,
                )
            )
        else:
            raise ValueError(f"Unknown include type: {uri}")

    @check_done_decorator
    def pose(self, node):
        return node.set_data(C.Pose.from_rpy_6d(vector6f(node.text)))

    @check_done_decorator
    def inertia(self, node):
        return node.set_data(
            C.Inertia(
                float(node.pop("ixx", default=0)),
                float(node.pop("ixy", default=0)),
                float(node.pop("ixz", default=0)),
                float(node.pop("iyy", default=0)),
                float(node.pop("iyz", default=0)),
                float(node.pop("izz", default=0)),
            )
        )

    @check_done_decorator
    def inertial(self, node):
        return node.set_data(
            C.Inertial(
                float(node.pop("mass", default=0)),
                node.pop("pose", return_type="data", default=C.Pose.identity()),
                node.pop("inertia", return_type="data", default=C.Inertia.zeros()),
            )
        )

    @check_done_decorator
    def surface_contact(self, node):
        return node.set_data(
            C.SDFSurfaceContact(
                collide_bitmask=int(
                    node.pop("collide_bitmask", default="0xffff"),
                    0,
                ),
                collide_without_contact=bool_string(
                    node.pop("collide_without_contact", default="false")
                ),
            )
        )

    @check_done_decorator
    def surface_friction(self, node):
        # TODO(Jiayuan Mao @ 03/24: handle other types of friction notations.
        ode_node = node.pop("ode", required=True, return_type="node")
        if ode_node is not None:
            return node.set_data(
                C.SDFSurfaceFriction(
                    ode_mu=float(ode_node.pop("mu", default=1)),
                    ode_mu2=float(ode_node.pop("mu2", default=1)),
                )
            )

    def surface_init(self, node):
        self.enter_scope("surface")

    @check_done_decorator
    def surface(self, node):
        self.exit_scope("surface")
        return node.set_data(
            C.SurfaceInfo(
                node.pop("contact", return_type="data", default=C.SurfaceContact()),
                node.pop("friction", return_type="data", default=C.SurfaceFriction()),
            )
        )

    @check_done_decorator
    def geometry(self, node):
        assert len(node.children) == 1
        c = node.children.pop()
        if c.tag == "box":
            return node.set_data(C.BoxShapeInfo(vector3f(c.pop("size", required=True))))
        elif c.tag == "sphere":
            return node.set_data(
                C.SphereShapeInfo(float(c.pop("radius", required=True)))
            )
        elif c.tag == "cylinder":
            return node.set_data(
                C.CylinderShapeInfo(
                    float(c.pop("radius", required=True)),
                    float(c.pop("length", required=True)) / 2,
                )
            )
        elif c.tag == "capsule":
            return node.set_data(
                C.CapsuleShapeInfo(
                    float(c.pop("radius", required=True)),
                    float(c.pop("length", required=True)) / 2,
                )
            )
        elif c.tag == "plane":
            size = vector2f(c.pop("size", required=True))
            return node.set_data(
                C.PlaneShapeInfo(
                    half_width=size[0] / 2,
                    half_height=size[1] / 2,
                    normal=vector3f(c.pop("normal", default="0 0 1")),
                )
            )
        elif c.tag == "mesh":
            return node.set_data(
                C.MeshShapeInfo(
                    self._resolve_path(c.pop("uri", required=True)),
                    vector3f(c.pop("scale", default="1 1 1")),
                )
            )
        else:
            raise NotImplementedError("Unknown geometry tag: {}.".format(c.tag))

    @check_done_decorator
    def collision(self, node):
        node.attributes.pop("group", None)  # TODO: Figure out what this is.
        link_collision_name = self._st["link_collision"][-1]
        default_name = f"{link_collision_name[0]}_collision_{link_collision_name[1]}"
        return node.set_data(
            C.Collision(
                name=node.attributes.pop("name", default_name),
                pose=node.pop("pose", return_type="data", default=C.Pose.identity()),
                shape=node.pop("geometry", return_type="data", required=True),
                surface=node.pop("surface", return_type="data", default=None),
            )
        )

    @check_done_decorator
    def material(self, node):
        # TODO(Jiayuan Mao @ 03/24: store the script somewhere.
        node.pop("script")

        return node.set_data(
            C.PhongMaterial(
                vector4f(node.pop("ambient", default="1 1 1 1")),
                vector4f(node.pop("diffuse", default="1 1 1 1")),
                vector4f(node.pop("specular", default="0 0 0 1")),
                vector4f(node.pop("emissive", default="0 0 0 1")),
            )
        )

    @check_done_decorator
    def visual(self, node):
        link_visual_name = self._st["link_visual"][-1]
        default_name = f"{link_visual_name[0]}_visual_{link_visual_name[1]}"
        self._st["link_visual"][-1] = (link_visual_name[0], link_visual_name[1] + 1)
        cast_shadows = bool_string(node.pop("cast_shadows", default="true"))
        return node.set_data(
            C.SDFVisual(
                name=node.attributes.pop("name", default_name),
                pose=node.pop("pose", return_type="data", default=C.Pose.identity()),
                shape=node.pop("geometry", return_type="data", required=True),
                material=node.pop(
                    "material",
                    return_type="data",
                    default=C.PhongMaterial(),
                ),
                cast_shadows=cast_shadows,
            )
        )

    def link_init(self, node):
        self._st["link_visual"].append((node.attributes["name"], 0))
        self._st["link_collision"].append((node.attributes["name"], 0))

    @check_done_decorator
    def link(self, node):
        self._st["link_visual"].pop()
        self._st["link_collision"].pop()
        name = node.attributes.pop("name")
        pose = node.pop("pose", return_type="data", default=None)
        inertial = node.pop("inertial", return_type="data", default=C.Inertial.zeros())
        self_collide = bool_string(node.pop("self_collide", default="true"))

        # TODO(Jiayuan Mao @ 03/24): handle parent.
        link = C.SDFLink(
            name=name,
            parent=None,
            pose=pose,
            inertial=inertial,
            self_collide=self_collide,
        )
        for c in node.pop_all_children():
            if c.tag == "collision":
                link.collisions.append(c.data)
            elif c.tag == "visual":
                link.visuals.append(c.data)
            elif c.tag == "sensor":
                link.sensors.append(c.data)
            else:
                raise TypeError("Unknown tag: {}.".format(c.tag))
        return node.set_data(link)

    @check_done_decorator
    def joint(self, node):
        type = node.attributes.pop("type", "continuous")

        if type == "fixed":
            joint_info = C.FixedJointInfo()
            node.pop("axis")  # TODO: check the axis parameter in fixed joints.
            for c in ["limit", "dynamics"]:
                node.pop(c)
        elif type in ("continuous", "revolute", "prismatic"):
            axis_node = node.pop("axis", return_type="node", required=True)

            axis = vector3f(axis_node.pop("xyz", default="0 0 1"))
            limit = None
            limit_node = axis_node.pop("limit", return_type="node", default=None)
            if limit_node is not None:
                limit = C.JointLimit(
                    lower=safe_float(limit_node.pop("lower", default=None)),
                    upper=safe_float(limit_node.pop("upper", default=None)),
                    effort=safe_float(limit_node.pop("effort", default=None)),
                    velocity=safe_float(limit_node.pop("velocity", default=None)),
                )

            dynamics = None
            dynamics_node = axis_node.pop("dynamics", return_type="node", default=None)
            if dynamics_node is not None:
                dynamics = C.JointDynamics(
                    damping=float(dynamics_node.pop("damping", default="0")),
                    friction=float(dynamics_node.pop("friction", default="0")),
                )

            if type == "continuous":
                joint_info = C.ContinuousJointInfo(axis, limit=limit, dynamics=dynamics)
            elif type == "revolute":
                joint_info = C.RevoluteJointInfo(axis, limit=limit, dynamics=dynamics)
            elif type == "prismatic":
                joint_info = C.PrismaticJointInfo(axis, limit=limit, dynamics=dynamics)
            else:
                raise NotImplementedError("Unknown joint type: {}.".format(type))
        else:
            raise NotImplementedError("Unknown joint type: {}.".format(type))

        joint = C.Joint(
            name=node.attributes.pop("name"),
            parent=node.pop("parent", required=True),
            child=node.pop("child", required=True),
            pose=node.pop("pose", return_type="data", default=C.Pose.identity()),
            joint_info=joint_info,
        )
        return node.set_data(joint)

    @check_done_decorator
    def sensor_contact(self, node):
        # TODO(Jiayuan Mao @ 03/24): implement sensor information.
        node.pop_all_children()
        return node

    def sensor_init(self, node):
        self.enter_scope("sensor")

    @check_done_decorator
    def sensor(self, node):
        self.exit_scope("sensor")
        name = node.attributes.pop("name")
        type = node.attributes.pop("type")
        if type == "camera":
            return node.set_data(C.Sensor.from_type(type, name=name))
        else:
            # TODO(Jiayuan Mao @ 03/24: implement other types of sensors.
            node.pop_all_children()
            return node

    @check_done_decorator
    def model(self, node):
        st = self._st["state"]
        if len(st) > 0:
            pass
        else:
            name = node.attributes.pop("name", type(self).DEFAULT_MODEL_NAME)
            pose = node.pop("pose", return_type="data", default=None)
            static = bool_string(node.pop("static", default="false"))
            model = C.Model(name=name, pose=pose, parent=None, static=static)
            for c in node.pop_all_children():
                if c.tag == "joint":
                    model.joints.append(c.data)
                elif c.tag == "link":
                    model.links.append(c.data)
                elif c.tag == "group":
                    model.groups.append(c.data)
                elif c.tag == "disable_collisions":
                    model.disable_collisions.append(c.data)
                else:
                    raise NotImplementedError(
                        "Unknown tag for model: {}.".format(c.tag)
                    )
            return node.set_data(model)

    @check_done_decorator
    def state_joint(self, node):
        name = node.attributes.pop("name")
        state = C.JointState(name)
        for c in node.pop_all_children():
            assert c.tag == "angle"
            state.axis_states.append(
                C.JointAxisState(axis=c.attributes.pop("axis", 0), value=float(c.text))
            )
        return node.set_data(state)

    @check_done_decorator
    def state_link(self, node):
        name = node.attributes.pop("name")
        state = C.LinkState(
            name,
            pose=node.pop("pose", return_type="data", default=C.Pose.identity()),
        )
        return node.set_data(state)

    @check_done_decorator
    def state_model(self, node):
        state = C.ModelState(name=node.attributes.pop("name"), pose=node.pop("pose"))
        for c in node.pop_all_children():
            if c.tag == "joint":
                state.joint_states.append(c.data)
            elif c.tag == "link":
                state.link_states.append(c.data)
            else:
                raise NotImplementedError(
                    "Unknown tag for model state: {}.".format(c.tag)
                )
        return node.set_data(state)

    def state_init(self, node):
        self.enter_scope("state")

    @check_done_decorator
    def state(self, node):
        self.exit_scope("state")
        state = C.WorldState(
            node.attributes.pop("world_name", type(self).DEFAULT_WORLD_NAME)
        )
        for c in node.pop_all_children():
            if c.tag == "model":
                state.model_states.append(c.data)
            else:
                raise NotImplementedError(
                    "Unknown tag for world states: {}.".format(c.tag)
                )
        return node.set_data(state)

    @check_done_decorator
    def gui_camera(self, node):
        name = node.attributes.pop("name", None)
        projection_type = node.pop("projection_type", default="perspective")
        kwargs = dict(name=name, projection_type=projection_type)

        definition = node.attributes.pop("definition_type", "pose")
        if definition == "pose":
            pose = node.pop("pose", return_type="data", required=True)
            camera = C.GUICamera(pose=pose, **kwargs)
        elif definition == "lookat":
            xyz = vector3f(node.pop("xyz", required=True))
            point_to = vector3f(node.pop("point_to", required=True))
            camera = C.GUICamera.from_lookat(xyz=xyz, point_to=point_to, **kwargs)
        else:
            raise NotImplementedError(
                "Unknown camera definition type: {}".format(definition)
            )

        return node.set_data(camera)

    def gui_init(self, node):
        self.enter_scope("gui")

    @check_done_decorator
    def gui(self, node):
        self.exit_scope("gui")
        gui = C.GUI(
            camera=node.pop("camera", return_type="data", required=True),
        )
        return node.set_data(gui)

    @check_done_decorator
    def world(self, node):
        name = node.attributes.pop("name", type(self).DEFAULT_WORLD_NAME)
        static = bool_string(node.pop("static", default="false"))
        world = C.World(name, static)
        for c in node.pop_all_children():
            if c.tag == "model":
                world.models.append(c.data)
            elif c.tag == "include":
                world.models.append(c.data)
            elif c.tag == "state":
                world.states.append(c.data)
            elif c.tag == "gui":
                assert world.gui is None
                world.gui = c.data
            else:
                raise NotImplementedError("Unknown tag for world: {}.".format(c.tag))
        return node.set_data(world)

    @check_done_decorator
    def sdf(self, node):
        sdf_version = node.attributes.pop("version")
        lisdf = C.LISDF(sdf_version)
        for c in node.pop_all_children():
            if c.tag == "world":
                lisdf.worlds.append(c.data)
            elif c.tag == "model":
                assert lisdf.model is None
                lisdf.model = c.data
            elif c.tag == "include":
                assert lisdf.model is None
                lisdf.model = c.data
            else:
                raise NotImplementedError("Unknown tag for sdf: {}.".format(c.tag))
        lisdf.build_lookup_tables()
        return node.set_data(lisdf)


def load_sdf(filename: str, verbose: bool = False) -> C.LISDF:
    visitor = SDFVisitor()
    visitor.set_verbose(verbose)
    return visitor.load_file(filename).data


def load_sdf_string(string: str, verbose: bool = False) -> C.LISDF:
    visitor = SDFVisitor()
    visitor.set_verbose(verbose)
    return visitor.load_string(string).data
