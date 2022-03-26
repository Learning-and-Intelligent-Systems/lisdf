import lisdf.components as C
from lisdf.parsing.string_utils import (
    bool_string,
    vector2f,
    vector3f,
    vector3f_or_float,
    vector4f,
    vector6f,
)
from lisdf.parsing.xml_j.visitor import XMLVisitor, check_done_decorator


class SDFVisitor(XMLVisitor):
    def include(self, node):
        uri = node.pop("uri", required=True)

        if uri.endswith(".sdf"):
            content = self.load_file(self._resolve_path(uri))
            scale_f, scale = vector3f_or_float(node.pop("scale", default="1"))
            assert content.model is not None  # does not allow worlds definitions.
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
            scale_f, scale = vector3f_or_float(node.pop("scale", default="1"))
            return node.set_data(
                C.URDFInclude(
                    name=node.attributes.pop("name", None),
                    uri=node.attributes.pop("uri", None),
                    scale=scale,
                    pose=node.pop(
                        "pose", return_type="data", default=C.Pose.identity()
                    ),
                    static=bool_string(node.pop("static", default="false")),
                    _scale1d=scale_f,
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
            C.Surface(
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
        return node.set_data(
            C.Geom(
                name=node.attributes.pop("name", None),
                pose=node.pop("pose", return_type="data", default=None),
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
        cast_shadows = bool_string(node.pop("cast_shadows", default="true"))
        return node.set_data(
            C.SDFGeom(
                name=node.attributes.pop("name", None),
                pose=node.pop("pose", return_type="data", default=None),
                shape=node.pop("geometry", return_type="data", required=True),
                visual=node.pop(
                    "material",
                    return_type="data",
                    default=C.PhongMaterial(),
                ),
                cast_shadows=cast_shadows,
            )
        )

    @check_done_decorator
    def link(self, node):
        name = node.attributes.pop("name")
        pose = node.pop("pose", return_type="data", default=C.Pose.identity())
        inertial = node.pop("inertial", return_type="data", default=C.Inertial.zeros())
        self_collide = bool_string(node.pop("self_collide", default="true"))
        # TODO(Jiayuan Mao @ 03/24: handle parent.
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
                link.collisions.append(c.data)
            elif c.tag == "sensor":
                link.sensors.append(c.data)
            else:
                raise TypeError("Unknown tag: {}.".format(c.tag))
        return node.set_data(link)

    @check_done_decorator
    def joint(self, node):
        type = node.attributes.pop("type", "continuous")

        # TODO(Jiayuan Mao @ 03/24: implement dynamics control information.
        if type == "fixed":
            joint_info = C.FixedJointInfo()
        elif type == "continuous":
            joint_info = C.HingeJointInfo(
                vector3f(
                    node.pop("axis", required=True, return_type="node").pop(
                        "xyz",
                        required=True,
                        return_type="text",
                    )
                )
            )
        elif type == "revolute":
            # TODO(Jiayuan Mao @ 03/24: store joint limits.
            joint_info = C.HingeJointInfo(
                vector3f(
                    node.pop("axis", required=True, return_type="node").pop(
                        "xyz",
                        required=True,
                        return_type="text",
                    )
                )
            )
        elif type == "prismatic":
            joint_info = C.PrismaticJointInfo(
                vector3f(
                    node.pop("axis", required=True, return_type="node").pop(
                        "xyz",
                        required=True,
                        return_type="text",
                    )
                )
            )
        else:
            raise NotImplementedError("Unknown joint type: {}.".format(type))

        joint = C.Joint(
            name=node.attributes.pop("name", None),
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
            name = node.attributes.pop("name", None)
            pose = node.pop("pose", return_type="data", default=C.Pose.identity())
            static = bool_string(node.pop("static", default="false"))
            model = C.Model(name=name, pose=pose, parent=None, static=static)
            for c in node.pop_all_children():
                if c.tag == "joint":
                    model.joints.append(c.data)
                elif c.tag == "link":
                    model.links.append(c.data)
                else:
                    raise NotImplementedError(
                        "Unknown tag for model: {}.".format(c.tag)
                    )
            return node.set_data(model)

    @check_done_decorator
    def state_joint(self, node):
        name = node.attributes.pop("name", None)
        state = C.JointState(name)
        for c in node.pop_all_children():
            assert c.tag == "angle"
            state.axis_states.append(
                C.JointAxisState(axis=c.attributes.pop("axis", 0), value=float(c.text))
            )
        return node.set_data(state)

    @check_done_decorator
    def state_link(self, node):
        name = node.attributes.pop("name", None)
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
        state = C.WorldState(node.attributes.pop("world_name"))
        for c in node.pop_all_children():
            if c.tag == "model":
                state.model_states.append(c.data)
            else:
                raise NotImplementedError(
                    "Unknown tag for world states: {}.".format(c.tag)
                )
        return node.set_data(state)

    @check_done_decorator
    def world(self, node):
        name = node.attributes.pop("name", None)
        static = bool_string(node.pop("static", default="false"))
        world = C.World(name, static)
        for c in node.pop_all_children():
            if c.tag == "model":
                world.models.append(c.data)
            elif c.tag == "include":
                world.models.append(c.data)
            elif c.tag == "state":
                world.states.append(c.data)
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
        return node.set_data(lisdf)


def load_sdf(filename: str) -> C.LISDF:
    visitor = SDFVisitor()
    return visitor.load_file(filename).data
