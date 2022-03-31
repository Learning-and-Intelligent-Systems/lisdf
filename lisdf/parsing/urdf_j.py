import lisdf.components as C
from lisdf.parsing.string_utils import vector2f, vector3f, vector4f
from lisdf.parsing.xml_j.visitor import XMLVisitor, check_done_decorator


class URDFVisitor(XMLVisitor):
    @check_done_decorator
    def origin(self, node):
        return node.set_data(
            C.Pose.from_rpy(
                vector3f(node.attributes.pop("xyz", "0 0 0")),
                vector3f(node.attributes.pop("rpy", "0 0 0")),
            )
        )

    @check_done_decorator
    def geometry(self, node):
        assert len(node.children) == 1
        c = node.children.pop()
        if c.tag == "box":
            return node.set_data(C.BoxShapeInfo(vector3f(c.attributes.pop("size"))))
        elif c.tag == "sphere":
            return node.set_data(C.SphereShapeInfo(float(c.attributes.pop("radius"))))
        elif c.tag == "cylinder":
            return node.set_data(
                C.CylinderShapeInfo(
                    float(c.attributes.pop("radius")),
                    float(c.attributes.pop("length")) / 2,
                )
            )
        elif c.tag == "capsule":
            return node.set_data(
                C.CapsuleShapeInfo(
                    float(c.attributes.pop("radius")),
                    float(c.attributes.pop("length")) / 2,
                )
            )
        elif c.tag == "plane":
            size = vector2f(c.attributes.pop("size"))
            return node.set_data(
                C.PlaneShapeInfo(
                    half_width=size[0] / 2,
                    half_height=size[1] / 2,
                    normal=vector3f(c.attributes.pop("normal", "0 0 1")),
                )
            )
        elif c.tag == "mesh":
            return node.set_data(
                C.MeshShapeInfo(
                    self._resolve_path(c.attributes.pop("filename")),
                    vector3f(c.attributes.pop("scale", "1 1 1")),
                )
            )
        else:
            raise NotImplementedError("Unknown geometry tag: {}.".format(c.tag))

    @check_done_decorator
    def collision(self, node):
        return node.set_data(
            C.Geom(
                name=node.attributes.pop("name", None),
                pose=node.pop("origin", return_type="data", default=None),
                shape=node.pop("geometry", return_type="data", required=True),
            )
        )

    @check_done_decorator
    def inertia(self, node):
        return node.set_data(
            C.Inertia(
                float(node.attributes.pop("ixx", 0)),
                float(node.attributes.pop("ixy", 0)),
                float(node.attributes.pop("ixz", 0)),
                float(node.attributes.pop("iyy", 0)),
                float(node.attributes.pop("iyz", 0)),
                float(node.attributes.pop("izz", 0)),
            )
        )

    @check_done_decorator
    def inertial(self, node):
        return node.set_data(
            C.Inertial(
                float(
                    node.pop("mass", default=0, return_type="node").attributes.pop(
                        "value"
                    )
                ),
                node.pop("origin", return_type="data", default=C.Pose.identity()),
                node.pop("inertia", return_type="data", default=C.Inertia.zeros()),
            )
        )

    def visual_init(self, node):
        self.enter_scope("visual")

    @check_done_decorator
    def visual(self, node):
        self.exit_scope("visual")
        return node.set_data(
            C.Geom(
                name=node.attributes.pop("name", None),
                pose=node.pop("origin", return_type="data", default=None),
                shape=node.pop("geometry", return_type="data", required=True),
                visual=node.pop("material", return_type="data", default=None),
            )
        )

    @check_done_decorator
    def color(self, node):
        string = node.attributes.pop("rgba", "0 0 0 1").strip()
        if string.count(" ") == 0:
            rgba = (float(string),) * 3 + (1.0,)
        elif string.count(" ") == 2:
            rgba = tuple(vector3f(string)) + (1.0,)
        elif string.count(" ") == 3:
            rgba = tuple(vector4f(string))
        else:
            raise ValueError("Invalid color string: {}".format(string))
        return node.set_data(C.RGBA(*rgba))

    @check_done_decorator
    def texture(self, node):
        return node.set_data(
            C.Texture(
                filename=self._resolve_path(node.attributes.pop("filename")),
            )
        )

    def visual_material(self, node):
        material_name = node.attributes.pop("name")
        assert material_name in self._data["materials"]
        material = self._data["materials"][material_name]
        return node.set_data(material)

    @check_done_decorator
    def material(self, node):
        assert len(node.children) == 1
        children = list(node.pop_all_children())
        self._data["materials"][node.attributes.pop("name")] = children[0]
        return None

    @check_done_decorator
    def link(self, node):
        node.attributes.pop("type", None)  # TODO: warning unused type.
        link = C.Link(
            name=node.attributes.pop("name", None),
            parent=None,
            pose=node.pop("origin", return_type="data", default=None),
            inertial=node.pop("inertial", return_type="data", default=None),
        )
        for c in node.pop_all_children():
            if c.tag == "collision":
                link.collisions.append(c.data)
            elif c.tag == "visual":
                link.collisions.append(c.data)
            else:
                raise TypeError("Unknown tag: {}.".format(c.tag))
        return node.set_data(link)

    @check_done_decorator
    def joint(self, node):
        name = node.attributes.pop("name", None)
        type = node.attributes.pop("type")
        pose = node.pop("origin", return_type="data", default=C.Pose.identity())
        parent = node.pop("parent", return_type="node", required=True).attributes.pop(
            "link"
        )
        child = node.pop("child", return_type="node", required=True).attributes.pop(
            "link"
        )

        if type == "fixed":
            node.pop("axis")  # TODO: check the axis parameter in fixed joints.
            return node.set_data(
                C.Joint(
                    name=name,
                    pose=pose,
                    parent=parent,
                    child=child,
                    joint_info=C.FixedJointInfo(),
                )
            )

        axis = vector3f(
            node.pop("axis", return_type="node", required=True).attributes.pop("xyz")
        )

        limit = None
        limit_node = node.pop("limit", return_type="node", default=None)
        if limit_node is not None:
            limit = C.JointLimit(
                lower=float(limit_node.attributes.pop("lower"))
                if "lower" in limit_node.attributes
                else None,
                upper=float(limit_node.attributes.pop("upper"))
                if "upper" in limit_node.attributes
                else None,
                effort=float(limit_node.attributes.pop("effort")),
                velocity=float(limit_node.attributes.pop("velocity")),
            )
        dynamics = None
        dynamics_node = node.pop("dynamics", return_type="node", default=None)
        if dynamics_node is not None:
            dynamics = C.JointDynamics(
                damping=float(dynamics_node.attributes.pop("damping", 0)),
                friction=float(dynamics_node.attributes.pop("friction", 0)),
            )
        calibration = None
        calibration_node = node.pop("calibration", return_type="node", default=None)
        if calibration_node is not None:
            calibration = C.JointCalibration(
                rising=float(calibration_node.attributes.pop("rising", 0)),
                falling=float(calibration_node.attributes.pop("falling", 0)),
            )
        control_info = None
        control_node = node.pop("safety_controller", return_type="node", default=None)
        if control_node is not None:
            control_info = C.JointControlInfo(
                lower=float(control_node.attributes.pop("soft_lower_limit"))
                if "soft_lower_limit" in control_node.attributes
                else None,
                upper=float(control_node.attributes.pop("soft_upper_limit"))
                if "soft_upper_limit" in control_node.attributes
                else None,
                position=float(control_node.attributes.pop("k_position"))
                if "k_position" in control_node.attributes
                else None,
                velocity=float(control_node.attributes.pop("k_velocity"))
                if "k_velocity" in control_node.attributes
                else None,
            )
        mimic_info = None
        mimic_node = node.pop("mimic", return_type="node", default=None)
        if mimic_node is not None:
            mimic_info = C.JointMimic(
                joint=mimic_node.attributes.pop("joint"),
                multiplier=float(mimic_node.attributes.pop("multiplier", 1)),
                offset=float(mimic_node.attributes.pop("offset", 0)),
            )

        return node.set_data(
            C.Joint(
                name=name,
                pose=pose,
                parent=parent,
                child=child,
                joint_info=C.JointInfo.from_type(
                    type,
                    axis=axis,
                    limit=limit,
                    dynamics=dynamics,
                    calibration=calibration,
                    mimic=mimic_info,
                ),
                control_info=control_info,
            )
        )

    def transmission_init(self, node):
        return "skip"  # TODO: implement transmission.

    def gazebo_init(self, node):
        return "skip"

    @check_done_decorator
    def robot(self, node):
        node.attributes.pop("version", None)
        model = C.URDFModel(
            name=node.attributes.pop("name", None),
        )

        for c in node.pop_all_children():
            if c.tag == "link":
                model.links.append(c.data)
            elif c.tag == "joint":
                model.joints.append(c.data)
            else:
                raise TypeError("Unknown tag: {}.".format(c.tag))
        return node.set_data(model)


def load_urdf(filename: str) -> C.URDFModel:
    visitor = URDFVisitor()
    return visitor.load_file(filename).data
