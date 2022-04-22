import warnings

import lisdf.components as C
from lisdf.parsing.srdf import SRDFParserMixin
from lisdf.parsing.string_utils import safe_float, vector2f, vector3f, vector4f
from lisdf.parsing.xml_j.visitor import XMLVisitor, check_done_decorator


class URDFVisitor(XMLVisitor, SRDFParserMixin):
    def _parse_filename(self, filename: str) -> str:
        if filename.startswith("package://"):
            return filename
        elif filename.startswith("file://"):
            return self._resolve_path(filename[len("file://") :])
        else:
            return self._resolve_path(filename)

    @check_done_decorator
    def origin(self, node):
        return node.set_data(
            C.Pose.from_rpy(
                vector3f(node.attributes.pop("xyz", "0 0 0")),
                vector3f(node.attributes.pop("rpy", "0 0 0")),
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
                    self._parse_filename(c.attributes.pop("filename")),
                    vector3f(c.attributes.pop("scale", "1 1 1")),
                )
            )
        else:
            raise NotImplementedError("Unknown geometry tag: {}.".format(c.tag))

    @check_done_decorator
    def collision(self, node):
        node.attributes.pop("group", None)  # TODO: Figure out what this is.
        link_collision_name = self._st["link_collision"][-1]
        default_name = f"{link_collision_name[0]}_collision_{link_collision_name[1]}"
        self._st["link_collision"][-1] = (
            link_collision_name[0],
            link_collision_name[1] + 1,
        )
        return node.set_data(
            C.Collision(
                name=node.attributes.pop("name", default_name),
                pose=node.pop("origin", return_type="data", default=None),
                shape=node.pop("geometry", return_type="data", required=True),
            )
        )

    @check_done_decorator
    def visual(self, node):
        link_visual_name = self._st["link_visual"][-1]
        default_name = f"{link_visual_name[0]}_visual_{link_visual_name[1]}"
        self._st["link_visual"][-1] = (link_visual_name[0], link_visual_name[1] + 1)
        return node.set_data(
            C.Visual(
                name=node.attributes.pop("name", default_name),
                pose=node.pop("origin", return_type="data", default=None),
                shape=node.pop("geometry", return_type="data", required=True),
                material=node.pop("material", return_type="data", default=None),
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
                filename=self._parse_filename(node.attributes.pop("filename")),
            )
        )

    @check_done_decorator
    def material(self, node):
        if len(node.children) == 0:
            material_name = node.attributes.pop("name")
            assert material_name in self._data["materials"]
            material = self._data["materials"][material_name]
            return node.set_data(material)

        assert len(node.children) == 1
        children = list(node.pop_all_children())
        self._data["materials"][node.attributes.pop("name")] = children[0].data
        return None

    def link_init(self, node):
        self._st["link_visual"].append((node.attributes["name"], 0))
        self._st["link_collision"].append((node.attributes["name"], 0))

    @check_done_decorator
    def link(self, node):
        self._st["link_visual"].pop()
        self._st["link_collision"].pop()
        node.attributes.pop("type", None)  # TODO: warning unused type.
        # TODO: URDF doesn't support pose actually...
        pose = node.pop("origin", return_type="data", default=None)
        link = C.Link(
            name=node.attributes.pop("name"),
            parent=None,
            pose=pose,
            inertial=node.pop("inertial", return_type="data", default=None),
        )
        for c in node.pop_all_children():
            if c.tag == "collision":
                link.collisions.append(c.data)
            elif c.tag == "visual":
                link.visuals.append(c.data)
            else:
                raise TypeError("Unknown tag: {}.".format(c.tag))
        return node.set_data(link)

    @check_done_decorator
    def joint(self, node):
        name = node.attributes.pop("name")
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
            for c in ["limit", "dynamics", "calibration", "mimic", "safety_controller"]:
                node.pop(c)
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
                lower=safe_float(limit_node.attributes.pop("lower", None)),
                upper=safe_float(limit_node.attributes.pop("upper", None)),
                effort=safe_float(limit_node.attributes.pop("effort", None)),
                velocity=safe_float(limit_node.attributes.pop("velocity", None)),
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
                lower=safe_float(control_node.attributes.pop("soft_lower_limit", None)),
                upper=safe_float(control_node.attributes.pop("soft_upper_limit", None)),
                position=safe_float(control_node.attributes.pop("k_position", None)),
                velocity=safe_float(control_node.attributes.pop("k_velocity", None)),
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
        warnings.warn("Transmission properties in URDF has not been implemented.")
        return "skip"  # TODO: implement transmission.

    def gazebo_init(self, node):
        warnings.warn("Gazebo properties in URDF has not been implemented.")
        return "skip"

    def verbose_init(self, node):
        # This is not a official part of URDF specification.
        return "skip"

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
            elif c.tag == "group":
                model.groups.append(c.data)
            elif c.tag == "disable_collisions":
                model.disable_collisions.append(c.data)
            else:
                raise TypeError("Unknown tag: {}.".format(c.tag))

        return node.set_data(model)


def load_urdf(filename: str, verbose: bool = False) -> C.URDFModel:
    visitor = URDFVisitor()
    visitor.set_verbose(verbose)
    return visitor.load_file(filename).data


def load_urdf_string(string: str, verbose: bool = False) -> C.URDFModel:
    visitor = URDFVisitor()
    visitor.set_verbose(verbose)
    return visitor.load_string(string).data
