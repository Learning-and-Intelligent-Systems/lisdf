from lisdf.parsing.xml_reflection.basics import node_add, xml_children
from lisdf.parsing.xml_reflection.core import (
    AggregateElement,
    Attribute,
    Element,
    FactoryType,
    Object,
    ValueType,
    add_type,
    end_namespace,
    reflect,
    start_namespace,
)

start_namespace("sdf")

name_attribute = Attribute("name", str)
pose_element = Element("pose", "vector6", False)


class Entity(Object):
    def __init__(self, name=None, pose=None):
        self.name = name
        self.pose = pose


reflect(Entity, params=[name_attribute, pose_element])


class Inertia(Object):
    KEYS = ["ixx", "ixy", "ixz", "iyy", "iyz", "izz"]

    def __init__(self, ixx=0.0, ixy=0.0, ixz=0.0, iyy=0.0, iyz=0.0, izz=0.0):
        self.ixx = ixx
        self.ixy = ixy
        self.ixz = ixz
        self.iyy = iyy
        self.iyz = iyz
        self.izz = izz

    def to_matrix(self):
        return [
            [self.ixx, self.ixy, self.ixz],
            [self.ixy, self.iyy, self.iyz],
            [self.ixz, self.iyz, self.izz],
        ]


reflect(Inertia, params=[Element(key, float) for key in Inertia.KEYS])


class Inertial(Object):
    def __init__(self, mass=0.0, inertia=None, pose=None):
        self.mass = mass
        self.inertia = inertia
        self.pose = pose


reflect(
    Inertial,
    params=[
        Element("mass", float, required=False),
        Element("inertia", Inertia, required=False),
        pose_element,
    ],
)


class Link(Entity):
    def __init__(
        self, name=None, pose=None, visual=None, inertial=None, collision=None
    ):
        Entity.__init__(self, name, pose)
        self.aggregate_init()
        self.inertial = inertial
        self.visuals = []
        if visual:
            self.visual = visual
        self.collisions = []
        if collision:
            self.collision = collision

    def __get_visual(self):
        """Return the first visual or None."""
        if self.visuals:
            return self.visuals[0]

    def __set_visual(self, visual):
        """Set the first visual."""
        if self.visuals:
            self.visuals[0] = visual
        else:
            self.visuals.append(visual)
        if visual:
            self.add_aggregate("visual", visual)

    def __get_collision(self):
        """Return the first collision or None."""
        if self.collisions:
            return self.collisions[0]

    def __set_collision(self, collision):
        """Set the first collision."""
        if self.collisions:
            self.collisions[0] = collision
        else:
            self.collisions.append(collision)
        if collision:
            self.add_aggregate("collision", collision)

    # Properties for backwards compatibility
    visual = property(__get_visual, __set_visual)
    collision = property(__get_collision, __set_collision)


class Script(Object):
    def __init__(self, uri=None, name=None):
        self.uri = uri
        self.name = name


class Material(Object):
    def __init__(
        self,
        script=None,
        ambient=None,
        diffuse=None,
        specular=None,
        emissive=None,
        name=None,
    ):
        self.script = script
        self.ambient = ambient
        self.diffuse = diffuse
        self.specular = specular
        self.emissive = emissive
        self.name = name


reflect(
    Script,
    tag="script",
    params=[
        Element("name", str, required=False),
        Element("uri", str),
    ],
)


reflect(
    Material,
    tag="material",
    params=[
        Attribute("name", str, required=False),
        Element("script", Script, required=False),
        Element("ambient", "vector4", required=False),
        Element("diffuse", "vector4", required=False),
        Element("specular", "vector4", required=False),
        Element("emissive", "vector4", required=False),
    ],
)


class Box(Object):
    def __init__(self, size=None):
        self.size = size


reflect(Box, tag="box", params=[Element("size", "vector3")])


class Plane(Object):
    def __init__(self, normal=0.0, size=0.0):
        self.normal = normal
        self.size = size


reflect(
    Plane,
    tag="plane",
    params=[Element("normal", "vector3"), Element("size", "vector2")],
)


class Cylinder(Object):
    def __init__(self, radius=0.0, length=0.0):
        self.radius = radius
        self.length = length


reflect(
    Cylinder,
    tag="cylinder",
    params=[Element("radius", float), Element("length", float)],
)


class Sphere(Object):
    def __init__(self, radius=0.0):
        self.radius = radius


reflect(Sphere, tag="sphere", params=[Element("radius", float)])


class Mesh(Object):
    def __init__(self, uri=None, scale=None):
        self.uri = uri
        self.scale = scale


reflect(
    Mesh,
    tag="mesh",
    params=[Element("uri", str), Element("scale", "vector3", required=False)],
)


class GeometricType(ValueType):
    def __init__(self):
        self.factory = FactoryType(
            "geometric",
            {
                "box": Box,
                "cylinder": Cylinder,
                "sphere": Sphere,
                "mesh": Mesh,
                "plane": Plane,
            },
        )

    def from_xml(self, node, path):
        children = xml_children(node)
        assert len(children) == 1, "One element only for geometric"
        return self.factory.from_xml(children[0], path=path)

    def write_xml(self, node, obj):
        name = self.factory.get_name(obj)
        child = node_add(node, name)
        obj.write_xml(child)


add_type("geometric", GeometricType())


class Collision(Entity):
    def __init__(self, name=None, pose=None, geometry=None):
        Entity.__init__(self, name, pose)
        self.geometry = geometry


reflect(
    Collision,
    parent_cls=Link,
    params=[
        pose_element,
        Element("geometry", "geometric"),
        Element("material", Material, False),
        Element("surface", str, var="surface", required=False),
    ],
)


class Sensor(Entity):
    def __init__(self, name=None, pose=None):
        Entity.__init__(self, name, pose)
        self.name = name
        self.pose = pose


reflect(
    Sensor,
    parent_cls=Link,
    params=[
        Attribute("type", str),
        Element("always_on", str, required=False),
        Element("update_rate", float, required=False),
        Element("contact", str, required=False),
        pose_element,
    ],
)


class Visual(Entity):
    def __init__(self, name=None, pose=None, geometry=None, material=None):
        Entity.__init__(self, name, pose)
        self.geometry = geometry
        self.material = material


reflect(
    Visual,
    parent_cls=Link,
    params=[
        pose_element,
        Element("geometry", "geometric"),
        Element("material", Material, required=False),
        Element("cast_shadows", str, required=False),
    ],
)


reflect(
    Link,
    parent_cls=Entity,
    params=[
        Element("inertial", Inertial, required=False),
        AggregateElement("visual", Visual, var="visuals"),
        AggregateElement("collision", Collision, var="collisions"),
        Element("self_collide", str, required=False),
        Element("sensor", Sensor, var="sensor", required=False),
    ],
)


class Axis(Object):
    def __init__(self, xyz=None):
        self.xyz = xyz


class Joint(Object):
    TYPES = [
        "unknown",
        "ball",
        "continuous",
        "fixed",
        "prismatic",
        "revolute",
        "screw",
        "universal",
    ]

    def __init__(
        self, name=None, pose=None, parent=None, child=None, joint_type=None, axis=None
    ):
        self.name = name
        self.parent = parent
        self.child = child
        self.type = joint_type
        self.axis = axis
        self.pose = pose

    def check_valid(self):
        assert self.type in self.TYPES, "Invalid joint type: {}".format(
            self.type
        )  # noqa

    # Aliases
    @property
    def joint_type(self):
        return self.type

    @joint_type.setter
    def joint_type(self, value):
        self.type = value


reflect(Axis, parent_cls=Joint, params=[Element("xyz", "vector3", required=False)])

reflect(
    Joint,
    tag="joint",
    params=[
        name_attribute,
        Attribute("type", str),
        pose_element,
        Element("axis", Axis, False),
        Element("parent", str),
        Element("child", str),
    ],
)


class Model(Object):
    def __init__(self, name=None):
        self.name = name
        self.aggregate_init()

        self.links = []
        self.joints = []

        self.joint_map = {}
        self.link_map = {}

        self.parent_map = {}
        self.child_map = {}

    def add_aggregate(self, typeName, elem):
        Object.add_aggregate(self, typeName, elem)

        if typeName == "joint":
            joint = elem
            self.joint_map[joint.name] = joint
            self.parent_map[joint.child] = (joint.name, joint.parent)
            if joint.parent in self.child_map:
                self.child_map[joint.parent].append((joint.name, joint.child))
            else:
                self.child_map[joint.parent] = [(joint.name, joint.child)]
        elif typeName == "link":
            link = elem
            self.link_map[link.name] = link

    def add_link(self, link):
        self.add_aggregate("link", link)

    def add_joint(self, joint):
        self.add_aggregate("joint", joint)

    def get_chain(self, root, tip, joints=True, links=True, fixed=True):
        chain = []
        if links:
            chain.append(tip)
        link = tip
        while link != root:
            (joint, parent) = self.parent_map[link]
            if joints:
                if fixed or self.joint_map[joint].joint_type != "fixed":
                    chain.append(joint)
            if links:
                chain.append(parent)
            link = parent
        chain.reverse()
        return chain

    def get_root(self):
        root = None
        for link in self.link_map:
            if link not in self.parent_map:
                assert root is None, "Multiple roots detected, invalid URDF."
                root = link
        assert root is not None, "No roots detected, invalid URDF."
        return root


reflect(
    Model,
    tag="model",
    params=[
        name_attribute,
        pose_element,
        Element("static", str, required=False, default="false"),
        AggregateElement("link", Link, var="links"),
        AggregateElement("joint", Joint, var="joints"),
    ],
)


class SDF(Object):
    SUPPORTED_VERSIONS = ["1.5", "1.6", "1.7", "1.8", "1.9"]

    def __init__(self, version="1.9", scene=None, model=None, actor=None, light=None):
        self.aggregate_init()

        if version not in self.SUPPORTED_VERSIONS:
            raise ValueError(
                "Invalid version; only %s is supported"
                % (",".join(self.SUPPORTED_VERSIONS))
            )

        self.version = version
        self.scene = scene
        self.model = model
        self.actor = actor
        self.light = light
        self.worlds = []

    def process_includes(self):
        print(self.includes)

    def post_read_xml(self):
        if self.version is None:
            self.version = "1.9"

        split = self.version.split(".")
        if len(split) != 2:
            raise ValueError("The version attribute should be in the form 'x.y'")

        if split[0] == "" or split[1] == "":
            raise ValueError("Empty major or minor number is not allowed")

        if int(split[0]) < 0 or int(split[1]) < 0:
            raise ValueError("Version number must be positive")

        if self.version not in self.SUPPORTED_VERSIONS:
            raise ValueError(
                "Invalid version; only %s is supported"
                % (",".join(self.SUPPORTED_VERSIONS))
            )


class World(Object):
    def __init__(self, name=None):
        self.name = name
        self.aggregate_init()
        self.models = []
        self.includes = []


class State(Object):
    def __init__(self, world_name=None):
        self.world_name = world_name
        self.aggregate_init()
        self.model_states = []


class ModelState(Object):
    def __init__(self, name=None):
        self.name = name
        self.aggregate_init()
        self.joint_states = []


class JointState(Object):
    def __init__(self, name=None):
        self.name = name
        self.aggregate_init()
        self.angle = []


class Include(Object):
    def __init__(self):
        pass

    @property
    def model_name(self):
        return self.uri.split("/")[-1] + "/model.sdf"


reflect(
    JointState,
    parent_cls=ModelState,
    params=[name_attribute, Element("angle", float)],
)

reflect(
    ModelState,
    parent_cls=State,
    params=[
        name_attribute,
        AggregateElement("joint", JointState, var="joint_states"),
    ],
)

reflect(
    State,
    tag="state",
    params=[
        Attribute("world_name", str),
        AggregateElement("model", ModelState, var="model_states"),
    ],
)

reflect(
    World,
    tag="world",
    params=[
        name_attribute,
        Element("state", State, required=False),
        AggregateElement("model", Model, var="models"),
        AggregateElement("include", Include, var="includes"),
    ],
)

reflect(
    Include,
    params=[
        Element("name", str, required=False),
        Element("uri", str, required=False),
        Element("pose", str, required=False),
    ],
)


reflect(
    SDF,
    tag="sdf",
    params=[
        Attribute("version", str),
        Element("model", Model, required=False),
        # TODO: add these
        # Element('scene', Scene, required=False),
        # Element('actor', Actor, required=False),
        Element("light", str, required=False),
        AggregateElement("world", World, var="worlds"),
    ],
)

end_namespace()
