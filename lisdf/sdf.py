import pdb
from urdf_parser_py.xml_reflection.basics import *
from urdf_parser_py.xml_reflection.core import get_type
import urdf_parser_py.xml_reflection as xmlr

# TODO: add __str__ methods

xmlr.start_namespace('sdf')

'''
class Pose(xmlr.Object):
    def __init__(self, vec=None):
        self.pose = vec

xmlr.reflect(Pose, tag='pose', params=[
    xmlr.Attribute('pose', 'vector6', False, default=[0, 0, 0, 0, 0, 0]),
])
'''

name_attribute = xmlr.Attribute('name', str)
pose_element = xmlr.Element('pose', 'vector6', False)

class Entity(xmlr.Object):
    def __init__(self, name=None, pose=None):
        self.name = name
        self.pose = pose


xmlr.reflect(Entity, params=[
    name_attribute,
    pose_element
])


class Inertia(xmlr.Object):
    KEYS = ['ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz']

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
            [self.ixz, self.iyz, self.izz]]


xmlr.reflect(Inertia,
             params=[xmlr.Element(key, float) for key in Inertia.KEYS])


class Inertial(xmlr.Object):
    def __init__(self, mass=0.0, inertia=None, pose=None):
        self.mass = mass
        self.inertia = inertia
        self.pose = pose


xmlr.reflect(Inertial, params=[
    xmlr.Element('mass', float, required=False),
    xmlr.Element('inertia', Inertia, required=False),
    pose_element
])

class Link(Entity):
    def __init__(self, name=None, pose=None, visual=None, inertial=None, collision=None):
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
            self.add_aggregate('visual', visual)

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
            self.add_aggregate('collision', collision)

    # Properties for backwards compatibility
    visual = property(__get_visual, __set_visual)
    collision = property(__get_collision, __set_collision)

class Material(xmlr.Object):
    def __init__(self, script=None, ambient=None, diffuse=None, specular=None, emissive=None):
        self.script = script
        self.ambient = ambient
        self.diffuse = diffuse
        self.specular = specular
        self.emissive = emissive

xmlr.reflect(Material, tag='material', params=[
    xmlr.Element('script', str, required=False),
    xmlr.Element('ambient', 'vector4', required=False),
    xmlr.Element('diffuse', 'vector4', required=False),
    xmlr.Element('specular', 'vector4', required=False),
    xmlr.Element('emissive', 'vector4', required=False)
])

class Box(xmlr.Object):
    def __init__(self, size=None):
        self.size = size


xmlr.reflect(Box, tag='box', params=[
    xmlr.Element('size', 'vector3')
])

class Plane(xmlr.Object):
    def __init__(self, normal=0.0, size=0.0):
        self.normal = normal
        self.size = size

xmlr.reflect(Plane, tag='plane', params=[
    xmlr.Element('normal', 'vector3'),
    xmlr.Element('size', 'vector2')
])

class Cylinder(xmlr.Object):
    def __init__(self, radius=0.0, length=0.0):
        self.radius = radius
        self.length = length


xmlr.reflect(Cylinder, tag='cylinder', params=[
    xmlr.Element('radius', float),
    xmlr.Element('length', float)
])


class Sphere(xmlr.Object):
    def __init__(self, radius=0.0):
        self.radius = radius


xmlr.reflect(Sphere, tag='sphere', params=[
    xmlr.Element('radius', float)
])


class Mesh(xmlr.Object):
    def __init__(self, uri=None, scale=None):
        self.uri = uri
        self.scale = scale


xmlr.reflect(Mesh, tag='mesh', params=[
    xmlr.Element('uri', str),
    xmlr.Element('scale', 'vector3', required=False)
])

class GeometricType(xmlr.ValueType):
    def __init__(self):
        self.factory = xmlr.FactoryType('geometric', {
            'box': Box,
            'cylinder': Cylinder,
            'sphere': Sphere,
            'mesh': Mesh,
            'plane': Plane
        })

    def from_xml(self, node, path):
        children = xml_children(node)
        assert len(children) == 1, 'One element only for geometric'
        return self.factory.from_xml(children[0], path=path)

    def write_xml(self, node, obj):
        name = self.factory.get_name(obj)
        child = node_add(node, name)
        obj.write_xml(child)

xmlr.add_type('geometric', GeometricType())

class Collision(Entity):
    def __init__(self, name=None, pose=None, geometry=None):
        Entity.__init__(self, name, pose)
        self.geometry = geometry

xmlr.reflect(Collision, parent_cls=Link, params=[
    pose_element,
    xmlr.Element('geometry', 'geometric'),
    xmlr.Element('material', Material, False),
    xmlr.Element('surface', str, var='surface', required=False),
])


class Sensor(Entity):
    def __init__(self, name=None, pose=None):
        Entity.__init__(self, name, pose)
        self.name=name
        self.pose=pose


xmlr.reflect(Sensor, parent_cls=Link, params=[
    xmlr.Attribute("type", str),
    xmlr.Element("always_on", str, required=False),
    xmlr.Element("update_rate", float, required=False),
    xmlr.Element("contact", str, required=False),
    pose_element
])


class Visual(Entity):
    def __init__(self, name=None, pose=None, geometry=None, material=None):
        Entity.__init__(self, name, pose)
        self.geometry = geometry
        self.material = material

xmlr.reflect(Visual, parent_cls=Link, params=[
    pose_element,
    xmlr.Element('geometry', 'geometric'),
    xmlr.Element('material', Material, required=False),
    xmlr.Element('cast_shadows', str, required=False)
])


xmlr.reflect(Link, parent_cls=Entity, params=[
    xmlr.Element('inertial', Inertial, required=False),
    xmlr.AggregateElement('visual', Visual, var='visuals'),
    xmlr.AggregateElement('collision', Collision, var='collisions'),
    xmlr.Element('self_collide', str, required=False),
    xmlr.Element('sensor', Sensor, var='sensor', required=False)
])

class Axis(xmlr.Object):
    def __init__(self, xyz=None):
        self.xyz = xyz

class Joint(xmlr.Object):
    TYPES = ['unknown', 'ball', 'continuous', 'fixed', 'prismatic', 'revolute', 'screw', 'universal']

    def __init__(self, name=None,  pose=None, parent=None, child=None, joint_type=None,
                 axis=None):
        self.name = name
        self.parent = parent
        self.child = child
        self.type = joint_type
        self.axis = axis
        self.pose = pose

    def check_valid(self):
        assert self.type in self.TYPES, "Invalid joint type: {}".format(self.type)  # noqa

    # Aliases
    @property
    def joint_type(self): return self.type

    @joint_type.setter
    def joint_type(self, value): self.type = value

xmlr.reflect(Axis, parent_cls=Joint, params=[
    xmlr.Element('xyz', 'vector3', required=False)
    ])

xmlr.reflect(Joint, tag='joint', params=[
    name_attribute,
    xmlr.Attribute('type', str),
    pose_element,
    xmlr.Element('axis', Axis, False),
    xmlr.Element('parent', str),
    xmlr.Element('child', str),
])

class Model(xmlr.Object):
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
        xmlr.Object.add_aggregate(self, typeName, elem)

        if typeName == 'joint':
            joint = elem
            self.joint_map[joint.name] = joint
            self.parent_map[joint.child] = (joint.name, joint.parent)
            if joint.parent in self.child_map:
                self.child_map[joint.parent].append((joint.name, joint.child))
            else:
                self.child_map[joint.parent] = [(joint.name, joint.child)]
        elif typeName == 'link':
            link = elem
            self.link_map[link.name] = link

    def add_link(self, link):
        self.add_aggregate('link', link)

    def add_joint(self, joint):
        self.add_aggregate('joint', joint)

    def get_chain(self, root, tip, joints=True, links=True, fixed=True):
        chain = []
        if links:
            chain.append(tip)
        link = tip
        while link != root:
            (joint, parent) = self.parent_map[link]
            if joints:
                if fixed or self.joint_map[joint].joint_type != 'fixed':
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

xmlr.reflect(Model, tag='model', params=[
    name_attribute,
    pose_element,
    xmlr.Element('static', str, required=False, default="false"),
    xmlr.AggregateElement('link', Link, var='links'),
    xmlr.AggregateElement('joint', Joint, var='joints'),
])

class SDF(xmlr.Object):
    SUPPORTED_VERSIONS = ["1.5", "1.6", "1.7", "1.8", "1.9"]

    def __init__(self, version="1.9", scene=None, model=None, actor=None, light=None):
        self.aggregate_init()

        if version not in self.SUPPORTED_VERSIONS:
            raise ValueError("Invalid version; only %s is supported" % (','.join(self.SUPPORTED_VERSIONS)))

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

        if split[0] == '' or split[1] == '':
            raise ValueError("Empty major or minor number is not allowed")

        if int(split[0]) < 0 or int(split[1]) < 0:
            raise ValueError("Version number must be positive")

        if self.version not in self.SUPPORTED_VERSIONS:
            raise ValueError("Invalid version; only %s is supported" % (','.join(self.SUPPORTED_VERSIONS)))

class World(xmlr.Object):
    def __init__(self, name=None):
        self.name = name
        self.aggregate_init()
        self.models = []
        self.includes = []

class State(xmlr.Object):
    def __init__(self, world_name=None):
        self.world_name = world_name
        self.aggregate_init()
        self.model_states = []

class ModelState(xmlr.Object):
    def __init__(self, name=None):
        self.name = name
        self.aggregate_init()
        self.joint_states = []

class JointState(xmlr.Object):
    def __init__(self, name=None):
        self.name = name
        self.aggregate_init()
        self.angle = []

class Include(xmlr.Object):
    def __init__(self):
        pass

    @property
    def model_name(self):
        return self.uri.split("/")[-1]+"/model.sdf"



xmlr.reflect(JointState, parent_cls=ModelState, params=[
    name_attribute,
    xmlr.Element('angle', float)
    ])

xmlr.reflect(ModelState, parent_cls=State, params=[
    name_attribute,
    xmlr.AggregateElement('joint', JointState, var='joint_states')
    ])

xmlr.reflect(State, tag='state', params=[
    xmlr.Attribute('world_name', str),
    xmlr.AggregateElement('model', ModelState, var='model_states')
    ])

xmlr.reflect(World, tag='world', params=[
    name_attribute,
    xmlr.Element('state', State, required=False),
    xmlr.AggregateElement('model', Model, var='models'),
    xmlr.AggregateElement('include', Include, var='includes')
    ])

xmlr.reflect(Include, params=[
    xmlr.Element('name', str, required=False),
    xmlr.Element('uri', str, required=False),
    xmlr.Element('pose', str, required=False)
    ])


xmlr.reflect(SDF, tag='sdf', params=[
    xmlr.Attribute('version', str),
    xmlr.Element('model', Model, required=False),
    # TODO: add these
    # xmlr.Element('scene', Scene, required=False),
    # xmlr.Element('actor', Actor, required=False),
    xmlr.Element('light', str, required=False),
    xmlr.AggregateElement('world', World, var='worlds')
    ])

xmlr.end_namespace()
