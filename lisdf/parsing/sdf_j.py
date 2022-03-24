#! /usr/bin/env python3
# -*- coding: utf-8 -*-
# File   : sdf_j.py
# Author : Jiayuan Mao
# Email  : maojiayuan@gmail.com
# Date   : 03/23/2022
#
# This file is part of lisdf.
# Distributed under terms of the MIT license.

import lisdf.components as C

from .string_utils import bool_string, vector2f, vector3f, vector4f, vector6f
from .xml_j.visitor import XMLVisitor, check_done_decorator

__all__ = ["SDFVisitor", "load_sdf"]


class SDFVisitor(XMLVisitor):
    def include(self, node):
        # TODO: Fix
        return None

    @check_done_decorator
    def inertia(self, node):
        return node.set_data(
            C.Inertia(
                float(self._pop_children(node, "ixx", default=0)),
                float(self._pop_children(node, "ixy", default=0)),
                float(self._pop_children(node, "ixz", default=0)),
                float(self._pop_children(node, "iyy", default=0)),
                float(self._pop_children(node, "iyz", default=0)),
                float(self._pop_children(node, "izz", default=0)),
            )
        )

    @check_done_decorator
    def pose(self, node):
        return node.set_data(C.Pose.from_rpy_6d(vector6f(node.text)))

    @check_done_decorator
    def inertial(self, node):
        return node.set_data(
            C.Inertial(
                float(self._pop_children(node, "mass", default=0)),
                self._pop_children(
                    node, "pose", return_type="data", default=C.Pose.identity()
                ),
                self._pop_children(
                    node, "inertia", return_type="data", default=C.Inertia.zeros()
                ),
            )
        )

    @check_done_decorator
    def contact(self, node):
        if self.node_stack[-1].tag == "surface":
            return node.set_data(
                C.SurfaceContact(
                    sdf_configs={
                        "collide_bitmask": int(
                            self._pop_children(
                                node, "collide_bitmask", default="0xffff"
                            ),
                            0,
                        ),
                        "collide_without_contact": bool_string(
                            self._pop_children(
                                node, "collide_without_contact", default="false"
                            )
                        ),
                    }
                )
            )
        elif self.node_stack[-1].tag == "sensor":
            # TODO: Fix
            node.children = list()
        return node

    @check_done_decorator
    def friction(self, node):
        configs = dict()

        ode_node = self._pop_children(node, "ode", return_type="node")
        if ode_node is not None:
            configs["ode_mu"] = float(self._pop_children(ode_node, "mu", default=1))
            configs["ode_mu2"] = float(self._pop_children(ode_node, "mu2", default=1))
        return node.set_data(C.SurfaceFriction(sdf_configs=configs))

    @check_done_decorator
    def surface(self, node):
        return node.set_data(
            C.Surface(
                self._pop_children(
                    node, "contact", return_type="data", default=C.SurfaceContact()
                ),
                self._pop_children(
                    node, "friction", return_type="data", default=C.SurfaceFriction()
                ),
            )
        )

    @check_done_decorator
    def link(self, node):
        name = node.attributes.pop("name")
        pose = self._pop_children(
            node, "pose", return_type="data", default=C.Pose.identity()
        )
        inertial = self._pop_children(
            node, "inertial", return_type="data", default=C.Inertial.zeros()
        )
        self_collide = bool_string(
            self._pop_children(node, "self_collide", default="true")
        )
        link = C.Link(name, pose, inertial, sdf_configs=dict(self_collide=self_collide))
        for c in node.children:
            if c.tag == "collision":
                link.collisions.append(c.data)
            elif c.tag == "visual":
                link.collisions.append(c.data)
            elif c.tag == "sensor":
                link.sensors.append(c.data)
            else:
                raise TypeError("Unknown tag: {}.".format(c.tag))
        node.children = list()  # clear the children list.
        return node.set_data(link)

    @check_done_decorator
    def collision(self, node):
        if self.node_stack[-1].tag == "link":
            return node.set_data(
                C.Geom(
                    node.attributes.pop("name", None),
                    self._pop_children(
                        node, "pose", return_type="data", default=C.Pose.identity()
                    ),
                    self._pop_children(
                        node, "geometry", return_type="data", required=True
                    ),
                    surface=self._pop_children(
                        node, "surface", return_type="data", default=C.Surface()
                    ),
                )
            )
        return node

    @check_done_decorator
    def visual(self, node):
        cast_shadows = bool_string(
            self._pop_children(node, "cast_shadows", default="true")
        )
        return node.set_data(
            C.Geom(
                node.attributes.pop("name", None),
                self._pop_children(
                    node, "pose", return_type="data", default=C.Pose.identity()
                ),
                self._pop_children(node, "geometry", return_type="data", required=True),
                self._pop_children(
                    node,
                    "material",
                    return_type="data",
                    default=C.PhongMaterial.default(),
                ),
                sdf_configs=dict(cast_shadows=cast_shadows),
            )
        )

    @check_done_decorator
    def geometry(self, node):
        assert len(node.children) == 1
        c = node.children.pop()
        if c.tag == "box":
            return node.set_data(
                C.BoxShapeInfo(vector3f(self._pop_children(c, "size", required=True)))
            )
        elif c.tag == "sphere":
            return node.set_data(
                C.SphereShapeInfo(float(self._pop_children(c, "radius", required=True)))
            )
        elif c.tag == "cylinder":
            return node.set_data(
                C.CylinderShapeInfo(
                    float(self._pop_children(c, "radius", required=True)),
                    float(self._pop_children(c, "length", required=True)) / 2,
                )
            )
        elif c.tag == "capsule":
            return node.set_data(
                C.CapsuleShapeInfo(
                    float(self._pop_children(c, "radius", required=True)),
                    float(self._pop_children(c, "length", required=True)) / 2,
                )
            )
        elif c.tag == "plane":
            size = vector2f(self._pop_children(c, "size", required=True))
            return node.set_data(
                C.PlaneShapeInfo(
                    half_width=size[0] / 2,
                    half_height=size[1] / 2,
                    normal=vector3f(self._pop_children(c, "normal", default="0 0 1")),
                )
            )
        elif c.tag == "mesh":
            return node.set_data(
                C.MeshShapeInfo(
                    self._resolve_path(self._pop_children(c, "uri", required=True)),
                    vector3f(self._pop_children(c, "scale", default="1 1 1")),
                )
            )
        else:
            raise NotImplementedError("Unknown geometry tag: {}.".format(c.tag))

    @check_done_decorator
    def material(self, node):
        self._pop_children(node, "script")  # TODO: fix it.

        return node.set_data(
            C.PhongMaterial(
                vector4f(self._pop_children(node, "ambient", default="1 1 1 1")),
                vector4f(self._pop_children(node, "diffuse", default="1 1 1 1")),
                vector4f(self._pop_children(node, "specular", default="0 0 0 1")),
                vector4f(self._pop_children(node, "emissive", default="0 0 0 1")),
            )
        )

    @check_done_decorator
    def sensor(self, node):
        name = node.attributes.pop("name")
        type = node.attributes.pop("type")
        if type == "camera":
            return node.set_data(C.Sensor.from_type(type, name=name))
        else:
            # TODO:: Fix.
            node.children = list()
            return node

    @check_done_decorator
    def joint(self, node):
        type = node.attributes.pop("type", "continuous")

        # TODO: add dynamics
        if type == "fixed":
            joint_info = C.FixedJointInfo()
        elif type == "continuous":
            joint_info = C.HingeJointInfo(
                vector3f(
                    self._pop_children(
                        self._pop_children(
                            node, "axis", required=True, return_type="node"
                        ),
                        "xyz",
                        required=True,
                        return_type="text",
                    )
                )
            )
        elif type == "revolute":
            # TODO: Handle limits
            joint_info = C.HingeJointInfo(
                vector3f(
                    self._pop_children(
                        self._pop_children(
                            node, "axis", required=True, return_type="node"
                        ),
                        "xyz",
                        required=True,
                        return_type="text",
                    )
                )
            )
        elif type == "prismatic":
            joint_info = C.PrismaticJointInfo(
                vector3f(
                    self._pop_children(
                        self._pop_children(
                            node, "axis", required=True, return_type="node"
                        ),
                        "xyz",
                        required=True,
                        return_type="text",
                    )
                )
            )
        else:
            raise NotImplementedError("Unknown joint type: {}.".format(type))

        joint = C.Joint(
            node.attributes.pop("name", None),
            self._pop_children(node, "parent", required=True),
            self._pop_children(node, "child", required=True),
            self._pop_children(
                node, "pose", return_type="data", default=C.Pose.identity()
            ),
            joint_info=joint_info,
        )
        return node.set_data(joint)

    @check_done_decorator
    def model(self, node):
        name = node.attributes.pop("name", None)
        pose = self._pop_children(
            node, "pose", return_type="data", default=C.Pose.identity()
        )
        static = bool_string(self._pop_children(node, "static", default="false"))
        model = C.Model(name, pose, None, static=static)
        for c in node.children:
            if c.tag == "joint":
                model.joints.append(c.data)
            elif c.tag == "link":
                model.links.append(c.data)
            else:
                raise NotImplementedError("Unknown tag: {}.".format(c.tag))
        node.children = list()
        return node.set_data(model)

    @check_done_decorator
    def urdf_model(self, node):
        """
        <urdf-model name="pr2">
            <uri>pr2.urdf</uri>
            <pose>0 0 0 0 0 0</pose>
            <size>1 1 1</pose>
            <static>false</static>
        </urdf-model>
        """
        name = node.attributes.pop("name", None)
        uri = self._resolve_path(self._pop_children(node, "uri", required=True))
        size = vector3f(self._pop_children(node, "size", default="1 1 1"))
        pose = self._pop_children(
            node, "pose", return_type="data", default=C.Pose.identity()
        )
        static = bool_string(self._pop_children(node, "static", default="false"))
        return node.set_data(C.URDFModel(name, uri, size, pose, static=static))

    @check_done_decorator
    def world(self, node):
        name = node.attributes.pop("name", None)
        static = bool_string(self._pop_children(node, "static", default="false"))
        world = C.World(name, static)
        for c in node.children:
            if c.tag == "model":
                world.models.append(c.data)
            else:
                raise NotImplementedError("Unknown tag: {}.".format(c.tag))
        node.children = list()
        return node.set_data(world)

    @check_done_decorator
    def sdf(self, node):
        sdf_version = node.attributes.pop("version")
        lisdf = C.LISDF(sdf_version)
        for c in node.children:
            if c.tag == "world":
                lisdf.worlds.append(c.data)
            elif c.tag == "model":
                assert lisdf.model is None
                lisdf.model = c.data
            else:
                raise NotImplementedError("Unknown tag: {}.".format(c.tag))
        node.children = list()
        return node.set_data(lisdf)


def load_sdf(filename):
    visitor = SDFVisitor()
    return visitor.load_file(filename).data
