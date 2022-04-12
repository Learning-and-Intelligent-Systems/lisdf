import lisdf.components as C

from .xml_j.visitor import XMLVisitorInterface, check_done_decorator
from .xml_j.xml import XMLNode


class SRDFParserMixin(XMLVisitorInterface):
    """
    A configurable parameter for controlling the syntax of the SRDF file.

    When USE_ATTRIBUTES is true, arguments are specified as attributes of the
    XML node. For example,
        <disable_collisions link1="x" link2="y" />

    When USE_ATTRIBUTES is false, arguments are specified as child nodes of the
    XML node. For example,
        <disable_collisions>
            <link1 link="x">
            <link2 link="y" />
        </disable_collisions>
    """

    USE_ATTRIBUTES = True

    def group_init(self, node: XMLNode):
        if self.get_scope() != "group":  # not already in a group.
            self.enter_scope("group")
            node.attributes["is_group_def"] = True  # store a temporary attribute.

    @check_done_decorator
    def group(self, node: XMLNode):
        is_group_def = node.attributes.pop("is_group_def", False)

        if is_group_def:
            self.exit_scope("group")
            group = C.Group(node.attributes.pop("name"))
            for c in node.pop_all_children():
                if c.tag == "group":
                    group.sub_groups.append(c.data)
                elif c.tag == "link":
                    group.links.append(c.data)
                elif c.tag == "joint":
                    group.joints.append(c.data)
                elif c.tag == "chain":
                    group.joints.append(c.data)
                else:
                    raise ValueError(f"Unknown tag {c.tag}.")
            return node.set_data(group)

        return node.set_data(C.GroupIdentifier(node.attributes.pop("name")))

    @check_done_decorator
    def group_link(self, node: XMLNode):
        return node.set_data(C.LinkIdentifier(node.attributes.pop("name")))

    @check_done_decorator
    def group_joint(self, node: XMLNode):
        return node.set_data(C.JointIdentifier(node.attributes.pop("name")))

    @check_done_decorator
    def group_chain(self, node: XMLNode):
        return node.set_data(
            C.ChainIdentifier(
                node.attributes.pop("base_link"), node.attributes.pop("tip_link")
            )
        )

    @check_done_decorator
    def disable_collisions(self, node: XMLNode):
        if not self.USE_ATTRIBUTES:
            return node.set_data(
                C.DisableCollisions(
                    node.pop("link1", return_type="node", required=True).attributes.pop(
                        "name"
                    ),
                    node.pop("link2", return_type="node", required=True).attributes.pop(
                        "name"
                    ),
                    node.pop("reason", return_type="text", default=None),
                )
            )
        else:
            return node.set_data(
                C.DisableCollisions(
                    node.attributes.pop("link1"),
                    node.attributes.pop("link2"),
                    node.attributes.pop("reason", None),
                )
            )
