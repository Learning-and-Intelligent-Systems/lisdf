import re
import xml.etree.ElementTree as et
from typing import Any, List, Optional

from lisdf.utils.printing import indent_text


class _G(dict):
    def __getattr__(self, attr):
        return self[attr]


class XMLNode(object):
    tag: str
    attributes: _G
    children: List["XMLNode"]
    parent: Optional["XMLNode"] = None
    text: Optional[str]
    data: Any

    def __init__(self, tag: str):
        self.tag = tag
        self.attributes = _G()
        self.children = list()
        self.parent = None
        self.text = ""
        self.data = None

    def add(self, node: "XMLNode"):
        self.children.append(node)
        node.set_parent(self)

    def set_text(self, text: str):
        self.text = text.strip()

    def set_parent(self, parent: "XMLNode"):
        self.parent = parent

    def set_data(self, data: Any):
        self.data = data
        return self

    def pop(self, tag: str, required=False, return_type="text", default=None) -> Any:
        """Pop a child node from the children list based on the tag name.
        This function also checks the uniqueness of such child node."""
        assert return_type in ("node", "text", "data")

        rv = list()
        for i, c in enumerate(self.children):
            if c.tag == tag:
                rv.append(i)
        assert len(rv) in (0, 1)
        if len(rv) == 0:
            assert not required
            return default
        else:
            obj = self.children[rv[0]]
            self.children = self.children[: rv[0]] + self.children[rv[0] + 1 :]
            if return_type == "node":
                return obj
            elif return_type == "text":
                return obj.text
            elif return_type == "data":
                return obj.data
            else:
                raise ValueError("Unknown return type: {}.".format(return_type))

    def pop_all_children(self) -> List["XMLNode"]:
        try:
            return self.children
        finally:
            self.children = list()

    def clone(self) -> "XMLNode":
        node = XMLNode(self.tag)
        node.attributes.update(self.attributes.copy())
        node.children = [c.clone() for c in self.children]
        for c in node.children:
            c.parent = self
        node.text = self.text
        return node

    def open_tag(self) -> str:
        if len(self.attributes):
            attributes = " " + (
                " ".join([f'{key}="{value}"' for key, value in self.attributes.items()])
            )
        else:
            attributes = ""
        return f"<{self.tag}{attributes}>"

    def close_tag(self) -> str:
        return f"</{self.tag}>"

    def __str__(self):
        fmt = ""
        if len(self.attributes):
            attributes = " " + (
                " ".join([f'{key}="{value}"' for key, value in self.attributes.items()])
            )
        else:
            attributes = ""
        fmt += f"<{self.tag}{attributes}"
        if self.text or len(self.children):
            fmt += ">\n"
            if self.text:
                fmt += indent_text(self.text) + "\n"
            for c in self.children:
                fmt += indent_text(str(c)).rstrip() + "\n"
            fmt += f"</{self.tag}>\n"
        else:
            fmt += " />\n"
        return fmt

    __repr__ = __str__


def _xml2obj(element) -> XMLNode:
    if element.tag is None and len(element.attrib) == 0 and len(element) == 0:
        return element.text

    node = XMLNode(element.tag)
    node.attributes.update(element.attrib)
    if element.text is not None:
        node.set_text(element.text)
    for c in element:
        c = _xml2obj(c)
        if isinstance(c, str):
            node.set_text(c)
        else:
            node.add(c)
    return node


def load_string(value: str) -> XMLNode:
    # TODO:: temporary fix for some drake files.
    value = re.sub(' xmlns="[^"]+"', "", value, count=1)
    return _xml2obj(et.fromstring(value))


def load_file(filename: str) -> XMLNode:
    with open(filename) as f:
        return load_string(f.read())
