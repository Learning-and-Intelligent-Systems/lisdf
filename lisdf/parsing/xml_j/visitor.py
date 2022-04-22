import functools
import os.path as osp
from abc import ABC, abstractmethod
from collections import defaultdict
from typing import Any, Callable, Dict, List, Optional

from lisdf.parsing.xml_j.xml import XMLNode, load_file, load_string
from lisdf.utils.printing import indent_text


class XMLVisitorInterface(ABC):
    @abstractmethod
    def get_scope(self) -> Optional[str]:
        raise NotImplementedError()

    @abstractmethod
    def enter_scope(self, scope: str) -> None:
        raise NotImplementedError()

    @abstractmethod
    def exit_scope(self, scope: str) -> None:
        raise NotImplementedError()


class XMLVisitor(XMLVisitorInterface):
    """
    A Top-Down Visitor for the XML tree.

    The basic idea is to have a list of callbacks for each XML node type.
    There are a few stacks that keep track of path from the root to the
    current node.


    - The filename stack keeps track of the current file being processed.
    For example, when you handle an <include> tag, you may call the
    load_file method to load the included file. In this case, the filename
    will be pushed into the stack.
    - The tag stack keeps track of the current tag being processed.
    - The scope stack is a special stack maintained by the programmer.
    It is used to decide which function to be called given the tag.
    Specifically, when the scope stack is empty, upon seeing a new node with
    tag <model>, the visitor will call the function named model(node).
    If the scope stack is empty, say, the top element is "state". Upon seeing
    a new node with tag <model>, the visitor will call the function named
    state_model(node).
    The programmer can use enter_scope and exit_scope to push and pop.

    See the docstring for the visit method for detailed explanation.
    """

    def __init__(self) -> None:
        self.filename_stack: List[str] = list()
        self.node_stack: List[XMLNode] = list()
        self.scope_stack: List[str] = list()

        # The SDF parser doesn't rely on additional stacks.
        # These are primarily for the MJCF parser.

        # optional stacks.
        self._st: Dict[str, List[Any]] = defaultdict(list)
        # optional data.
        self._data: Dict[str, Dict[str, Any]] = defaultdict(dict)
        self._indent: int = 0
        self._verbose = False

    def set_verbose(self, flag: bool = True) -> None:
        self._verbose = flag

    def _get_processor(self, tag: str) -> Optional[Callable[[XMLNode], Any]]:
        if len(self.scope_stack) == 0:
            tag = tag.replace("-", "_")
            return getattr(self, tag, None)
        funcname = self.scope_stack[-1] + "_" + tag.replace("-", "_")
        if hasattr(self, funcname):
            return getattr(self, funcname)
        funcname = tag.replace("-", "_")
        return getattr(self, funcname, None)

    def get_scope(self) -> Optional[str]:
        return self.scope_stack[-1] if len(self.scope_stack) > 0 else None

    def enter_scope(self, scope: str) -> None:
        self.scope_stack.append(scope)

    def exit_scope(self, scope: str) -> None:
        el = self.scope_stack.pop()
        if el != scope:
            raise NameError(
                'Exiting scope "{}" but current scope is "{}".'.format(scope, el)
            )

    def load_file(self, filename: str) -> Any:
        node = load_file(filename)
        return self.visit(filename, node)

    def load_string(self, string: str) -> Any:
        node = load_string(string)
        return self.visit("string_file", node)

    def _resolve_path(self, path: str) -> str:
        return osp.normpath(osp.join(osp.dirname(self.filename_stack[-1]), path))

    def visit(self, filename: str, root: XMLNode) -> Any:
        """
        The visit function will traverse the XML tree and call the functions
        based on the tag. Specifically, when entering a new node with tag <model>,
        the visitor will call the function named model_init(node).
        Then, the visitor will recursively call the visit function for each
        child, and finally call the function named model(node).

        The strategy that I prefer is to use the check_done helper function.
        Specifically, when processing a node, one should always use
        node.pop() for selecting a child node, and node.attributes.pop() for
        selecting an attribute. After done processing the node, one should
        call check_done(node). If the node is not empty, the helper function
        will raise an Error.

        Args:
            filename: the filename of the corresponding XML file.
            root: the root node of the XML tree.

        Returns:
            The return value of user's function applied to the root node.
        """
        self.filename_stack.append(filename)
        if self._verbose:
            print(indent_text(filename, self._indent))

        def _inner(node: XMLNode):
            if self._verbose:
                print(indent_text(node.open_tag(), self._indent))
                if node.text:
                    print(indent_text(node.text, self._indent + 1))
            self._indent += 1
            try:
                proc = self._get_processor(node.tag + "_init")
                if proc is not None:
                    rv = proc(node)
                    if rv == "skip":
                        return None

                self.node_stack.append(node)
                for i, c in enumerate(node.children):
                    node.children[i] = _inner(c)
                node.children = [c for c in node.children if c is not None]
                self.node_stack.pop()
                proc = self._get_processor(node.tag)

                if proc is not None:
                    if self._verbose:
                        print(indent_text(node.close_tag(), self._indent - 1))
                    return proc(node)
                if self._verbose:
                    print(indent_text(node.close_tag(), self._indent - 1))
                return node
            finally:
                self._indent -= 1

        # NB(Jiayuan Mao @ 03/25): Essentially, `defer self.filename_stack.pop()`.
        try:
            root = root.clone()
            return _inner(root)
        finally:
            self.filename_stack.pop()


def check_done(node: XMLNode, attr: bool = True, children: bool = True) -> None:
    """A helper function to check whether all attributes and children of a
    node have been processed."""
    if attr:
        if len(node.attributes) != 0:
            print("Unprocessed attributes.")
            print(node)
            print("-" * 120)
            raise ValueError()
    if children:
        if len(node.children) != 0:
            print("Unprocessed children.")
            print(node)
            print("-" * 120)
            raise ValueError()
    return None


def check_done_decorator(func, attr=True, children=True):
    @functools.wraps(func)
    def wrapped(*args, **kwargs):
        rv = func(*args, **kwargs)
        if rv is not None:
            check_done(rv, attr, children)
        return rv

    return wrapped
