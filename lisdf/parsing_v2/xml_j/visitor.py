#! /usr/bin/env python3
# -*- coding: utf-8 -*-
# File   : visitor.py
# Author : Jiayuan Mao
# Email  : maojiayuan@gmail.com
# Date   : 03/23/2022
#
# This file is part of lisdf.
# Distributed under terms of the MIT license.

import functools
import os.path as osp
from collections import defaultdict
from typing import Any, Callable, Dict, List, Optional

from lisdf.utils.printing import indent_text

from .xml import XMLNode, load_file

__all__ = ["XMLVisitor"]


class XMLVisitor(object):
    def __init__(self) -> None:
        self.filename_stack: List[str] = list()
        self.node_stack: List[XMLNode] = list()
        self._st: Dict[str, List[Any]] = defaultdict(list)  # optional stacks.
        self._data: Dict[str, Dict[str, Any]] = defaultdict(dict)  # optional data.
        self._indent: int = 0

        self.verbose = False

    def set_verbose(self, flag: bool = True) -> None:
        self.verbose = flag

    def _get_processor(self, tag: str) -> Optional[Callable[[XMLNode], Any]]:
        tag = tag.replace("-", "_")
        return getattr(self, tag, None)

    def load_file(self, filename: str) -> Any:
        node = load_file(filename)
        return self.visit(filename, node)

    def visit(self, filename: str, root: XMLNode) -> Any:
        self.filename_stack.append(filename)
        if self.verbose:
            print(indent_text(filename, self._indent))

        def _inner(node: XMLNode):
            if self.verbose:
                print(indent_text(node.open_tag(), self._indent))
                if node.text:
                    print(indent_text(node.text, self._indent + 1))
            self._indent += 1
            try:
                proc = self._get_processor(node.tag + "_init")
                if proc is not None:
                    proc(node)
                self.node_stack.append(node)
                for i, c in enumerate(node.children):
                    node.children[i] = _inner(c)
                node.children = [c for c in node.children if c is not None]
                self.node_stack.pop()
                proc = self._get_processor(node.tag)

                if proc is not None:
                    if self.verbose:
                        print(indent_text(node.close_tag(), self._indent - 1))
                    return proc(node)
                if self.verbose:
                    print(indent_text(node.close_tag(), self._indent - 1))
                return node
            except Exception as e:
                raise e
            finally:
                self._indent -= 1

        try:
            root = root.clone()
            return _inner(root)
        finally:
            self.filename_stack.pop()

    def _resolve_path(self, path) -> str:
        return osp.normpath(osp.join(osp.dirname(self.filename_stack[-1]), path))

    def _pop_children(
        self, node: XMLNode, tag: str, required=False, return_type="text", default=None
    ):
        assert return_type in ("node", "text", "data")

        rv = list()
        for i, c in enumerate(node.children):
            if c.tag == tag:
                rv.append(i)
        assert len(rv) in (0, 1)
        if len(rv) == 0:
            assert not required
            return default
        else:
            obj = node.children[rv[0]]
            node.children = node.children[: rv[0]] + node.children[rv[0] + 1 :]
            if return_type == "node":
                return obj
            elif return_type == "text":
                return obj.text
            elif return_type == "data":
                return obj.data
            else:
                raise ValueError("Unknown return type: {}.".format(return_type))

    def _check_done(self, node: XMLNode, attr=True, children=True):
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
        if attr:
            assert len(rv.attributes) == 0
        if children:
            assert len(rv.children) == 0
        return rv

    return wrapped
