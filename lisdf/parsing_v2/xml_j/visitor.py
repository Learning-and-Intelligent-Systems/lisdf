#! /usr/bin/env python3
# -*- coding: utf-8 -*-
# File   : visitor.py
# Author : Jiayuan Mao
# Email  : maojiayuan@gmail.com
# Date   : 03/23/2022
#
# This file is part of lisdf.
# Distributed under terms of the MIT license.

from typing import Any, Optional
from collections import defaultdict
from lisdf.utils.printing import indent_text
from .xml import XMLNode, load_file

__all__ = ['XMLVisitor']


class XMLVisitor(object):
    def __init__(self):
        self.filename_stack = list()
        self.node_stack = list()
        self._st = defaultdict(list)  # optional stacks.
        self._data = defaultdict(dict)  # optional data.
        self._indent = 0

        self.verbose = False

    def set_verbose(self, flag=True):
        self.verbose = flag

    def _get_processor(self, tag):
        return getattr(self, tag, None)

    def load_file(self, filename):
        node = load_file(filename)
        return self.visit(filename, node)

    def visit(self, filename: str, root: XMLNode) -> Any:
        self.filename_stack.append(filename)
        if self.verbose:
            print(indent_text(filename, self._indent))

        def _inner(node: XMLNode):
            if self.verbose:
                print(indent_text(node.open_tag(), self._indent))
            self._indent += 1
            try:
                proc = self._get_processor(node.tag + '_init')
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
            except:
                raise
            finally:
                self._indent -= 1

        try:
            root = root.clone()
            return _inner(root)
        finally:
            self.filename_stack.pop()

    def _check_done(self, node: XMLNode, attr=True, children=True):
        if attr:
            if len(node.attributes) != 0:
                print('Unprocessed attributes.')
                print(node)
                print('-' * 120)
                raise ValueError()
        if children:
            if len(node.children) != 0:
                print('Unprocessed children.')
                print(node)
                print('-' * 120)
                raise ValueError()
        return None

