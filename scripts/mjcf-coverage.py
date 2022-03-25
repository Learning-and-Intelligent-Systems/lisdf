#! /usr/bin/env python3
# -*- coding: utf-8 -*-
# File   : mjcf-coverage.py
# Author : Jiayuan Mao
# Email  : maojiayuan@gmail.com
# Date   : 03/23/2022
#
# This file is part of lisdf.
# Distributed under terms of the MIT license.

import argparse

from lisdf.parsing.mjcf import MJCFVisitor

parser = argparse.ArgumentParser()
parser.add_argument("file")
args = parser.parse_args()


def main() -> None:
    visitor = MJCFVisitor()
    visitor.set_verbose()
    node = visitor.load_file(args.file)
    print(node)


if __name__ == "__main__":
    main()
