#! /usr/bin/env python3
# -*- coding: utf-8 -*-
# File   : sdf-coverage.py
# Author : Jiayuan Mao
# Email  : maojiayuan@gmail.com
# Date   : 03/23/2022
#
# This file is part of lisdf.
# Distributed under terms of the MIT license.

import argparse

from lisdf.parsing.sdf_j import SDFVisitor
from lisdf.parsing.xml_j.xml import load_file

parser = argparse.ArgumentParser()
parser.add_argument("file")
args = parser.parse_args()


def main() -> None:
    node = load_file(args.file)
    print(node)
    print("-" * 120)
    visitor = SDFVisitor()
    visitor.set_verbose()
    node = visitor.load_file(args.file)
    print("Coverage test passed.")


if __name__ == "__main__":
    main()
