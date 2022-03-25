"""
This file expands all mujoco include nodes (i.e., it flatten the file).

This can be a useful utility for debugging.
"""

import argparse

from lisdf.parsing.mjcf import MJCFVisitorFlatten

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("file")
args = parser.parse_args()


def main() -> None:
    visitor = MJCFVisitorFlatten()
    node = visitor.load_file(args.file)
    print(node)


if __name__ == "__main__":
    main()
