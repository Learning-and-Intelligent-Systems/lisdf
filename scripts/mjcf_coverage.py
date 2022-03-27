"""
This file tests the coverage of the MJCF Parser.

There are two types of checks for MJCF coverage:

    1.  In the MJCF parser, most visitor method has an assertion
    that ensures that all the attributes and children have been
    successfully processed.

    2. At the end of processing, it outputs an XML node that contains
    all attributes that haven't been processed. So you can take a look
    at that to see if there are any important missing features.
    (Most of them should be rendering-related, such as lighting.)

If an error occurs during parsing, the `visitor.set_verbose()` method
enables detailed logging of the current node being processed.
Thus, you can use this file to test whether all attributes in your SDF
file can be parsed by our parser.
"""

import argparse

from lisdf.parsing.mjcf import MJCFVisitor

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("file")
args = parser.parse_args()


def main() -> None:
    visitor = MJCFVisitor()
    visitor.set_verbose()
    node = visitor.load_file(args.file)
    print("-" * 80)
    print(node)


if __name__ == "__main__":
    main()
