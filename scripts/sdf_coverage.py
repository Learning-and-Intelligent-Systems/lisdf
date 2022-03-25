"""
This file tests the coverage of the SDF parser.

In the SDF parser, each visitor method has an assertion
that ensures that all the attributes and children have been
successfully processed.

The `visitor.set_verbose()` method enables detailed logging
of the current node being processed. Thus, you can use this file
to test whether all attributes in your SDF file can be parsed by
our parser.
"""

import argparse

from lisdf.parsing.sdf_j import SDFVisitor
from lisdf.parsing.xml_j.xml import load_file

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("file")
args = parser.parse_args()


def main() -> None:
    node = load_file(args.file)
    print(node)
    print("-" * 120)
    visitor = SDFVisitor()
    visitor.set_verbose()
    visitor.load_file(args.file)
    print(
        "Coverage test passed."
        "(If you see this message, it means the parser has covered all the"
        "attributes/nodes in your file.)"
    )


if __name__ == "__main__":
    main()
