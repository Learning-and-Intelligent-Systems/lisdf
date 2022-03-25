import os.path as osp

from lisdf.parsing.mjcf import load_mjcf


def test_mjcf_parsing():
    # TODO(Jiayuan Mao @ 03/24): add more assertions.
    filename = osp.join(
        osp.dirname(osp.dirname(osp.dirname(__file__))),
        "models",
        "mjcf",
        "sawyer_assembly_peg.xml",
    )
    _ = load_mjcf(filename)
