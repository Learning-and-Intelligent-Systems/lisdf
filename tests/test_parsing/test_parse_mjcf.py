import os.path as osp

from lisdf.parsing.mjcf import load_mjcf


def test_mjcf_parsing(models_dir):
    # TODO(Jiayuan Mao @ 03/24): add more assertions.
    filename = osp.join(
        models_dir,
        "mjcf",
        "sawyer_assembly_peg.xml",
    )
    _ = load_mjcf(filename)
