"""Setup script."""
from setuptools import find_packages, setup

setup(
    name="lisdf",
    version="0.0.0",
    packages=find_packages(include=["lisdf", "lisdf.*"]),
    install_requires=[
        "pyyaml",
        "numpy",
        "lark",
    ],
    package_data={"lisdf": ["lisdf/**", "py.typed"]},  # mypy, pddl files
    extras_require={
        "develop": [
            # Formatting
            "black",
            "isort",
            # Linting and type checking
            "flake8",
            "mypy",
            # Typing stubs for mypy
            "types-PyYAML",
            "types-mock",
            # Testing
            "pytest",
            "pytest-cov",
            "mock",
            # LISdf models
            "lisdf_models@git+https://github.com/Learning-and-Intelligent-Systems/lisdf-models.git",  # noqa: E501
        ]
    },
)
