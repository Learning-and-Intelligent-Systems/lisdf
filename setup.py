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
    include_package_data=True,
    package_data={"lisdf": ["py.typed"]},  # for mypy
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
        ]
    },
)
