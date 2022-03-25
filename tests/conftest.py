import os

import pytest

_LISDF_MODELS = os.path.join("lisdf-models", "models")


@pytest.fixture(scope="session")
def models_dir() -> str:
    """Determine models directory for the lisdf-models submodule"""
    # e.g. /home/user/lisdf/tests
    test_dir = os.path.dirname(__file__)

    # e.g. /home/user/lisdf
    lisdf_dir = os.path.dirname(test_dir)

    # e.g./home/user/lisdf/lisdf-models/models
    lisdf_models_dir = os.path.join(lisdf_dir, _LISDF_MODELS)

    # Check if lisdf-models actually exists
    if not os.path.exists(lisdf_models_dir):
        raise FileNotFoundError(
            f"Could not find lisdf-models submodule at {lisdf_models_dir}. "
            "Check you pulled the submodule!"
        )
    return lisdf_models_dir
