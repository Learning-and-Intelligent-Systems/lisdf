import os

import lisdf_models
import pytest


@pytest.fixture(scope="session")
def models_dir() -> str:
    """Determine models directory for the lisdf-models"""
    # e.g./home/user/lisdf/lisdf-models/models
    lisdf_models_dir = os.path.dirname(lisdf_models.__file__)

    # Check if lisdf-models actually exists
    if not os.path.exists(lisdf_models_dir):
        raise FileNotFoundError(
            f"Could not find lisdf-models submodule at {lisdf_models_dir}. "
            "Check you pulled the submodule!"
        )
    return lisdf_models_dir
