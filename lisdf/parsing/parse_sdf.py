import os
from typing import Union

from lisdf.parsing.sdf import SDF, Collision, Link, Mesh, Visual
from lisdf.parsing.urdf import Robot as URDF


def _handle_component(component, model_path: str) -> None:
    """
    Handle component and inject URI into link component geometry
    """
    if isinstance(component, Link):
        for link_component in component.aggregate_order:
            # TODO: are there other types we need to consider?
            if isinstance(link_component, (Collision, Visual)) and isinstance(
                link_component.geometry, Mesh
            ):
                link_component.geometry.uri = os.path.join(
                    model_path, link_component.geometry.uri
                )


def inject_absolute_path(sdf: SDF, model_path: str) -> SDF:
    """
    This function replace relative paths to object and material
    files with absolute paths so the sdf object is self-contained.
    """
    for world in sdf.aggregate_order:
        for model in world.models:
            for component in model.aggregate_order:
                _handle_component(component, model_path)
    return sdf


def load_urdf(model_name: str, models_dir: str) -> URDF:
    urdf_path = os.path.join(models_dir, model_name)
    with open(urdf_path) as f:
        xml_str = f.read()
        urdf = URDF.from_xml_string(xml_str)
    return urdf


def load_sdf(model_name: str, models_dir: str) -> SDF:
    sdf_path = os.path.join(models_dir, model_name)
    with open(sdf_path) as f:
        xml_str = f.read()
        sdf = SDF.from_xml_string(xml_str)
        for world in sdf.aggregate_order:
            # Load all the includes in the world
            for include in world.includes:
                include_model: Union[SDF, URDF]
                if include.uri.endswith("urdf"):
                    include_model = load_urdf(include.uri, models_dir=models_dir)
                else:
                    include_model = load_sdf(include.model_name, models_dir=models_dir)

                world.models.append(include_model)
    model_path = os.path.join(models_dir, os.path.dirname(model_name))
    return inject_absolute_path(sdf, model_path)


if __name__ == "__main__":  # pragma: no cover
    import lisdf_models

    models_dir = os.path.dirname(lisdf_models.__file__)
    sdf_test = "mud_test/model.sdf"
    sdf_results = load_sdf(sdf_test, models_dir=models_dir)
    # print(sdf_results)
