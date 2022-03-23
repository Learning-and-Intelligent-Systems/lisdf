import os

from lisdf.parsing.sdf import SDF, Collision, Link, Mesh, Visual


def inject_absolute_path(sdf: SDF, model_path: str):

    """
    This function replace relative paths to object and material
    files with absolute paths so the sdf object is self-contained.
    """
    for wi, world in enumerate(sdf.aggregate_order):
        for model in world.models:
            for component in model.aggregate_order:
                if isinstance(component, Link):
                    for link_component in component.aggregate_order:
                        if isinstance(link_component, Collision):
                            if (
                                "geometry" in link_component.__dict__.keys()
                                and isinstance(link_component.geometry, Mesh)
                            ):
                                link_component.geometry.uri = os.path.join(
                                    model_path, link_component.geometry.uri
                                )
                        if isinstance(link_component, Visual):
                            if (
                                "geometry" in link_component.__dict__.keys()
                                and isinstance(link_component.geometry, Mesh)
                            ):
                                link_component.geometry.uri = os.path.join(
                                    model_path, link_component.geometry.uri
                                )

    return sdf


def load_sdf(model_directory: str):
    sdf_path = os.path.join("models", model_directory)
    with open(sdf_path) as f:
        xml_str = f.read()
        sdf = SDF.from_xml_string(xml_str)
        for wi, world in enumerate(sdf.aggregate_order):
            for include in world.includes:
                include_model = load_sdf(include.model_name)
                sdf.aggregate_order[wi].models.append(include_model)
    return inject_absolute_path(sdf, model_directory)


if __name__ == "__main__":  # pragma: no cover
    sdf_test = "mud_test"
    sdf_results = load_sdf(sdf_test)
    print(sdf_results)
