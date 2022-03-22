import os

from lisdf.sdf import SDF


def load_sdf(model_filename):
    sdf_examples_dir = "models"
    sdf_path = os.path.join(sdf_examples_dir, model_filename)
    with open(sdf_path) as f:
        sdf = SDF.from_xml_string(f.read())
        for wi, world in enumerate(sdf.aggregate_order):
            for include in world.includes:
                include_model = load_sdf(include.model_name)
                sdf.aggregate_order[wi].models.append(include_model)

    return sdf


if __name__ == "__main__":
    sdf_test = "mud_test.sdf"
    sdf_results = load_sdf(sdf_test)
    print(sdf_results)
