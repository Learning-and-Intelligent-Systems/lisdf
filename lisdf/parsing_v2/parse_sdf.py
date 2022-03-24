from lisdf.parsing_v2.sdf import load_sdf

if __name__ == "__main__":  # pragma: no cover
    sdf_test = "mud_test.sdf"
    sdf_results = load_sdf(sdf_test)
    print(sdf_results)
