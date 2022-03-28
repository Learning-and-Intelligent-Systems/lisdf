from collections import Set

SUPPORTED_PLANNER_OUTPUT_VERSIONS: Set[str] = {"0.1"}
CURRENT_VERSION: str = "0.1"
assert CURRENT_VERSION in SUPPORTED_PLANNER_OUTPUT_VERSIONS

ENABLE_LISDF_PATH_CHECKING: bool = False

# Whether the validation checks should enforce dimensionalities
# between waypoint in a JointSpacePath and all the commands in
# an LISDFPlan. Keeping as a flag for now.
ENFORCE_JOINT_DIMENSIONALITIES: bool = True

DEFAULT_JSON_INDENT: int = 2
