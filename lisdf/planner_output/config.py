from lisdf.planner_output.command import ActuateGripper, JointSpacePath

SUPPORTED_PLANNER_OUTPUT_VERSIONS = {"0.1"}

SUPPORTED_COMMAND_TYPES = (JointSpacePath, ActuateGripper)

# Whether the validation checks should enforce dimensionalities
# between waypoint in a JointSpacePath and all the commands in
# an LISDFPlan. Keeping as a flag for now.
ENFORCE_JOINT_DIMENSIONALITIES: bool = True
