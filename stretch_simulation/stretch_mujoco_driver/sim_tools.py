"""The tools the simulated robot can be built with, shared by the sim driver and its
launch file (which needs the tool to build the matching URDF)."""

from stretch4_mujoco import Stretch4MujocoSimulator

DEFAULT_SIM_TOOL = "eoa_wrist_dw4_tool_sg4"

# Short names accepted for the `tool` parameter / launch argument.
SIM_TOOL_SHORT_NAMES = {
    "sg4": DEFAULT_SIM_TOOL,
    "pg4": Stretch4MujocoSimulator.PARALLEL_GRIPPER_TOOL_NAME,
}

# Per tool: its stretch4_body joint name (the sim's end_of_arm attribute) and its URDF
# joints, primary (the one gripper_joint reports) first.
SIM_TOOLS = {
    DEFAULT_SIM_TOOL: {
        "backend": "stretch_gripper",
        "tool_joints": ["gripper_finger_left_joint", "gripper_finger_right_joint"],
    },
    Stretch4MujocoSimulator.PARALLEL_GRIPPER_TOOL_NAME: {
        "backend": "parallel_gripper",
        "tool_joints": ["finger_left_joint", "finger_right_joint"],
    },
}


def resolve_sim_tool(tool):
    """Full tool name for `tool` ('sg4', 'pg4', a full name, or '' for the default)."""
    tool = (tool or "").strip()
    if not tool:
        return DEFAULT_SIM_TOOL
    tool = SIM_TOOL_SHORT_NAMES.get(tool.lower(), tool)
    if tool not in SIM_TOOLS:
        raise ValueError(f"Unknown sim tool '{tool}'; use one of "
                         f"{list(SIM_TOOL_SHORT_NAMES)} or {list(SIM_TOOLS)}.")
    return tool
