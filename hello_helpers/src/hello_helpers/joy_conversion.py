#!/usr/bin/env python3
from sensor_msgs.msg import Joy


stretch_gamepad_mapping = {
    'axes':{
        0:{'name':'left_stick_x', 'range':[-1,1]},
        1:{'name':'left_stick_y', 'range':[-1,1]},
        2:{'name':'right_stick_x', 'range':[-1,1]},
        3:{'name':'right_stick_y', 'range':[-1,1]},
        4:{'name':'left_trigger_pulled', 'range':[0,1]},
        5:{'name':'right_trigger_pulled', 'range':[0,1]}
        },
    'buttons':{
        0:{'name':'left_stick_button_pressed'},
        1:{'name':'right_stick_button_pressed'},
        2:{'name':'bottom_button_pressed'},
        3:{'name':'right_button_pressed'},
        4:{'name':'left_button_pressed'},
        5:{'name':'top_button_pressed'},
        6:{'name':'left_shoulder_button_pressed'},
        7:{'name':'right_shoulder_button_pressed'},
        8:{'name':'select_button_pressed'},
        9:{'name':'start_button_pressed'},
        10:{'name':'bottom_pad_pressed'},
        11:{'name':'top_pad_pressed'},
        12:{'name':'left_pad_pressed'},
        13:{'name':'right_pad_pressed'},
        }
    }

def get_default_gamepad_state():
    state = {'middle_led_ring_button_pressed': False, #This button should not be used
            'left_stick_x': 0,
            'left_stick_y': 0,
            'right_stick_x': 0,
            'right_stick_y': 0,
            'left_stick_button_pressed': False,
            'right_stick_button_pressed': False,
            'bottom_button_pressed': False,
            'top_button_pressed': False,
            'left_button_pressed': False,
            'right_button_pressed': False,
            'left_shoulder_button_pressed': False,
            'right_shoulder_button_pressed': False,
            'select_button_pressed': False,
            'start_button_pressed': False,
            'left_trigger_pulled': 0,
            'right_trigger_pulled': 0,
            'bottom_pad_pressed': False,
            'top_pad_pressed': False,
            'left_pad_pressed': False,
            'right_pad_pressed': False}
    return state

# Joy messages in the Stretch gamepad layout (stretch_gamepad_mapping) carry this frame_id, so
# a consumer can tell them from Joy messages in other layouts, such as the ROS joy package's
# (frame_id "joy"; stretch_core's joy_to_stretch_joy converts those).
STRETCH_JOY_FRAME_ID = "stretch_gamepad"
NUM_STRETCH_JOY_AXES = len(stretch_gamepad_mapping['axes'])
NUM_STRETCH_JOY_BUTTONS = len(stretch_gamepad_mapping['buttons'])


def get_default_joy_msg():
    """A Joy message in the Stretch gamepad layout, labeled as such, with nothing pressed."""
    out_msg = Joy()
    out_msg.header.frame_id = STRETCH_JOY_FRAME_ID
    out_msg.axes = [0.0]*NUM_STRETCH_JOY_AXES
    out_msg.buttons = [0]*NUM_STRETCH_JOY_BUTTONS
    return out_msg


def check_stretch_joy(joy_msg):
    """Whether joy_msg can be read as the Stretch gamepad layout, as (usable, warning).

    A message labeled with STRETCH_JOY_FRAME_ID and of the right size is usable with no
    warning. An unlabeled one of the right size is usable, with a warning, since publishers
    that build their own Joy messages predate the label. One of the wrong size isn't usable;
    the warning says where it seems to come from."""
    frame_id = joy_msg.header.frame_id
    right_size = (len(joy_msg.axes) == NUM_STRETCH_JOY_AXES
                  and len(joy_msg.buttons) == NUM_STRETCH_JOY_BUTTONS)
    size = (f"{len(joy_msg.axes)} axes and {len(joy_msg.buttons)} buttons (the Stretch "
            f"layout has {NUM_STRETCH_JOY_AXES} and {NUM_STRETCH_JOY_BUTTONS})")
    if frame_id == STRETCH_JOY_FRAME_ID:
        if right_size:
            return True, None
        return False, f"Ignoring a Joy message labeled '{STRETCH_JOY_FRAME_ID}' that has {size}."
    if right_size:
        return True, (f"Joy message has frame_id '{frame_id}', not '{STRETCH_JOY_FRAME_ID}', so it "
                      f"isn't from one of the Stretch nodes; reading it as the Stretch gamepad "
                      f"layout since its size matches. Build Joy messages with "
                      f"hello_helpers.joy_conversion to label them.")
    source = (" It looks like the ROS joy package's layout; run stretch_core's "
              "joy_to_stretch_joy to convert it." if frame_id == "joy" else "")
    return False, (f"Ignoring a Joy message that isn't from one of the Stretch nodes (frame_id "
                   f"'{frame_id}', not '{STRETCH_JOY_FRAME_ID}'): it has {size}.{source}")

def unpack_joy_to_gamepad_state(joy_msg):
    stretch_gamepad_state = get_default_gamepad_state()
    if len(joy_msg.axes)!= NUM_STRETCH_JOY_AXES:
        print("Invalid Joy message Axes length received.")
        return stretch_gamepad_state
    if len(joy_msg.buttons)!= NUM_STRETCH_JOY_BUTTONS:
        print("Invalid Joy message Buttons length received.")
        return stretch_gamepad_state
    
    for i in range(len(joy_msg.axes)):
        name = stretch_gamepad_mapping['axes'][i]['name']
        r = stretch_gamepad_mapping['axes'][i]['range']
        value = joy_msg.axes[i]
        value = value if is_value_between_bounds(value,r) else 0
        stretch_gamepad_state[name] = value
        
    for i in range(len(joy_msg.buttons)):
        name = stretch_gamepad_mapping['buttons'][i]['name']
        value = True if joy_msg.buttons[i]==1 else False
        stretch_gamepad_state[name] = value
    return stretch_gamepad_state

def unpack_gamepad_state_to_joy(gamepad_state):
    msg = get_default_joy_msg()
    
    msg.axes[0] = gamepad_state['left_stick_x']
    msg.axes[1] = gamepad_state['left_stick_y']
    msg.axes[2] = gamepad_state['right_stick_x']
    msg.axes[3] = gamepad_state['right_stick_y']
    msg.axes[4] = gamepad_state['left_trigger_pulled']
    msg.axes[5] = gamepad_state['right_trigger_pulled']
    
    msg.buttons[0] = 1 if gamepad_state['left_stick_button_pressed'] else 0
    msg.buttons[1] = 1 if gamepad_state['right_stick_button_pressed'] else 0
    msg.buttons[2] = 1 if gamepad_state['bottom_button_pressed'] else 0
    msg.buttons[3] = 1 if gamepad_state['right_button_pressed'] else 0
    msg.buttons[4] = 1 if gamepad_state['left_button_pressed'] else 0
    msg.buttons[5] = 1 if gamepad_state['top_button_pressed'] else 0
    msg.buttons[6] = 1 if gamepad_state['left_shoulder_button_pressed'] else 0
    msg.buttons[7] = 1 if gamepad_state['right_shoulder_button_pressed'] else 0
    msg.buttons[8] = 1 if gamepad_state['select_button_pressed'] else 0
    msg.buttons[9] = 1 if gamepad_state['start_button_pressed'] else 0
    msg.buttons[10] = 1 if gamepad_state['bottom_pad_pressed'] else 0
    msg.buttons[11] = 1 if gamepad_state['top_pad_pressed'] else 0
    msg.buttons[12] = 1 if gamepad_state['left_pad_pressed'] else 0
    msg.buttons[13] = 1 if gamepad_state['right_pad_pressed'] else 0
    
    return msg
        
        
def is_value_between_bounds(value, r):
    return r[0] <= value <= r[1]


class UnsupportedToolError(Exception):
    pass

def get_Idx(tool_name):
    """
    Returns the Idx class corresponding to the tool name.
    """
    if tool_name == 'eoa_wrist_dw4_tool_sg4':
        return SE4_dw4_sg4_Idx
    elif tool_name == 'eoa_wrist_dw4_tool_pg4':
        return SE4_dw4_pg4_Idx
    elif tool_name == 'eoa_wrist_dw4_tool_nil':
        return eoa_wrist_dw4_tool_nil_Idx
    elif tool_name == 'eoa_wrist_dw4_tool_tablet_12in':
        return eoa_wrist_dw4_tool_tablet_12in_Idx
    elif tool_name == 'tool_stretch_dex_wrist':
        return tool_stretch_dex_wrist_Idx
    elif tool_name == 'tool_stretch_gripper':
        return tool_stretch_gripper_Idx
    elif tool_name == 'tool_none':
        return tool_none_Idx
    else:
        raise UnsupportedToolError('Undefined tool name in QposConversion.')

class SE4_dw4_sg4_Idx:
    LIFT = 1
    ARM = 0
    GRIPPER = 7
    WRIST_ROLL = 4
    WRIST_PITCH = 3
    WRIST_YAW = 2
    BASE_TRANSLATE = 5
    BASE_ROTATE = 6

    num_joints = 8

class SE4_dw4_pg4_Idx:
    LIFT = 1
    ARM = 0
    GRIPPER = 7
    WRIST_ROLL = 4
    WRIST_PITCH = 3
    WRIST_YAW = 2
    BASE_TRANSLATE = 5
    BASE_ROTATE = 6

    num_joints = 8

class eoa_wrist_dw4_tool_nil_Idx:
    LIFT = 1
    ARM = 0
    WRIST_ROLL = 4
    WRIST_PITCH = 3
    WRIST_YAW = 2
    BASE_TRANSLATE = 5
    BASE_ROTATE = 6

    num_joints = 7

class eoa_wrist_dw4_tool_tablet_12in_Idx:
    LIFT = 1
    ARM = 0
    WRIST_ROLL = 4
    WRIST_PITCH = 3
    WRIST_YAW = 2
    BASE_TRANSLATE = 5
    BASE_ROTATE = 6

    num_joints = 7

class tool_stretch_dex_wrist_Idx:
    LIFT = 1
    ARM = 0
    GRIPPER = 5
    WRIST_ROLL = 4
    WRIST_PITCH = 3
    WRIST_YAW = 2
    BASE_TRANSLATE = 6
    BASE_ROTATE = 7

    num_joints = 8

class tool_stretch_gripper_Idx:
    LIFT = 1
    ARM = 0
    GRIPPER = 3
    WRIST_YAW = 2
    BASE_TRANSLATE = 4
    BASE_ROTATE = 5

    num_joints = 6

class tool_none_Idx:
    LIFT = 1
    ARM = 0
    WRIST_YAW = 2
    BASE_TRANSLATE = 3
    BASE_ROTATE = 4

    num_joints = 5

class JointStateMapping:
    """
    Mapping of ROS joint names to the corresponding joint names in the robot.
    """
    ROS_ARM_JOINTS = ["arm_l4_joint", "arm_l3_joint", "arm_l2_joint", "arm_l1_joint"]
    ROS_LIFT_JOINT = "lift_joint"
    ROS_GRIPPER_FINGER = "gripper_finger_left_joint"
    ROS_WRIST_YAW = "wrist_yaw_joint"
    ROS_WRIST_PITCH = "wrist_pitch_joint"
    ROS_WRIST_ROLL = "wrist_roll_joint"