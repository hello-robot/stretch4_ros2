#! /usr/bin/env python3
"""Converts Joy messages from another layout (by default the ROS joy package's
game_controller_node) to the Stretch gamepad layout that the Stretch drivers read
(hello_helpers.joy_conversion), labeled as from a Stretch node.

    ros2 run joy game_controller_node --ros-args -r joy:=joy_standard
    ros2 run stretch_core joy_to_stretch_joy

Each Stretch input names its source in the incoming message with a parameter:

    axes.<name>      source of a stick or trigger, plus axes.<name>.scale and .offset
                     (output = scale * source + offset, clamped to the Stretch range)
    buttons.<name>   source of a button

A source is "button:<i>", "axis:<i>", "axis:<i>:><threshold>" or "axis:<i>:<<threshold>"
(an axis read as a button, e.g. a d-pad reported as axes: "axis:7:>0.5"), or "" for none.
The names are the Stretch gamepad state's (hello_helpers.joy_conversion). The defaults are
for game_controller_node; see DEFAULT_SOURCES.
"""
import re

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from sensor_msgs.msg import Joy

import hello_helpers.joy_conversion as jc

# The ROS joy package's game_controller_node (SDL2's game controller layout): axes left x,
# left y, right x, right y, left trigger, right trigger; buttons A, B, X, Y, Back, Guide,
# Start, left stick, right stick, left shoulder, right shoulder, d-pad up, down, left, right,
# then extra buttons. It reports each axis negated (up and left positive, a pulled trigger
# toward -1); the Stretch layout has right and up positive and triggers from 0 to 1.
# Checked with the Stretch gamepad (an X360-style pad) on 2026-11-23.
DEFAULT_SOURCES = {
    'axes': {
        'left_stick_x': ('axis:0', -1.0, 0.0),
        'left_stick_y': ('axis:1', 1.0, 0.0),
        'right_stick_x': ('axis:2', -1.0, 0.0),
        'right_stick_y': ('axis:3', 1.0, 0.0),
        'left_trigger_pulled': ('axis:4', -1.0, 0.0),
        'right_trigger_pulled': ('axis:5', -1.0, 0.0),
    },
    'buttons': {
        'bottom_button_pressed': 'button:0',  # A
        'right_button_pressed': 'button:1',  # B
        'left_button_pressed': 'button:2',  # X
        'top_button_pressed': 'button:3',  # Y
        'select_button_pressed': 'button:4',  # Back
        'start_button_pressed': 'button:6',  # Start
        'left_stick_button_pressed': 'button:7',
        'right_stick_button_pressed': 'button:8',
        'left_shoulder_button_pressed': 'button:9',
        'right_shoulder_button_pressed': 'button:10',
        'top_pad_pressed': 'button:11',
        'bottom_pad_pressed': 'button:12',
        'left_pad_pressed': 'button:13',
        'right_pad_pressed': 'button:14',
    },
}

SOURCE_PATTERN = re.compile(r'^(button|axis):(\d+)(?::([<>])(-?\d+(?:\.\d+)?))?$')


def parse_source(spec):
    """A source spec as (kind, index, comparison, threshold), or None for "". Raises
    ValueError if it isn't one of the forms in the module docstring."""
    if spec == '':
        return None
    match = SOURCE_PATTERN.match(spec)
    if match is None or (match.group(1) == 'button' and match.group(3)):
        raise ValueError(f"'{spec}' isn't \"button:<i>\", \"axis:<i>\", \"axis:<i>:><threshold>\", "
                         f"\"axis:<i>:<<threshold>\" or \"\"")
    kind, index, comparison, threshold = match.groups()
    return kind, int(index), comparison, float(threshold) if threshold is not None else None


class JoyToStretchJoy(Node):
    def __init__(self):
        super().__init__('joy_to_stretch_joy')
        self.declare_parameter('input_topic', 'joy_standard', ParameterDescriptor(
            description="Joy topic to convert", read_only=True))
        self.declare_parameter('output_topic', 'joy', ParameterDescriptor(
            description="Joy topic for the Stretch gamepad layout", read_only=True))

        self.axis_ranges = {axis['name']: axis['range'] for axis in jc.stretch_gamepad_mapping['axes'].values()}
        for name, (spec, scale, offset) in DEFAULT_SOURCES['axes'].items():
            self.declare_parameter(f'axes.{name}', spec, ParameterDescriptor(
                description=f"Source of {name}: button:<i>, axis:<i>, or \"\""))
            self.declare_parameter(f'axes.{name}.scale', scale)
            self.declare_parameter(f'axes.{name}.offset', offset)
        for name, spec in DEFAULT_SOURCES['buttons'].items():
            self.declare_parameter(f'buttons.{name}', spec, ParameterDescriptor(
                description=f"Source of {name}: button:<i>, axis:<i>:>t, axis:<i>:<t, or \"\""))

        self.sources = {}  # parameter name -> parsed source
        for name in self._source_parameter_names():
            self.sources[name] = parse_source(self.get_parameter(name).value)  # bad overrides fail here
        self.add_on_set_parameters_callback(self.check_parameters)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.publisher = self.create_publisher(Joy, output_topic, 10)
        self.create_subscription(Joy, input_topic, self.convert, 10)
        self.get_logger().info(f"Converting Joy messages on {input_topic} to the Stretch gamepad "
                               f"layout on {output_topic}.")

    def _source_parameter_names(self):
        return ([f'axes.{name}' for name in DEFAULT_SOURCES['axes']]
                + [f'buttons.{name}' for name in DEFAULT_SOURCES['buttons']])

    def check_parameters(self, parameters):
        """Accept a new source only if it parses; remember the parsed form."""
        parsed = {}
        for parameter in parameters:
            if parameter.name in self.sources:
                try:
                    parsed[parameter.name] = parse_source(parameter.value)
                except (ValueError, TypeError) as e:
                    return SetParametersResult(successful=False, reason=f"{parameter.name}: {e}")
        self.sources.update(parsed)
        return SetParametersResult(successful=True)

    def _read(self, source, msg):
        """A source's value in msg: the axis value, or 1.0 / 0.0 for a pressed or released
        button or thresholded axis. None if msg doesn't have that axis or button."""
        if source is None:
            return 0.0
        kind, index, comparison, threshold = source
        values = msg.buttons if kind == 'button' else msg.axes
        if index >= len(values):
            return None
        value = float(values[index])
        if kind == 'button':
            return 1.0 if value else 0.0
        if comparison == '>':
            return 1.0 if value > threshold else 0.0
        if comparison == '<':
            return 1.0 if value < threshold else 0.0
        return value

    def convert(self, msg):
        state = jc.get_default_gamepad_state()
        missing = []
        for name in DEFAULT_SOURCES['axes']:
            value = self._read(self.sources[f'axes.{name}'], msg)
            if value is None:
                missing.append(f'axes.{name}')
                continue
            scaled = (self.get_parameter(f'axes.{name}.scale').value * value
                      + self.get_parameter(f'axes.{name}.offset').value)
            low, high = self.axis_ranges[name]
            state[name] = min(max(scaled, low), high)
        for name in DEFAULT_SOURCES['buttons']:
            value = self._read(self.sources[f'buttons.{name}'], msg)
            if value is None:
                missing.append(f'buttons.{name}')
                continue
            state[name] = value > 0.5
        if missing:
            self.get_logger().warn(
                f"The input Joy message has {len(msg.axes)} axes and {len(msg.buttons)} buttons, "
                f"too few for the sources of {', '.join(missing)}; those read as released.",
                throttle_duration_sec=10.0)

        out = jc.unpack_gamepad_state_to_joy(state)  # labeled as the Stretch layout
        out.header.stamp = msg.header.stamp
        self.publisher.publish(out)


def main():
    rclpy.init()
    node = JoyToStretchJoy()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
