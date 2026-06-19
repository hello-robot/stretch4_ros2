#pragma once

#include "stretch_core/robot_self_filter.hpp"

#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <string>
#include <vector>

namespace stretch_core
{

inline void requireArraySize(const std::string & name, size_t actual, size_t expected)
{
  if (actual != expected) {
    throw std::runtime_error(
      name + " must contain exactly " + std::to_string(expected) +
      " entries for the generated URDF self-filter boxes, got " +
      std::to_string(actual));
  }
}

inline std::vector<float> loadRequiredFloatArrayParameter(
  rclcpp::Node & node,
  const std::string & name,
  size_t count)
{
  const auto raw = node.get_parameter(name).as_double_array();
  requireArraySize(name, raw.size(), count);
  std::vector<float> values;
  values.reserve(raw.size());
  for (double v : raw) {
    values.push_back(static_cast<float>(v));
  }
  return values;
}

inline std::vector<std::string> loadRequiredStringArrayParameter(
  rclcpp::Node & node,
  const std::string & name,
  size_t count)
{
  const auto raw = node.get_parameter(name).as_string_array();
  requireArraySize(name, raw.size(), count);
  return raw;
}

inline std::vector<float> loadOptionalFloatArrayParameter(
  rclcpp::Node & node,
  const std::string & name)
{
  if (!node.has_parameter(name)) {
    return {};
  }
  const auto raw = node.get_parameter(name).as_double_array();
  if (raw.empty()) {
    return {};
  }
  std::vector<float> values;
  values.reserve(raw.size());
  for (double v : raw) {
    values.push_back(static_cast<float>(v));
  }
  return values;
}

inline void validateSelfFilterBoxArrays(const RobotSelfFilterConfig & config)
{
  const size_t n = config.self_filter_box_frames.size();
  if (n == 0) {
    throw std::runtime_error(
      "URDF self-filter boxes are required. Launch through self_filter_config.py "
      "so self_filter_box_* geometry is generated from the URDF.");
  }

  requireArraySize("self_filter_box_names", config.self_filter_box_names.size(), n);
  requireArraySize("self_filter_box_groups", config.self_filter_box_groups.size(), n);
  requireArraySize("self_filter_box_origin_x", config.self_filter_box_origin_x.size(), n);
  requireArraySize("self_filter_box_origin_y", config.self_filter_box_origin_y.size(), n);
  requireArraySize("self_filter_box_origin_z", config.self_filter_box_origin_z.size(), n);
  requireArraySize("self_filter_box_rpy_roll", config.self_filter_box_rpy_roll.size(), n);
  requireArraySize("self_filter_box_rpy_pitch", config.self_filter_box_rpy_pitch.size(), n);
  requireArraySize("self_filter_box_rpy_yaw", config.self_filter_box_rpy_yaw.size(), n);
  requireArraySize("self_filter_box_half_extents_x", config.self_filter_box_half_extents_x.size(), n);
  requireArraySize("self_filter_box_half_extents_y", config.self_filter_box_half_extents_y.size(), n);
  requireArraySize("self_filter_box_half_extents_z", config.self_filter_box_half_extents_z.size(), n);
  if (!config.self_filter_box_buffers.empty()) {
    requireArraySize("self_filter_box_buffers", config.self_filter_box_buffers.size(), n);
  }
  if (!config.self_filter_box_footprint_buffers.empty()) {
    requireArraySize(
      "self_filter_box_footprint_buffers", config.self_filter_box_footprint_buffers.size(), n);
  }
}

inline void declareRobotSelfFilterParameters(rclcpp::Node & node)
{
  node.declare_parameter("base_radius", 0.25);
  node.declare_parameter<std::string>("arm_line_start_frame", "arm_l0_link");
  node.declare_parameter<std::string>("arm_line_start_height_frame", "lift_link");
  node.declare_parameter("arm_line_height_offset_z", 0.0);
  node.declare_parameter<std::string>("arm_line_end_frame", "wrist_link");
  node.declare_parameter("arm_filter_radius", 0.07);
  node.declare_parameter("arm_filter_radius_buffer", 0.02);

  node.declare_parameter("self_filter_box_frames", std::vector<std::string>{});
  node.declare_parameter("self_filter_box_names", std::vector<std::string>{});
  node.declare_parameter("self_filter_box_groups", std::vector<std::string>{});
  node.declare_parameter("self_filter_box_origin_x", std::vector<double>{});
  node.declare_parameter("self_filter_box_origin_y", std::vector<double>{});
  node.declare_parameter("self_filter_box_origin_z", std::vector<double>{});
  node.declare_parameter("self_filter_box_rpy_roll", std::vector<double>{});
  node.declare_parameter("self_filter_box_rpy_pitch", std::vector<double>{});
  node.declare_parameter("self_filter_box_rpy_yaw", std::vector<double>{});
  node.declare_parameter("self_filter_box_half_extents_x", std::vector<double>{});
  node.declare_parameter("self_filter_box_half_extents_y", std::vector<double>{});
  node.declare_parameter("self_filter_box_half_extents_z", std::vector<double>{});
  node.declare_parameter("self_filter_arm_buffer", 0.04);
  node.declare_parameter("self_filter_wrist_buffer", 0.025);
  node.declare_parameter("self_filter_gripper_cam_buffer", 0.025);
  node.declare_parameter("self_filter_tool_buffer", 0.025);
  node.declare_parameter("self_filter_box_buffers", std::vector<double>{});
  node.declare_parameter("self_filter_box_footprint_buffers", std::vector<double>{});

  node.declare_parameter("publish_raw_urdf_self_filter_markers", false);
  node.declare_parameter("publish_buffered_self_filter_markers", true);
  node.declare_parameter<std::string>("resolved_tool_preset", "unknown");
  node.declare_parameter("self_filter_tf_timeout_sec", 0.05);
  node.declare_parameter("self_filter_spatial_gate_enabled", true);
  node.declare_parameter("self_filter_gate_radius_m", 1.5);
  node.declare_parameter("self_filter_gate_z_min_m", -0.05);
  node.declare_parameter("self_filter_gate_z_max_m", 1.6);
}

inline RobotSelfFilterConfig loadRobotSelfFilterConfig(rclcpp::Node & node)
{
  RobotSelfFilterConfig config;
  config.base_radius = static_cast<float>(node.get_parameter("base_radius").as_double());
  config.arm_line_start_frame = node.get_parameter("arm_line_start_frame").as_string();
  config.arm_line_start_height_frame =
    node.get_parameter("arm_line_start_height_frame").as_string();
  config.arm_line_height_offset_z = static_cast<float>(
    node.get_parameter("arm_line_height_offset_z").as_double());
  config.arm_line_end_frame = node.get_parameter("arm_line_end_frame").as_string();
  config.arm_filter_radius =
    static_cast<float>(node.get_parameter("arm_filter_radius").as_double());
  config.arm_filter_radius_buffer =
    static_cast<float>(node.get_parameter("arm_filter_radius_buffer").as_double());

  config.self_filter_box_frames = node.get_parameter("self_filter_box_frames").as_string_array();
  const size_t box_count = config.self_filter_box_frames.size();
  if (box_count == 0) {
    throw std::runtime_error(
      "URDF self-filter boxes are required. Launch through self_filter_config.py "
      "so self_filter_box_* geometry is generated from the URDF.");
  }
  config.self_filter_box_names =
    loadRequiredStringArrayParameter(node, "self_filter_box_names", box_count);
  config.self_filter_box_groups =
    loadRequiredStringArrayParameter(node, "self_filter_box_groups", box_count);
  config.self_filter_box_origin_x =
    loadRequiredFloatArrayParameter(node, "self_filter_box_origin_x", box_count);
  config.self_filter_box_origin_y =
    loadRequiredFloatArrayParameter(node, "self_filter_box_origin_y", box_count);
  config.self_filter_box_origin_z =
    loadRequiredFloatArrayParameter(node, "self_filter_box_origin_z", box_count);
  config.self_filter_box_rpy_roll =
    loadRequiredFloatArrayParameter(node, "self_filter_box_rpy_roll", box_count);
  config.self_filter_box_rpy_pitch =
    loadRequiredFloatArrayParameter(node, "self_filter_box_rpy_pitch", box_count);
  config.self_filter_box_rpy_yaw =
    loadRequiredFloatArrayParameter(node, "self_filter_box_rpy_yaw", box_count);
  config.self_filter_box_half_extents_x =
    loadRequiredFloatArrayParameter(node, "self_filter_box_half_extents_x", box_count);
  config.self_filter_box_half_extents_y =
    loadRequiredFloatArrayParameter(node, "self_filter_box_half_extents_y", box_count);
  config.self_filter_box_half_extents_z =
    loadRequiredFloatArrayParameter(node, "self_filter_box_half_extents_z", box_count);
  config.self_filter_arm_buffer =
    static_cast<float>(node.get_parameter("self_filter_arm_buffer").as_double());
  config.self_filter_wrist_buffer =
    static_cast<float>(node.get_parameter("self_filter_wrist_buffer").as_double());
  config.self_filter_gripper_cam_buffer =
    static_cast<float>(node.get_parameter("self_filter_gripper_cam_buffer").as_double());
  config.self_filter_tool_buffer =
    static_cast<float>(node.get_parameter("self_filter_tool_buffer").as_double());
  config.self_filter_box_buffers = loadOptionalFloatArrayParameter(node, "self_filter_box_buffers");
  config.self_filter_box_footprint_buffers =
    loadOptionalFloatArrayParameter(node, "self_filter_box_footprint_buffers");
  validateSelfFilterBoxArrays(config);

  config.publish_raw_urdf_self_filter_markers =
    node.get_parameter("publish_raw_urdf_self_filter_markers").as_bool();
  config.publish_buffered_self_filter_markers =
    node.get_parameter("publish_buffered_self_filter_markers").as_bool();
  config.resolved_tool_preset = node.get_parameter("resolved_tool_preset").as_string();
  config.tf_timeout_sec = node.get_parameter("self_filter_tf_timeout_sec").as_double();
  config.self_filter_spatial_gate_enabled =
    node.get_parameter("self_filter_spatial_gate_enabled").as_bool();
  config.self_filter_gate_radius_m =
    static_cast<float>(node.get_parameter("self_filter_gate_radius_m").as_double());
  config.self_filter_gate_z_min_m =
    static_cast<float>(node.get_parameter("self_filter_gate_z_min_m").as_double());
  config.self_filter_gate_z_max_m =
    static_cast<float>(node.get_parameter("self_filter_gate_z_max_m").as_double());
  return config;
}

inline bool isRobotSelfFilterParameter(const std::string & name)
{
  return name == "base_radius" ||
         name == "arm_line_start_frame" || name == "arm_line_start_height_frame" ||
         name == "arm_line_height_offset_z" ||
         name == "arm_line_end_frame" ||
         name == "arm_filter_radius" ||
         name == "arm_filter_radius_buffer" ||
         name == "self_filter_box_frames" ||
         name == "self_filter_box_names" || name == "self_filter_box_groups" ||
         name == "self_filter_box_origin_x" || name == "self_filter_box_origin_y" ||
         name == "self_filter_box_origin_z" ||
         name == "self_filter_box_rpy_roll" || name == "self_filter_box_rpy_pitch" ||
         name == "self_filter_box_rpy_yaw" ||
         name == "self_filter_box_half_extents_x" || name == "self_filter_box_half_extents_y" ||
         name == "self_filter_box_half_extents_z" ||
         name == "self_filter_arm_buffer" ||
         name == "self_filter_wrist_buffer" ||
         name == "self_filter_gripper_cam_buffer" ||
         name == "self_filter_tool_buffer" ||
         name == "self_filter_box_buffers" ||
         name == "self_filter_box_footprint_buffers" ||
         name == "publish_raw_urdf_self_filter_markers" ||
         name == "publish_buffered_self_filter_markers" ||
         name == "resolved_tool_preset" ||
         name == "self_filter_tf_timeout_sec" ||
         name == "self_filter_spatial_gate_enabled" ||
         name == "self_filter_gate_radius_m" ||
         name == "self_filter_gate_z_min_m" ||
         name == "self_filter_gate_z_max_m";
}

}  // namespace stretch_core
