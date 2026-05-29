#pragma once

#include "stretch_core/robot_self_filter.hpp"

#include <rclcpp/rclcpp.hpp>
#include <vector>

namespace stretch_core
{

inline void declareRobotSelfFilterParameters(rclcpp::Node & node)
{
  node.declare_parameter("filter_base", true);
  node.declare_parameter("base_radius", 0.25);
  node.declare_parameter("filter_arm", false);
  node.declare_parameter<std::string>("arm_line_start_frame", "arm_l0_link");
  node.declare_parameter<std::string>("arm_line_start_height_frame", "lift_link");
  node.declare_parameter("arm_line_height_offset_z", 0.0);
  node.declare_parameter<std::string>("arm_line_end_frame", "wrist_link");
  node.declare_parameter("arm_filter_radius", 0.07);
  node.declare_parameter("arm_filter_radius_buffer", 0.02);
  node.declare_parameter("filter_arm_shoulder", false);
  node.declare_parameter<std::string>("arm_shoulder_box_frame", "arm_l0_link");
  node.declare_parameter("arm_shoulder_box_origin_x", -0.06);
  node.declare_parameter("arm_shoulder_box_origin_y", 0.0);
  node.declare_parameter("arm_shoulder_box_origin_z", 0.0);
  node.declare_parameter("arm_shoulder_half_extents_x", 0.08);
  node.declare_parameter("arm_shoulder_half_extents_y", 0.10);
  node.declare_parameter("arm_shoulder_half_extents_z", 0.10);
  node.declare_parameter("arm_shoulder_rear_extent", 0.06);
  node.declare_parameter("arm_shoulder_base_overshoot", 0.04);
  node.declare_parameter("arm_shoulder_buffer", 0.02);
  node.declare_parameter("filter_wrist", false);
  node.declare_parameter(
    "wrist_chain_frames",
    std::vector<std::string>{
    "wrist_link", "wrist_yaw_link", "wrist_pitch_link", "wrist_roll_link",
    "tool_attachment_site_link"});
  node.declare_parameter("wrist_chain_link_radius", 0.06);
  node.declare_parameter("wrist_chain_buffer", 0.02);
  node.declare_parameter("filter_attachment", false);
  node.declare_parameter<std::string>("attachment_frame", "quick_connect_interface_link");
  node.declare_parameter("attachment_half_extents_x", 0.10);
  node.declare_parameter("attachment_half_extents_y", 0.08);
  node.declare_parameter("attachment_half_extents_z", 0.08);
  node.declare_parameter("attachment_buffer", 0.02);
  node.declare_parameter("self_filter_tf_timeout_sec", 0.05);
}

inline RobotSelfFilterConfig loadRobotSelfFilterConfig(rclcpp::Node & node)
{
  RobotSelfFilterConfig config;
  config.filter_base = node.get_parameter("filter_base").as_bool();
  config.base_radius = static_cast<float>(node.get_parameter("base_radius").as_double());
  config.filter_arm = node.get_parameter("filter_arm").as_bool();
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
  config.filter_arm_shoulder = node.get_parameter("filter_arm_shoulder").as_bool();
  config.arm_shoulder_box_frame = node.get_parameter("arm_shoulder_box_frame").as_string();
  config.arm_shoulder_box_origin_x =
    static_cast<float>(node.get_parameter("arm_shoulder_box_origin_x").as_double());
  config.arm_shoulder_box_origin_y =
    static_cast<float>(node.get_parameter("arm_shoulder_box_origin_y").as_double());
  config.arm_shoulder_box_origin_z =
    static_cast<float>(node.get_parameter("arm_shoulder_box_origin_z").as_double());
  config.arm_shoulder_half_extents_x =
    static_cast<float>(node.get_parameter("arm_shoulder_half_extents_x").as_double());
  config.arm_shoulder_half_extents_y =
    static_cast<float>(node.get_parameter("arm_shoulder_half_extents_y").as_double());
  config.arm_shoulder_half_extents_z =
    static_cast<float>(node.get_parameter("arm_shoulder_half_extents_z").as_double());
  config.arm_shoulder_rear_extent =
    static_cast<float>(node.get_parameter("arm_shoulder_rear_extent").as_double());
  config.arm_shoulder_base_overshoot =
    static_cast<float>(node.get_parameter("arm_shoulder_base_overshoot").as_double());
  config.arm_shoulder_buffer =
    static_cast<float>(node.get_parameter("arm_shoulder_buffer").as_double());
  config.filter_wrist = node.get_parameter("filter_wrist").as_bool();
  config.wrist_chain_frames = node.get_parameter("wrist_chain_frames").as_string_array();
  config.wrist_chain_link_radius =
    static_cast<float>(node.get_parameter("wrist_chain_link_radius").as_double());
  config.wrist_chain_buffer =
    static_cast<float>(node.get_parameter("wrist_chain_buffer").as_double());
  config.filter_attachment = node.get_parameter("filter_attachment").as_bool();
  config.attachment_frame = node.get_parameter("attachment_frame").as_string();
  config.attachment_half_extents_x =
    static_cast<float>(node.get_parameter("attachment_half_extents_x").as_double());
  config.attachment_half_extents_y =
    static_cast<float>(node.get_parameter("attachment_half_extents_y").as_double());
  config.attachment_half_extents_z =
    static_cast<float>(node.get_parameter("attachment_half_extents_z").as_double());
  config.attachment_buffer =
    static_cast<float>(node.get_parameter("attachment_buffer").as_double());
  config.tf_timeout_sec = node.get_parameter("self_filter_tf_timeout_sec").as_double();
  return config;
}

inline bool isRobotSelfFilterParameter(const std::string & name)
{
  return name == "filter_base" || name == "base_radius" || name == "filter_arm" ||
         name == "arm_line_start_frame" || name == "arm_line_start_height_frame" ||
         name == "arm_line_height_offset_z" || name == "arm_line_end_frame" || name == "arm_filter_radius" ||
         name == "arm_filter_radius_buffer" || name == "filter_arm_shoulder" ||
         name == "arm_shoulder_box_frame" || name == "arm_shoulder_box_origin_x" ||
         name == "arm_shoulder_box_origin_y" || name == "arm_shoulder_box_origin_z" ||
         name == "arm_shoulder_half_extents_x" || name == "arm_shoulder_half_extents_y" ||
         name == "arm_shoulder_half_extents_z" || name == "arm_shoulder_rear_extent" ||
         name == "arm_shoulder_base_overshoot" || name == "arm_shoulder_buffer" ||
         name == "filter_wrist" || name == "wrist_chain_frames" ||
         name == "wrist_chain_link_radius" || name == "wrist_chain_buffer" ||
         name == "filter_attachment" || name == "attachment_frame" ||
         name == "attachment_half_extents_x" || name == "attachment_half_extents_y" ||
         name == "attachment_half_extents_z" || name == "attachment_buffer" ||
         name == "self_filter_tf_timeout_sec";
}

}  // namespace stretch_core
