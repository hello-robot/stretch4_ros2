#pragma once

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <Eigen/Dense>
#include <string>
#include <vector>

namespace stretch_core
{

struct RobotSelfFilterConfig
{
  float base_radius{0.25f};
  std::string arm_line_start_frame{"arm_l0_link"};
  std::string arm_line_start_height_frame{"lift_link"};
  float arm_line_height_offset_z{0.0f};
  std::string arm_line_end_frame{"wrist_link"};
  float arm_filter_radius{0.07f};
  float arm_filter_radius_buffer{0.02f};
  std::vector<std::string> self_filter_box_frames;
  std::vector<std::string> self_filter_box_names;
  std::vector<std::string> self_filter_box_groups;
  std::vector<float> self_filter_box_origin_x;
  std::vector<float> self_filter_box_origin_y;
  std::vector<float> self_filter_box_origin_z;
  std::vector<float> self_filter_box_rpy_roll;
  std::vector<float> self_filter_box_rpy_pitch;
  std::vector<float> self_filter_box_rpy_yaw;
  std::vector<float> self_filter_box_half_extents_x;
  std::vector<float> self_filter_box_half_extents_y;
  std::vector<float> self_filter_box_half_extents_z;
  float self_filter_arm_buffer{0.04f};
  float self_filter_wrist_buffer{0.025f};
  float self_filter_gripper_cam_buffer{0.025f};
  float self_filter_tool_buffer{0.025f};
  std::vector<float> self_filter_box_buffers;
  std::vector<float> self_filter_box_footprint_buffers;
  bool publish_raw_urdf_self_filter_markers{false};
  bool publish_buffered_self_filter_markers{true};
  std::string resolved_tool_preset{"unknown"};
  double tf_timeout_sec{0.05};
  bool self_filter_spatial_gate_enabled{true};
  float self_filter_gate_radius_m{1.5f};
  float self_filter_gate_z_min_m{-0.05f};
  float self_filter_gate_z_max_m{1.6f};
};

class RobotSelfFilter
{
public:
  void setConfig(const RobotSelfFilterConfig & config);

  bool updateArmSegment(
    tf2_ros::Buffer & buffer,
    const std::string & target_frame,
    const tf2::TimePoint & time,
    rclcpp::Logger logger,
    const rclcpp::Clock & clock);

  bool updateSelfFilterBoxes(
    tf2_ros::Buffer & buffer,
    const std::string & target_frame,
    const tf2::TimePoint & time,
    rclcpp::Logger logger,
    const rclcpp::Clock & clock);

  bool armValid() const {return arm_valid_;}
  bool selfFilterBoxesValid() const {return self_filter_boxes_valid_;}

  bool isInsideBaseCylinder(const Eigen::Vector3f & point) const;
  bool isInsideArmCapsule(const Eigen::Vector3f & point) const;
  bool isInsideSelfFilterBoxes(const Eigen::Vector3f & point) const;
  bool isSelfFiltered(const Eigen::Vector3f & point) const;
  bool isWithinSelfFilterGate(const Eigen::Vector3f & point) const;

  void appendSelfFilterMarkers(
    visualization_msgs::msg::MarkerArray & markers,
    const std::string & target_frame,
    const rclcpp::Time & stamp) const;

  std::vector<Eigen::Vector2f> computeFootprintPolygon2d(
    const std::vector<Eigen::Vector2f> & base_polygon) const;

private:
  static std::vector<Eigen::Vector2f> convexHull(std::vector<Eigen::Vector2f> points);
  void appendArmCapsuleSamples2d(std::vector<Eigen::Vector2f> & points) const;
  void appendSelfFilterBoxSamples2d(std::vector<Eigen::Vector2f> & points) const;

  static bool lookupTranslation(
    tf2_ros::Buffer & buffer,
    const std::string & target_frame,
    const std::string & source_frame,
    const tf2::TimePoint & time,
    double timeout_sec,
    Eigen::Vector3f & translation,
    rclcpp::Logger logger,
    const rclcpp::Clock & clock);

  static bool lookupTransform(
    tf2_ros::Buffer & buffer,
    const std::string & target_frame,
    const std::string & source_frame,
    const tf2::TimePoint & time,
    double timeout_sec,
    Eigen::Affine3f & transform,
    rclcpp::Logger logger,
    const rclcpp::Clock & clock);

  RobotSelfFilterConfig config_;
  bool arm_valid_{false};
  bool self_filter_boxes_valid_{false};
  Eigen::Vector3f arm_start_{Eigen::Vector3f::Zero()};
  Eigen::Vector3f arm_end_{Eigen::Vector3f::Zero()};
  Eigen::Vector3f arm_broadphase_center_{Eigen::Vector3f::Zero()};
  float arm_broadphase_radius_sq_{0.0f};
  float arm_radius_sq_{0.0f};
  struct SelfFilterBoxState
  {
    std::string frame_name;
    std::string display_name;
    std::string group_name;
    Eigen::Affine3f pose{Eigen::Affine3f::Identity()};
    Eigen::Affine3f inverse_pose{Eigen::Affine3f::Identity()};
    Eigen::Vector3f half_extents{Eigen::Vector3f::Zero()};
    float broadphase_radius_sq{0.0f};
    float filter_buffer{0.02f};
    float footprint_buffer{0.0f};
  };
  std::vector<SelfFilterBoxState> self_filter_boxes_;
  float base_radius_sq_{0.0f};
  float gate_radius_sq_{0.0f};
};

}  // namespace stretch_core
