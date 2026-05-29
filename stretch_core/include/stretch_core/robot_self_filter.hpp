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
  bool filter_base{true};
  float base_radius{0.25f};
  bool filter_arm{false};
  std::string arm_line_start_frame{"arm_l0_link"};
  std::string arm_line_start_height_frame{"lift_link"};
  float arm_line_height_offset_z{0.0f};
  std::string arm_line_end_frame{"wrist_link"};
  float arm_filter_radius{0.07f};
  float arm_filter_radius_buffer{0.02f};
  bool filter_arm_shoulder{false};
  std::string arm_shoulder_box_frame{"arm_l0_link"};
  float arm_shoulder_box_origin_x{0.0f};
  float arm_shoulder_box_origin_y{0.0f};
  float arm_shoulder_box_origin_z{0.0f};
  float arm_shoulder_half_extents_x{0.08f};
  float arm_shoulder_half_extents_y{0.10f};
  float arm_shoulder_half_extents_z{0.10f};
  float arm_shoulder_buffer{0.02f};
  bool filter_wrist{false};
  std::vector<std::string> wrist_chain_frames;
  std::vector<float> wrist_chain_box_origin_x;
  std::vector<float> wrist_chain_box_origin_y;
  std::vector<float> wrist_chain_box_origin_z;
  std::vector<float> wrist_chain_half_extents_x;
  std::vector<float> wrist_chain_half_extents_y;
  std::vector<float> wrist_chain_half_extents_z;
  float wrist_chain_buffer{0.02f};
  std::vector<float> wrist_chain_buffers;
  bool filter_attachment{false};
  std::string attachment_frame{"quick_connect_interface_link"};
  float attachment_half_extents_x{0.10f};
  float attachment_half_extents_y{0.08f};
  float attachment_half_extents_z{0.08f};
  float attachment_buffer{0.02f};
  double tf_timeout_sec{0.05};
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
    const rclcpp::Clock & clock,
    bool force = false);

  bool updateArmShoulderBox(
    tf2_ros::Buffer & buffer,
    const std::string & target_frame,
    const tf2::TimePoint & time,
    rclcpp::Logger logger,
    const rclcpp::Clock & clock,
    bool force = false);

  bool updateWristChain(
    tf2_ros::Buffer & buffer,
    const std::string & target_frame,
    const tf2::TimePoint & time,
    rclcpp::Logger logger,
    const rclcpp::Clock & clock,
    bool force = false);

  bool updateAttachmentBox(
    tf2_ros::Buffer & buffer,
    const std::string & target_frame,
    const tf2::TimePoint & time,
    rclcpp::Logger logger,
    const rclcpp::Clock & clock,
    bool force = false);

  bool armValid() const {return arm_valid_;}
  bool armShoulderValid() const {return arm_shoulder_valid_;}
  bool wristChainValid() const {return wrist_chain_valid_;}
  bool attachmentValid() const {return attachment_valid_;}

  bool isInsideBaseCylinder(const Eigen::Vector3f & point) const;
  bool isInsideArmCapsule(const Eigen::Vector3f & point) const;
  bool isInsideArmShoulderBox(const Eigen::Vector3f & point) const;
  bool isInsideWristChain(const Eigen::Vector3f & point) const;
  bool isInsideAttachmentBox(const Eigen::Vector3f & point) const;
  bool isSelfFiltered(const Eigen::Vector3f & point) const;

  void appendSelfFilterMarkers(
    visualization_msgs::msg::MarkerArray & markers,
    const std::string & target_frame,
    const rclcpp::Time & stamp,
    bool markers_only_viz = false) const;

  std::vector<Eigen::Vector2f> computeFootprintPolygon2d(
    const std::vector<Eigen::Vector2f> & base_polygon) const;

private:
  static std::vector<Eigen::Vector2f> convexHull(std::vector<Eigen::Vector2f> points);
  void appendArmCapsuleSamples2d(std::vector<Eigen::Vector2f> & points) const;
  void appendWristChainSamples2d(std::vector<Eigen::Vector2f> & points) const;
  void appendAttachmentSamples2d(std::vector<Eigen::Vector2f> & points) const;

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
  bool arm_shoulder_valid_{false};
  bool wrist_chain_valid_{false};
  bool attachment_valid_{false};
  Eigen::Vector3f arm_start_{Eigen::Vector3f::Zero()};
  Eigen::Vector3f arm_end_{Eigen::Vector3f::Zero()};
  float arm_radius_sq_{0.0f};
  Eigen::Affine3f arm_shoulder_pose_{Eigen::Affine3f::Identity()};
  Eigen::Affine3f arm_shoulder_inverse_pose_{Eigen::Affine3f::Identity()};
  Eigen::Vector3f arm_shoulder_half_extents_{Eigen::Vector3f::Zero()};
  float arm_shoulder_buffer_{0.0f};
  struct WristChainBoxState
  {
    std::string frame_name;
    Eigen::Affine3f pose{Eigen::Affine3f::Identity()};
    Eigen::Affine3f inverse_pose{Eigen::Affine3f::Identity()};
    Eigen::Vector3f half_extents{Eigen::Vector3f::Zero()};
    float filter_buffer{0.02f};
  };
  std::vector<WristChainBoxState> wrist_chain_boxes_;
  float wrist_chain_buffer_{0.02f};
  Eigen::Affine3f attachment_pose_{Eigen::Affine3f::Identity()};
  Eigen::Affine3f attachment_inverse_pose_{Eigen::Affine3f::Identity()};
  Eigen::Vector3f attachment_half_extents_{Eigen::Vector3f::Zero()};
  float attachment_buffer_{0.0f};
  float base_radius_sq_{0.0f};
};

}  // namespace stretch_core
