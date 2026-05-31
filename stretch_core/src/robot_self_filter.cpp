#include "stretch_core/robot_self_filter.hpp"
#include "stretch_core/robot_self_filter_params.hpp"

#include <tf2_eigen/tf2_eigen.hpp>

#include <algorithm>
#include <array>
#include <cmath>

namespace stretch_core
{

namespace
{
constexpr float kSegmentEpsilonSq = 1e-8f;

float squaredDistancePointToSegment(
  const Eigen::Vector3f & point,
  const Eigen::Vector3f & seg_start,
  const Eigen::Vector3f & seg_end)
{
  const Eigen::Vector3f ab = seg_end - seg_start;
  const float ab_len_sq = ab.squaredNorm();
  if (ab_len_sq < kSegmentEpsilonSq) {
    return (point - seg_start).squaredNorm();
  }

  const Eigen::Vector3f ap = point - seg_start;
  const float t = std::clamp(ap.dot(ab) / ab_len_sq, 0.0f, 1.0f);
  const Eigen::Vector3f closest = seg_start + t * ab;
  return (point - closest).squaredNorm();
}

geometry_msgs::msg::Quaternion eigenQuatToMsg(const Eigen::Quaternionf & q)
{
  geometry_msgs::msg::Quaternion msg;
  msg.x = q.x();
  msg.y = q.y();
  msg.z = q.z();
  msg.w = q.w();
  return msg;
}

geometry_msgs::msg::Point eigenVecToPoint(const Eigen::Vector3f & v)
{
  geometry_msgs::msg::Point p;
  p.x = v.x();
  p.y = v.y();
  p.z = v.z();
  return p;
}

void wristChainDebugColor(size_t index, float & r, float & g, float & b)
{
  static constexpr std::array<std::array<float, 3>, 5> kPalette = {{
    {0.95f, 0.20f, 0.20f},  // 0 wrist_link — red
    {0.20f, 0.90f, 0.20f},  // 1 wrist_yaw_link — green
    {0.20f, 0.40f, 0.95f},  // 2 wrist_pitch_link — blue
    {0.95f, 0.75f, 0.10f},  // 3 wrist_roll_link — amber
    {0.10f, 0.85f, 0.85f},  // 4 gripper_camera_link — cyan
  }};
  const size_t slot = index % kPalette.size();
  r = kPalette[slot][0];
  g = kPalette[slot][1];
  b = kPalette[slot][2];
}

}  // namespace

void RobotSelfFilter::setConfig(const RobotSelfFilterConfig & config)
{
  config_ = config;
  normalizeWristChainArrays(config_);
  base_radius_sq_ = config_.base_radius * config_.base_radius;
  const float arm_radius = config_.arm_filter_radius + config_.arm_filter_radius_buffer;
  arm_radius_sq_ = arm_radius * arm_radius;
  arm_shoulder_half_extents_ = Eigen::Vector3f(
    config_.arm_shoulder_half_extents_x,
    config_.arm_shoulder_half_extents_y,
    config_.arm_shoulder_half_extents_z);
  arm_shoulder_buffer_ = config_.arm_shoulder_buffer;
  wrist_chain_buffer_ = config_.wrist_chain_buffer;
  attachment_half_extents_ = Eigen::Vector3f(
    config_.attachment_half_extents_x,
    config_.attachment_half_extents_y,
    config_.attachment_half_extents_z);
  attachment_buffer_ = config_.attachment_buffer;
  gate_radius_sq_ = config_.self_filter_gate_radius_m * config_.self_filter_gate_radius_m;
}

bool RobotSelfFilter::lookupTranslation(
  tf2_ros::Buffer & buffer,
  const std::string & target_frame,
  const std::string & source_frame,
  const tf2::TimePoint & time,
  double timeout_sec,
  Eigen::Vector3f & translation,
  rclcpp::Logger logger,
  const rclcpp::Clock & clock)
{
  try {
    const auto tf_msg = buffer.lookupTransform(
      target_frame,
      source_frame,
      time,
      tf2::durationFromSec(timeout_sec));
    const Eigen::Affine3d affine = tf2::transformToEigen(tf_msg.transform);
    translation = affine.translation().cast<float>();
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      logger,
      clock,
      2000,
      "Self-filter TF lookup failed (%s -> %s): %s",
      target_frame.c_str(),
      source_frame.c_str(),
      ex.what());
    return false;
  }
}

bool RobotSelfFilter::lookupTransform(
  tf2_ros::Buffer & buffer,
  const std::string & target_frame,
  const std::string & source_frame,
  const tf2::TimePoint & time,
  double timeout_sec,
  Eigen::Affine3f & transform,
  rclcpp::Logger logger,
  const rclcpp::Clock & clock)
{
  try {
    const auto tf_msg = buffer.lookupTransform(
      target_frame,
      source_frame,
      time,
      tf2::durationFromSec(timeout_sec));
    transform = tf2::transformToEigen(tf_msg.transform).cast<float>();
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      logger,
      clock,
      2000,
      "Self-filter TF lookup failed (%s -> %s): %s",
      target_frame.c_str(),
      source_frame.c_str(),
      ex.what());
    return false;
  }
}

bool RobotSelfFilter::updateArmSegment(
  tf2_ros::Buffer & buffer,
  const std::string & target_frame,
  const tf2::TimePoint & time,
  rclcpp::Logger logger,
  const rclcpp::Clock & clock,
  bool force)
{
  arm_valid_ = false;
  if (!config_.filter_arm && !force) {
    return false;
  }

  Eigen::Vector3f arm_l0_pos;
  Eigen::Vector3f lift_pos;
  Eigen::Vector3f end;
  if (!lookupTranslation(
      buffer, target_frame, config_.arm_line_start_frame, time,
      config_.tf_timeout_sec, arm_l0_pos, logger, clock))
  {
    return false;
  }
  if (!lookupTranslation(
      buffer, target_frame, config_.arm_line_start_height_frame, time,
      config_.tf_timeout_sec, lift_pos, logger, clock))
  {
    return false;
  }
  if (!lookupTranslation(
      buffer, target_frame, config_.arm_line_end_frame, time,
      config_.tf_timeout_sec, end, logger, clock))
  {
    return false;
  }

  arm_start_ = Eigen::Vector3f(
    arm_l0_pos.x(), arm_l0_pos.y(),
    lift_pos.z() + config_.arm_line_height_offset_z);
  arm_end_ = Eigen::Vector3f(
    end.x(), end.y(), end.z() + config_.arm_line_height_offset_z);
  arm_valid_ = true;
  return true;
}

bool RobotSelfFilter::updateArmShoulderBox(
  tf2_ros::Buffer & buffer,
  const std::string & target_frame,
  const tf2::TimePoint & time,
  rclcpp::Logger logger,
  const rclcpp::Clock & clock,
  bool force)
{
  arm_shoulder_valid_ = false;
  if (!config_.filter_arm_shoulder && !force) {
    return false;
  }

  Eigen::Affine3f box_pose;
  if (!lookupTransform(
      buffer, target_frame, config_.arm_shoulder_box_frame, time,
      config_.tf_timeout_sec, box_pose, logger, clock))
  {
    return false;
  }

  const Eigen::Vector3f local_origin(
    config_.arm_shoulder_box_origin_x,
    config_.arm_shoulder_box_origin_y,
    config_.arm_shoulder_box_origin_z);
  arm_shoulder_pose_ = box_pose;
  arm_shoulder_pose_.translation() += box_pose.rotation() * local_origin;
  arm_shoulder_inverse_pose_ = arm_shoulder_pose_.inverse();
  arm_shoulder_valid_ = true;
  return true;
}

bool RobotSelfFilter::updateWristChain(
  tf2_ros::Buffer & buffer,
  const std::string & target_frame,
  const tf2::TimePoint & time,
  rclcpp::Logger logger,
  const rclcpp::Clock & clock,
  bool force)
{
  wrist_chain_valid_ = false;
  wrist_chain_boxes_.clear();
  if (!config_.filter_wrist && !force) {
    return false;
  }
  const size_t link_count = config_.wrist_chain_frames.size();
  if (link_count == 0) {
    return false;
  }

  wrist_chain_boxes_.reserve(link_count);
  for (size_t i = 0; i < link_count; ++i) {
    const auto & frame = config_.wrist_chain_frames[i];
    Eigen::Affine3f link_pose;
    if (!lookupTransform(
        buffer, target_frame, frame, time,
        config_.tf_timeout_sec, link_pose, logger, clock))
    {
      wrist_chain_boxes_.clear();
      return false;
    }

    const float origin_x = (i < config_.wrist_chain_box_origin_x.size()) ?
      config_.wrist_chain_box_origin_x[i] : 0.0f;
    const float origin_y = (i < config_.wrist_chain_box_origin_y.size()) ?
      config_.wrist_chain_box_origin_y[i] : 0.0f;
    const float origin_z = (i == 0 && i < config_.wrist_chain_box_origin_z.size()) ?
      config_.wrist_chain_box_origin_z[i] : 0.0f;

    const Eigen::Vector3f local_origin(origin_x, origin_y, origin_z);
    link_pose.translation() += link_pose.rotation() * local_origin;

    WristChainBoxState box;
    box.frame_name = frame;
    box.pose = link_pose;
    box.inverse_pose = link_pose.inverse();
    box.half_extents = Eigen::Vector3f(
      (i < config_.wrist_chain_half_extents_x.size()) ?
      config_.wrist_chain_half_extents_x[i] : 0.06f,
      (i < config_.wrist_chain_half_extents_y.size()) ?
      config_.wrist_chain_half_extents_y[i] : 0.06f,
      (i < config_.wrist_chain_half_extents_z.size()) ?
      config_.wrist_chain_half_extents_z[i] : 0.06f);
    if (config_.wrist_chain_buffers.empty()) {
      box.filter_buffer = wrist_chain_buffer_;
    } else if (i < config_.wrist_chain_buffers.size()) {
      box.filter_buffer = config_.wrist_chain_buffers[i];
    } else {
      box.filter_buffer = wrist_chain_buffer_;
    }
    wrist_chain_boxes_.push_back(box);
  }

  wrist_chain_valid_ = !wrist_chain_boxes_.empty();
  return wrist_chain_valid_;
}

bool RobotSelfFilter::updateAttachmentBox(
  tf2_ros::Buffer & buffer,
  const std::string & target_frame,
  const tf2::TimePoint & time,
  rclcpp::Logger logger,
  const rclcpp::Clock & clock,
  bool force)
{
  attachment_valid_ = false;
  if (!config_.filter_attachment && !force) {
    return false;
  }

  Eigen::Affine3f pose;
  if (!lookupTransform(
      buffer, target_frame, config_.attachment_frame, time,
      config_.tf_timeout_sec, pose, logger, clock))
  {
    return false;
  }

  const Eigen::Vector3f local_origin(
    config_.attachment_box_origin_x,
    config_.attachment_box_origin_y,
    config_.attachment_box_origin_z);
  attachment_pose_ = pose;
  attachment_pose_.translation() += pose.rotation() * local_origin;
  attachment_inverse_pose_ = attachment_pose_.inverse();
  attachment_valid_ = true;
  return true;
}

bool RobotSelfFilter::isInsideBaseCylinder(const Eigen::Vector3f & point) const
{
  if (!config_.filter_base) {
    return false;
  }
  const float xy_sq = point.x() * point.x() + point.y() * point.y();
  return xy_sq <= base_radius_sq_;
}

bool RobotSelfFilter::isWithinSelfFilterGate(const Eigen::Vector3f & point) const
{
  if (!config_.self_filter_spatial_gate_enabled) {
    return true;
  }

  const float xy_sq = point.x() * point.x() + point.y() * point.y();
  if (xy_sq > gate_radius_sq_) {
    return false;
  }

  const float z = point.z();
  if (z < config_.self_filter_gate_z_min_m || z > config_.self_filter_gate_z_max_m) {
    return false;
  }

  return true;
}

bool RobotSelfFilter::isSelfFiltered(const Eigen::Vector3f & point) const
{
  if (isInsideBaseCylinder(point)) {
    return true;
  }
  if (config_.filter_arm && isInsideArmCapsule(point)) {
    return true;
  }
  if (config_.filter_arm_shoulder && isInsideArmShoulderBox(point)) {
    return true;
  }
  if (config_.filter_wrist && isInsideWristChain(point)) {
    return true;
  }
  if (config_.filter_attachment && isInsideAttachmentBox(point)) {
    return true;
  }
  return false;
}

bool RobotSelfFilter::isInsideArmCapsule(const Eigen::Vector3f & point) const
{
  if (!arm_valid_) {
    return false;
  }
  return squaredDistancePointToSegment(point, arm_start_, arm_end_) <= arm_radius_sq_;
}

bool RobotSelfFilter::isInsideArmShoulderBox(const Eigen::Vector3f & point) const
{
  if (!arm_shoulder_valid_) {
    return false;
  }

  const Eigen::Vector3f local = arm_shoulder_inverse_pose_ * point;
  const float limit_x = arm_shoulder_half_extents_.x() + arm_shoulder_buffer_;
  const float limit_y = arm_shoulder_half_extents_.y() + arm_shoulder_buffer_;
  const float limit_z = arm_shoulder_half_extents_.z() + arm_shoulder_buffer_;

  return std::abs(local.x()) <= limit_x &&
         std::abs(local.y()) <= limit_y &&
         std::abs(local.z()) <= limit_z;
}

bool RobotSelfFilter::isInsideWristChain(const Eigen::Vector3f & point) const
{
  if (!wrist_chain_valid_) {
    return false;
  }
  for (const auto & box : wrist_chain_boxes_) {
    const Eigen::Vector3f local = box.inverse_pose * point;
    const float limit_x = box.half_extents.x() + box.filter_buffer;
    const float limit_y = box.half_extents.y() + box.filter_buffer;
    const float limit_z = box.half_extents.z() + box.filter_buffer;
    if (std::abs(local.x()) <= limit_x &&
      std::abs(local.y()) <= limit_y &&
      std::abs(local.z()) <= limit_z)
    {
      return true;
    }
  }
  return false;
}

bool RobotSelfFilter::isInsideAttachmentBox(const Eigen::Vector3f & point) const
{
  if (!attachment_valid_) {
    return false;
  }

  const Eigen::Vector3f local = attachment_inverse_pose_ * point;
  const float limit_x = attachment_half_extents_.x() + attachment_buffer_;
  const float limit_y = attachment_half_extents_.y() + attachment_buffer_;
  const float limit_z = attachment_half_extents_.z() + attachment_buffer_;
  return std::abs(local.x()) <= limit_x &&
         std::abs(local.y()) <= limit_y &&
         std::abs(local.z()) <= limit_z;
}

std::vector<Eigen::Vector2f> RobotSelfFilter::convexHull(std::vector<Eigen::Vector2f> points)
{
  if (points.size() <= 1) {
    return points;
  }

  auto cross = [](const Eigen::Vector2f & o, const Eigen::Vector2f & a, const Eigen::Vector2f & b) {
      return (a.x() - o.x()) * (b.y() - o.y()) - (a.y() - o.y()) * (b.x() - o.x());
    };

  std::sort(points.begin(), points.end(), [](const Eigen::Vector2f & a, const Eigen::Vector2f & b) {
      if (a.x() < b.x()) {
        return true;
      }
      if (a.x() > b.x()) {
        return false;
      }
      return a.y() < b.y();
    });

  std::vector<Eigen::Vector2f> hull;
  hull.reserve(points.size() * 2);

  for (const auto & p : points) {
    while (hull.size() >= 2 &&
      cross(hull[hull.size() - 2], hull.back(), p) <= 0.0f)
    {
      hull.pop_back();
    }
    hull.push_back(p);
  }

  const size_t lower_size = hull.size();
  for (int i = static_cast<int>(points.size()) - 2; i >= 0; --i) {
    const auto & p = points[static_cast<size_t>(i)];
    while (hull.size() > lower_size &&
      cross(hull[hull.size() - 2], hull.back(), p) <= 0.0f)
    {
      hull.pop_back();
    }
    hull.push_back(p);
  }

  if (!hull.empty()) {
    hull.pop_back();
  }
  return hull;
}

void RobotSelfFilter::appendArmCapsuleSamples2d(std::vector<Eigen::Vector2f> & points) const
{
  if (!config_.filter_arm || !arm_valid_) {
    return;
  }

  const float radius = config_.arm_filter_radius;
  const Eigen::Vector2f a(arm_start_.x(), arm_start_.y());
  const Eigen::Vector2f b(arm_end_.x(), arm_end_.y());
  const Eigen::Vector2f ab = b - a;
  const float length = ab.norm();

  Eigen::Vector2f tangent(1.0f, 0.0f);
  if (length > 1e-4f) {
    tangent = ab / length;
  }
  const Eigen::Vector2f normal(-tangent.y(), tangent.x());

  constexpr int kSegmentSteps = 10;
  constexpr int kCapAngles = 8;
  for (int i = 0; i <= kSegmentSteps; ++i) {
    const float t = static_cast<float>(i) / static_cast<float>(kSegmentSteps);
    const Eigen::Vector2f center = a + t * ab;
    points.push_back(center + normal * radius);
    points.push_back(center - normal * radius);
  }

  for (int i = 0; i <= kCapAngles; ++i) {
    const float theta = static_cast<float>(M_PI) * static_cast<float>(i) / static_cast<float>(kCapAngles);
    const float c = std::cos(theta);
    const float s = std::sin(theta);
    points.push_back(a + tangent * (c * radius) + normal * (s * radius));
    points.push_back(b + tangent * (c * radius) + normal * (s * radius));
  }
}

void RobotSelfFilter::appendWristChainSamples2d(std::vector<Eigen::Vector2f> & points) const
{
  if (!config_.filter_wrist || !wrist_chain_valid_) {
    return;
  }

  for (const auto & box : wrist_chain_boxes_) {
    const float ex = box.half_extents.x();
    const float ey = box.half_extents.y();
    const float ez = box.half_extents.z();
    const std::array<Eigen::Vector3f, 8> corners = {
      Eigen::Vector3f(-ex, -ey, -ez),
      Eigen::Vector3f(ex, -ey, -ez),
      Eigen::Vector3f(-ex, ey, -ez),
      Eigen::Vector3f(ex, ey, -ez),
      Eigen::Vector3f(-ex, -ey, ez),
      Eigen::Vector3f(ex, -ey, ez),
      Eigen::Vector3f(-ex, ey, ez),
      Eigen::Vector3f(ex, ey, ez),
    };

    for (const auto & corner : corners) {
      const Eigen::Vector3f world = box.pose * corner;
      points.emplace_back(world.x(), world.y());
    }
  }
}

void RobotSelfFilter::appendAttachmentSamples2d(std::vector<Eigen::Vector2f> & points) const
{
  if (!config_.filter_attachment || !attachment_valid_) {
    return;
  }

  const float ex = attachment_half_extents_.x();
  const float ey = attachment_half_extents_.y();
  const float ez = attachment_half_extents_.z();
  const std::array<Eigen::Vector3f, 8> corners = {
    Eigen::Vector3f(-ex, -ey, -ez),
    Eigen::Vector3f(ex, -ey, -ez),
    Eigen::Vector3f(-ex, ey, -ez),
    Eigen::Vector3f(ex, ey, -ez),
    Eigen::Vector3f(-ex, -ey, ez),
    Eigen::Vector3f(ex, -ey, ez),
    Eigen::Vector3f(-ex, ey, ez),
    Eigen::Vector3f(ex, ey, ez),
  };

  for (const auto & corner : corners) {
    const Eigen::Vector3f world = attachment_pose_ * corner;
    points.emplace_back(world.x(), world.y());
  }
}

std::vector<Eigen::Vector2f> RobotSelfFilter::computeFootprintPolygon2d(
  const std::vector<Eigen::Vector2f> & base_polygon) const
{
  std::vector<Eigen::Vector2f> samples;
  samples.reserve(base_polygon.size() + 64);
  samples.insert(samples.end(), base_polygon.begin(), base_polygon.end());
  appendArmCapsuleSamples2d(samples);
  appendWristChainSamples2d(samples);
  appendAttachmentSamples2d(samples);
  return convexHull(std::move(samples));
}

void RobotSelfFilter::appendSelfFilterMarkers(
  visualization_msgs::msg::MarkerArray & markers,
  const std::string & target_frame,
  const rclcpp::Time & stamp,
  bool markers_only_viz) const
{
  visualization_msgs::msg::Marker clear;
  clear.header.frame_id = target_frame;
  clear.header.stamp = stamp;
  clear.ns = "self_filter";
  clear.id = 0;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(clear);

  int marker_id = 1;

  if (config_.self_filter_spatial_gate_enabled) {
    const float gate_radius = config_.self_filter_gate_radius_m;
    const float gate_diameter = 2.0f * gate_radius;
    const float z_min = config_.self_filter_gate_z_min_m;
    const float z_max = config_.self_filter_gate_z_max_m;
    const float gate_height = std::max(z_max - z_min, 0.1f);
    const float gate_center_z = 0.5f * (z_min + z_max);

    visualization_msgs::msg::Marker gate_marker;
    gate_marker.header.frame_id = target_frame;
    gate_marker.header.stamp = stamp;
    gate_marker.ns = "self_filter/gate";
    gate_marker.id = marker_id++;
    gate_marker.type = visualization_msgs::msg::Marker::CYLINDER;
    gate_marker.action = visualization_msgs::msg::Marker::ADD;
    gate_marker.pose.position.x = 0.0;
    gate_marker.pose.position.y = 0.0;
    gate_marker.pose.position.z = gate_center_z;
    gate_marker.pose.orientation.w = 1.0;
    gate_marker.scale.x = gate_diameter;
    gate_marker.scale.y = gate_diameter;
    gate_marker.scale.z = gate_height;
    gate_marker.color.r = 0.6f;
    gate_marker.color.g = 0.6f;
    gate_marker.color.b = 0.6f;
    gate_marker.color.a = 0.15f;
    markers.markers.push_back(gate_marker);

    visualization_msgs::msg::Marker ring_marker;
    ring_marker.header.frame_id = target_frame;
    ring_marker.header.stamp = stamp;
    ring_marker.ns = "self_filter/gate_ring";
    ring_marker.id = marker_id++;
    ring_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    ring_marker.action = visualization_msgs::msg::Marker::ADD;
    ring_marker.pose.orientation.w = 1.0;
    ring_marker.scale.x = 0.02f;
    ring_marker.color.r = 1.0f;
    ring_marker.color.g = 1.0f;
    ring_marker.color.b = 1.0f;
    ring_marker.color.a = 0.9f;
    static constexpr int kRingSegments = 64;
    ring_marker.points.reserve(kRingSegments);
    for (int i = 0; i < kRingSegments; ++i) {
      const float angle =
        static_cast<float>(2.0 * M_PI * static_cast<double>(i) / static_cast<double>(kRingSegments));
      geometry_msgs::msg::Point pt;
      pt.x = gate_radius * std::cos(angle);
      pt.y = gate_radius * std::sin(angle);
      pt.z = 0.0;
      ring_marker.points.push_back(pt);
    }
    ring_marker.points.push_back(ring_marker.points.front());
    markers.markers.push_back(ring_marker);
  }

  if (config_.filter_base || markers_only_viz) {
    const float diameter = 2.0f * config_.base_radius;
    visualization_msgs::msg::Marker base_marker;
    base_marker.header.frame_id = target_frame;
    base_marker.header.stamp = stamp;
    base_marker.ns = "self_filter/base";
    base_marker.id = marker_id++;
    base_marker.type = visualization_msgs::msg::Marker::CYLINDER;
    base_marker.action = visualization_msgs::msg::Marker::ADD;
    base_marker.pose.position.x = 0.0;
    base_marker.pose.position.y = 0.0;
    base_marker.pose.position.z = 0.0;
    base_marker.pose.orientation.w = 1.0;
    base_marker.scale.x = diameter;
    base_marker.scale.y = diameter;
    base_marker.scale.z = 0.5f;
    base_marker.color.r = 0.95f;
    base_marker.color.g = 0.2f;
    base_marker.color.b = 0.2f;
    base_marker.color.a = 0.35f;
    markers.markers.push_back(base_marker);
  }

  if (arm_valid_) {
    const Eigen::Vector3f segment = arm_end_ - arm_start_;
    const float length = segment.norm();
    const float diameter = 2.0f * (config_.arm_filter_radius + config_.arm_filter_radius_buffer);

    visualization_msgs::msg::Marker arm_marker;
    arm_marker.header.frame_id = target_frame;
    arm_marker.header.stamp = stamp;
    arm_marker.ns = "self_filter/arm";
    arm_marker.id = marker_id++;
    arm_marker.type = visualization_msgs::msg::Marker::CYLINDER;
    arm_marker.action = visualization_msgs::msg::Marker::ADD;
    arm_marker.pose.position = eigenVecToPoint(0.5f * (arm_start_ + arm_end_));
    arm_marker.pose.orientation = eigenQuatToMsg(
      Eigen::Quaternionf::Identity());

    if (length > 1e-4f) {
      const Eigen::Vector3f direction = segment / length;
      arm_marker.pose.orientation = eigenQuatToMsg(
        Eigen::Quaternionf::FromTwoVectors(Eigen::Vector3f::UnitZ(), direction));
    }

    arm_marker.scale.x = diameter;
    arm_marker.scale.y = diameter;
    arm_marker.scale.z = std::max(length, 1e-3f);
    arm_marker.color.r = 0.0f;
    arm_marker.color.g = 0.9f;
    arm_marker.color.b = 0.2f;
    arm_marker.color.a = 0.35f;
    markers.markers.push_back(arm_marker);
  }

  if (arm_shoulder_valid_) {
    visualization_msgs::msg::Marker shoulder_marker;
    shoulder_marker.header.frame_id = target_frame;
    shoulder_marker.header.stamp = stamp;
    shoulder_marker.ns = "self_filter/arm_shoulder";
    shoulder_marker.id = marker_id++;
    shoulder_marker.type = visualization_msgs::msg::Marker::CUBE;
    shoulder_marker.action = visualization_msgs::msg::Marker::ADD;

    const Eigen::Quaternionf q(arm_shoulder_pose_.rotation());
    shoulder_marker.pose.position = eigenVecToPoint(arm_shoulder_pose_.translation());
    shoulder_marker.pose.orientation = eigenQuatToMsg(q);
    shoulder_marker.scale.x = 2.0f * arm_shoulder_half_extents_.x() + 2.0f * arm_shoulder_buffer_;
    shoulder_marker.scale.y = 2.0f * arm_shoulder_half_extents_.y() + 2.0f * arm_shoulder_buffer_;
    shoulder_marker.scale.z = 2.0f * arm_shoulder_half_extents_.z() + 2.0f * arm_shoulder_buffer_;
    shoulder_marker.color.r = 0.95f;
    shoulder_marker.color.g = 0.55f;
    shoulder_marker.color.b = 0.1f;
    shoulder_marker.color.a = 0.35f;
    markers.markers.push_back(shoulder_marker);
  }

  if (wrist_chain_valid_) {
    for (size_t i = 0; i < wrist_chain_boxes_.size(); ++i) {
      const auto & box = wrist_chain_boxes_[i];
      visualization_msgs::msg::Marker wrist_marker;
      wrist_marker.header.frame_id = target_frame;
      wrist_marker.header.stamp = stamp;
      wrist_marker.ns = "self_filter/wrist/" + box.frame_name;
      wrist_marker.id = static_cast<int>(i);
      wrist_marker.type = visualization_msgs::msg::Marker::CUBE;
      wrist_marker.action = visualization_msgs::msg::Marker::ADD;

      const Eigen::Quaternionf q(box.pose.rotation());
      wrist_marker.pose.position = eigenVecToPoint(box.pose.translation());
      wrist_marker.pose.orientation = eigenQuatToMsg(q);
      wrist_marker.scale.x = 2.0f * (box.half_extents.x() + box.filter_buffer);
      wrist_marker.scale.y = 2.0f * (box.half_extents.y() + box.filter_buffer);
      wrist_marker.scale.z = 2.0f * (box.half_extents.z() + box.filter_buffer);
      wristChainDebugColor(i, wrist_marker.color.r, wrist_marker.color.g, wrist_marker.color.b);
      wrist_marker.color.a = 0.35f;
      markers.markers.push_back(wrist_marker);
    }
  }

  if (attachment_valid_) {
    visualization_msgs::msg::Marker attachment_marker;
    attachment_marker.header.frame_id = target_frame;
    attachment_marker.header.stamp = stamp;
    attachment_marker.ns = "self_filter/attachment";
    attachment_marker.id = marker_id++;
    attachment_marker.type = visualization_msgs::msg::Marker::CUBE;
    attachment_marker.action = visualization_msgs::msg::Marker::ADD;

    const Eigen::Quaternionf q(attachment_pose_.rotation());
    attachment_marker.pose.position = eigenVecToPoint(attachment_pose_.translation());
    attachment_marker.pose.orientation = eigenQuatToMsg(q);
    attachment_marker.scale.x = 2.0f * (attachment_half_extents_.x() + attachment_buffer_);
    attachment_marker.scale.y = 2.0f * (attachment_half_extents_.y() + attachment_buffer_);
    attachment_marker.scale.z = 2.0f * (attachment_half_extents_.z() + attachment_buffer_);
    attachment_marker.color.r = 0.85f;
    attachment_marker.color.g = 0.2f;
    attachment_marker.color.b = 0.85f;
    attachment_marker.color.a = 0.35f;
    markers.markers.push_back(attachment_marker);
  }
}

}  // namespace stretch_core
