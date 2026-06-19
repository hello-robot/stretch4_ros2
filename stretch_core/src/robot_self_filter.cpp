#include "stretch_core/robot_self_filter.hpp"
#include "stretch_core/robot_self_filter_params.hpp"

#include <tf2_eigen/tf2_eigen.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

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

float boxBroadphaseRadiusSq(const Eigen::Vector3f & half_extents, float buffer)
{
  // Conservative sphere around an OBB. Points outside this sphere cannot be
  // inside the exact box, so we avoid the more expensive inverse transform.
  const Eigen::Vector3f padded(
    half_extents.x() + buffer,
    half_extents.y() + buffer,
    half_extents.z() + buffer);
  return padded.squaredNorm();
}

Eigen::Matrix3f rpyToRotation(float roll, float pitch, float yaw)
{
  const Eigen::AngleAxisf roll_angle(roll, Eigen::Vector3f::UnitX());
  const Eigen::AngleAxisf pitch_angle(pitch, Eigen::Vector3f::UnitY());
  const Eigen::AngleAxisf yaw_angle(yaw, Eigen::Vector3f::UnitZ());
  return (yaw_angle * pitch_angle * roll_angle).toRotationMatrix();
}

float groupFilterBuffer(const RobotSelfFilterConfig & config, const std::string & group_name)
{
  if (group_name == "arm") {
    return config.self_filter_arm_buffer;
  }
  if (group_name == "wrist") {
    return config.self_filter_wrist_buffer;
  }
  if (group_name == "gripper_camera" || group_name == "gripper_cam") {
    return config.self_filter_gripper_cam_buffer;
  }
  if (group_name == "tool") {
    return config.self_filter_tool_buffer;
  }
  throw std::runtime_error("Unknown self-filter box group: " + group_name);
}

geometry_msgs::msg::Point eigenVecToPoint(const Eigen::Vector3f & v)
{
  geometry_msgs::msg::Point p;
  p.x = v.x();
  p.y = v.y();
  p.z = v.z();
  return p;
}

void selfFilterBoxDebugColor(size_t index, float & r, float & g, float & b)
{
  static constexpr std::array<std::array<float, 3>, 5> kPalette = {{
    {0.95f, 0.20f, 0.20f},  // 0 red
    {0.20f, 0.90f, 0.20f},  // 1 green
    {0.20f, 0.40f, 0.95f},  // 2 blue
    {0.95f, 0.75f, 0.10f},  // 3 amber
    {0.10f, 0.85f, 0.85f},  // 4 cyan
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
  validateSelfFilterBoxArrays(config_);
  for (const auto & group_name : config_.self_filter_box_groups) {
    (void)groupFilterBuffer(config_, group_name);
  }
  base_radius_sq_ = config_.base_radius * config_.base_radius;
  const float arm_radius = config_.arm_filter_radius + config_.arm_filter_radius_buffer;
  arm_radius_sq_ = arm_radius * arm_radius;
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
  const rclcpp::Clock & clock)
{
  arm_valid_ = false;

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

  // Conservative sphere around the capsule for fast reject.
  arm_broadphase_center_ = 0.5f * (arm_start_ + arm_end_);
  const float broadphase_radius = 0.5f * (arm_end_ - arm_start_).norm() + std::sqrt(arm_radius_sq_);
  arm_broadphase_radius_sq_ = broadphase_radius * broadphase_radius;
  arm_valid_ = true;
  return true;
}

bool RobotSelfFilter::updateSelfFilterBoxes(
  tf2_ros::Buffer & buffer,
  const std::string & target_frame,
  const tf2::TimePoint & time,
  rclcpp::Logger logger,
  const rclcpp::Clock & clock)
{
  self_filter_boxes_valid_ = false;
  self_filter_boxes_.clear();
  const size_t box_count = config_.self_filter_box_frames.size();
  if (box_count == 0) {
    return false;
  }

  self_filter_boxes_.reserve(box_count);
  for (size_t i = 0; i < box_count; ++i) {
    const auto & frame = config_.self_filter_box_frames[i];
    Eigen::Affine3f link_pose;
    if (!lookupTransform(
        buffer, target_frame, frame, time,
        config_.tf_timeout_sec, link_pose, logger, clock))
    {
      continue;
    }

    const float origin_x = config_.self_filter_box_origin_x[i];
    const float origin_y = config_.self_filter_box_origin_y[i];
    const float origin_z = config_.self_filter_box_origin_z[i];
    const float roll = config_.self_filter_box_rpy_roll[i];
    const float pitch = config_.self_filter_box_rpy_pitch[i];
    const float yaw = config_.self_filter_box_rpy_yaw[i];

    Eigen::Affine3f local_box_pose = Eigen::Affine3f::Identity();
    local_box_pose.linear() = rpyToRotation(roll, pitch, yaw);
    local_box_pose.translation() = Eigen::Vector3f(origin_x, origin_y, origin_z);

    SelfFilterBoxState box;
    box.frame_name = frame;
    box.display_name = config_.self_filter_box_names[i];
    box.group_name = config_.self_filter_box_groups[i];
    box.pose = link_pose * local_box_pose;
    box.inverse_pose = box.pose.inverse();
    box.half_extents = Eigen::Vector3f(
      config_.self_filter_box_half_extents_x[i],
      config_.self_filter_box_half_extents_y[i],
      config_.self_filter_box_half_extents_z[i]);
    if (config_.self_filter_box_buffers.empty()) {
      box.filter_buffer = groupFilterBuffer(config_, box.group_name);
    } else {
      box.filter_buffer = config_.self_filter_box_buffers[i];
    }
    // Keep Nav2 conservative by default: the footprint uses the same buffer that
    // removes self returns, unless an expert footprint override is explicitly set.
    box.footprint_buffer = box.filter_buffer;
    if (i < config_.self_filter_box_footprint_buffers.size()) {
      box.footprint_buffer = config_.self_filter_box_footprint_buffers[i];
    }
    box.broadphase_radius_sq = boxBroadphaseRadiusSq(box.half_extents, box.filter_buffer);
    self_filter_boxes_.push_back(box);
  }

  self_filter_boxes_valid_ = !self_filter_boxes_.empty();
  return self_filter_boxes_valid_;
}

bool RobotSelfFilter::isInsideBaseCylinder(const Eigen::Vector3f & point) const
{
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

  if (arm_valid_ &&
    (point - arm_broadphase_center_).squaredNorm() <= arm_broadphase_radius_sq_ &&
    squaredDistancePointToSegment(point, arm_start_, arm_end_) <= arm_radius_sq_)
  {
    return true;
  }

  if (isInsideSelfFilterBoxes(point)) {
    return true;
  }

  return false;
}

bool RobotSelfFilter::isInsideArmCapsule(const Eigen::Vector3f & point) const
{
  if (!arm_valid_) {
    return false;
  }
  if ((point - arm_broadphase_center_).squaredNorm() > arm_broadphase_radius_sq_) {
    return false;
  }
  return squaredDistancePointToSegment(point, arm_start_, arm_end_) <= arm_radius_sq_;
}

bool RobotSelfFilter::isInsideSelfFilterBoxes(const Eigen::Vector3f & point) const
{
  if (!self_filter_boxes_valid_) {
    return false;
  }
  for (const auto & box : self_filter_boxes_) {
    if ((point - box.pose.translation()).squaredNorm() > box.broadphase_radius_sq) {
      continue;
    }
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
  if (!arm_valid_) {
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

void RobotSelfFilter::appendSelfFilterBoxSamples2d(std::vector<Eigen::Vector2f> & points) const
{
  if (!self_filter_boxes_valid_) {
    return;
  }

  for (const auto & box : self_filter_boxes_) {
    const float ex = box.half_extents.x() + box.footprint_buffer;
    const float ey = box.half_extents.y() + box.footprint_buffer;
    const float ez = box.half_extents.z() + box.footprint_buffer;
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

std::vector<Eigen::Vector2f> RobotSelfFilter::computeFootprintPolygon2d(
  const std::vector<Eigen::Vector2f> & base_polygon) const
{
  std::vector<Eigen::Vector2f> samples;
  samples.reserve(base_polygon.size() + 64);
  samples.insert(samples.end(), base_polygon.begin(), base_polygon.end());
  appendArmCapsuleSamples2d(samples);
  appendSelfFilterBoxSamples2d(samples);
  return convexHull(std::move(samples));
}

void RobotSelfFilter::appendSelfFilterMarkers(
  visualization_msgs::msg::MarkerArray & markers,
  const std::string & target_frame,
  const rclcpp::Time & stamp) const
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

  {
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

  if (self_filter_boxes_valid_) {
    for (size_t i = 0; i < self_filter_boxes_.size(); ++i) {
      const auto & box = self_filter_boxes_[i];
      const Eigen::Quaternionf q(box.pose.rotation());
      float color_r = 0.0f;
      float color_g = 0.0f;
      float color_b = 0.0f;
      selfFilterBoxDebugColor(i, color_r, color_g, color_b);

      if (config_.publish_raw_urdf_self_filter_markers) {
        visualization_msgs::msg::Marker raw_marker;
        raw_marker.header.frame_id = target_frame;
        raw_marker.header.stamp = stamp;
        raw_marker.ns = "self_filter/urdf_raw/" + box.group_name + "/" + box.display_name;
        raw_marker.id = marker_id++;
        raw_marker.type = visualization_msgs::msg::Marker::CUBE;
        raw_marker.action = visualization_msgs::msg::Marker::ADD;
        raw_marker.pose.position = eigenVecToPoint(box.pose.translation());
        raw_marker.pose.orientation = eigenQuatToMsg(q);
        raw_marker.scale.x = 2.0f * box.half_extents.x();
        raw_marker.scale.y = 2.0f * box.half_extents.y();
        raw_marker.scale.z = 2.0f * box.half_extents.z();
        raw_marker.color.r = color_r;
        raw_marker.color.g = color_g;
        raw_marker.color.b = color_b;
        raw_marker.color.a = 0.18f;
        markers.markers.push_back(raw_marker);
      }

      if (config_.publish_buffered_self_filter_markers) {
        visualization_msgs::msg::Marker buffered_marker;
        buffered_marker.header.frame_id = target_frame;
        buffered_marker.header.stamp = stamp;
        buffered_marker.ns = "self_filter/urdf_buffered/" + box.group_name + "/" + box.display_name;
        buffered_marker.id = marker_id++;
        buffered_marker.type = visualization_msgs::msg::Marker::CUBE;
        buffered_marker.action = visualization_msgs::msg::Marker::ADD;
        buffered_marker.pose.position = eigenVecToPoint(box.pose.translation());
        buffered_marker.pose.orientation = eigenQuatToMsg(q);
        buffered_marker.scale.x = 2.0f * (box.half_extents.x() + box.filter_buffer);
        buffered_marker.scale.y = 2.0f * (box.half_extents.y() + box.filter_buffer);
        buffered_marker.scale.z = 2.0f * (box.half_extents.z() + box.filter_buffer);
        buffered_marker.color.r = color_r;
        buffered_marker.color.g = color_g;
        buffered_marker.color.b = color_b;
        buffered_marker.color.a = 0.35f;
        markers.markers.push_back(buffered_marker);
      }
    }
  }

}

}  // namespace stretch_core
