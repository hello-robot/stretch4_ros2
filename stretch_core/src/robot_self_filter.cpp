#include "stretch_core/robot_self_filter.hpp"

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

}  // namespace

void RobotSelfFilter::setConfig(const RobotSelfFilterConfig & config)
{
  config_ = config;
  base_radius_sq_ = config_.base_radius * config_.base_radius;
  const float arm_radius = config_.arm_filter_radius + config_.arm_filter_radius_buffer;
  arm_radius_sq_ = arm_radius * arm_radius;
  arm_shoulder_half_extents_ = Eigen::Vector3f(
    config_.arm_shoulder_half_extents_x,
    config_.arm_shoulder_half_extents_y,
    config_.arm_shoulder_half_extents_z);
  arm_shoulder_rear_extent_ = config_.arm_shoulder_rear_extent;
  arm_shoulder_base_overshoot_ = config_.arm_shoulder_base_overshoot;
  arm_shoulder_buffer_ = config_.arm_shoulder_buffer;
  const float wrist_radius = config_.wrist_chain_link_radius + config_.wrist_chain_buffer;
  wrist_chain_radius_sq_ = wrist_radius * wrist_radius;
  attachment_half_extents_ = Eigen::Vector3f(
    config_.attachment_half_extents_x,
    config_.attachment_half_extents_y,
    config_.attachment_half_extents_z);
  attachment_buffer_ = config_.attachment_buffer;
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

  // arm_start_ = Eigen::Vector3f(arm_l0_pos.x(), arm_l0_pos.y(), lift_pos.z());
  arm_start_ = Eigen::Vector3f(
    arm_l0_pos.x(), arm_l0_pos.y(),
    lift_pos.z() + config_.arm_line_height_offset_z);
  // arm_end_ = end;
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
  wrist_chain_points_.clear();
  if (!config_.filter_wrist && !force) {
    return false;
  }
  if (config_.wrist_chain_frames.empty()) {
    return false;
  }

  wrist_chain_points_.reserve(config_.wrist_chain_frames.size());
  for (const auto & frame : config_.wrist_chain_frames) {
    Eigen::Vector3f point;
    if (!lookupTranslation(
        buffer, target_frame, frame, time,
        config_.tf_timeout_sec, point, logger, clock))
    {
      wrist_chain_points_.clear();
      return false;
    }
    wrist_chain_points_.push_back(point);
  }

  wrist_chain_valid_ = !wrist_chain_points_.empty();
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

  attachment_pose_ = pose;
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

  const Eigen::Vector3f local = arm_shoulder_pose_.inverse() * point;
  const float limit_x_pos = arm_shoulder_half_extents_.x() + arm_shoulder_buffer_;
  const float limit_x_neg = arm_shoulder_half_extents_.x() + arm_shoulder_buffer_ +
    arm_shoulder_rear_extent_;
  const float limit_y = arm_shoulder_half_extents_.y() + arm_shoulder_buffer_;
  const float limit_z = arm_shoulder_half_extents_.z() + arm_shoulder_buffer_;

  const bool inside_obb = local.x() <= limit_x_pos &&
    local.x() >= -limit_x_neg &&
    std::abs(local.y()) <= limit_y &&
    std::abs(local.z()) <= limit_z;

  if (inside_obb) {
    return true;
  }

  if (arm_shoulder_base_overshoot_ > 0.0f) {
    const float xy_dist = std::sqrt(point.x() * point.x() + point.y() * point.y());
    const float overshoot_limit = config_.base_radius + arm_shoulder_base_overshoot_;
    if (xy_dist <= overshoot_limit) {
      const Eigen::Vector3f local_overshoot = arm_shoulder_pose_.inverse() * point;
      if (local_overshoot.x() <= limit_x_pos &&
        local_overshoot.x() >= -limit_x_neg &&
        std::abs(local_overshoot.y()) <= limit_y &&
        std::abs(local_overshoot.z()) <= limit_z)
      {
        return true;
      }
    }
  }

  return false;
}

bool RobotSelfFilter::isInsideWristChain(const Eigen::Vector3f & point) const
{
  if (!wrist_chain_valid_) {
    return false;
  }
  for (const auto & center : wrist_chain_points_) {
    if ((point - center).squaredNorm() <= wrist_chain_radius_sq_) {
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

  const Eigen::Vector3f local = attachment_pose_.inverse() * point;
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

  const float radius = config_.arm_filter_radius + config_.arm_filter_radius_buffer;
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

  const float radius = config_.wrist_chain_link_radius + config_.wrist_chain_buffer;
  constexpr int kCapAngles = 8;
  for (const auto & center : wrist_chain_points_) {
    const Eigen::Vector2f c(center.x(), center.y());
    for (int i = 0; i < kCapAngles; ++i) {
      const float theta = 2.0f * static_cast<float>(M_PI) * static_cast<float>(i) /
        static_cast<float>(kCapAngles);
      points.push_back(c + radius * Eigen::Vector2f(std::cos(theta), std::sin(theta)));
    }
  }
}

void RobotSelfFilter::appendAttachmentSamples2d(std::vector<Eigen::Vector2f> & points) const
{
  if (!config_.filter_attachment || !attachment_valid_) {
    return;
  }

  const float ex = attachment_half_extents_.x() + attachment_buffer_;
  const float ey = attachment_half_extents_.y() + attachment_buffer_;
  const float ez = attachment_half_extents_.z() + attachment_buffer_;
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
    shoulder_marker.scale.x = 2.0f * arm_shoulder_half_extents_.x() + arm_shoulder_rear_extent_ +
      2.0f * arm_shoulder_buffer_;
    shoulder_marker.scale.y = 2.0f * arm_shoulder_half_extents_.y() + 2.0f * arm_shoulder_buffer_;
    shoulder_marker.scale.z = 2.0f * arm_shoulder_half_extents_.z() + 2.0f * arm_shoulder_buffer_;
    shoulder_marker.color.r = 0.95f;
    shoulder_marker.color.g = 0.55f;
    shoulder_marker.color.b = 0.1f;
    shoulder_marker.color.a = 0.35f;
    markers.markers.push_back(shoulder_marker);
  }

  if (wrist_chain_valid_) {
    const float diameter = 2.0f * (config_.wrist_chain_link_radius + config_.wrist_chain_buffer);
    for (size_t i = 0; i < wrist_chain_points_.size(); ++i) {
      visualization_msgs::msg::Marker wrist_marker;
      wrist_marker.header.frame_id = target_frame;
      wrist_marker.header.stamp = stamp;
      wrist_marker.ns = "self_filter/wrist";
      wrist_marker.id = marker_id++;
      wrist_marker.type = visualization_msgs::msg::Marker::SPHERE;
      wrist_marker.action = visualization_msgs::msg::Marker::ADD;
      wrist_marker.pose.position = eigenVecToPoint(wrist_chain_points_[i]);
      wrist_marker.pose.orientation.w = 1.0;
      wrist_marker.scale.x = diameter;
      wrist_marker.scale.y = diameter;
      wrist_marker.scale.z = diameter;
      wrist_marker.color.r = 0.2f;
      wrist_marker.color.g = 0.4f;
      wrist_marker.color.b = 0.95f;
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
