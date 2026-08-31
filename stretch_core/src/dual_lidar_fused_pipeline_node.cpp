// Publishes BOTH the merged point cloud and the LaserScan from a single pass.
//
// Replaces the pair dual_lidar_pointcloud_merger + dual_lidar_laserscan, which each ran
// their own ApproximateTime sync, their own cached TF lookup and their own full per-point
// transform.

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <message_filters/subscriber.hpp>
#include <message_filters/sync_policies/approximate_time.hpp>
#include <message_filters/synchronizer.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "stretch_core/fused_lidar_pipeline.hpp"
#include "stretch_core/point_cloud_layout.hpp"
#include "stretch_core/robot_self_filter.hpp"
#include "stretch_core/robot_self_filter_params.hpp"

namespace
{

std::string joinStrings(const std::vector<std::string> & values)
{
  std::ostringstream oss;
  for (size_t i = 0; i < values.size(); ++i) {
    if (i > 0) {
      oss << ", ";
    }
    oss << values[i];
  }
  return oss.str();
}

}  // namespace

class DualLidarFusedPipelineNode : public rclcpp::Node
{
public:
  explicit DualLidarFusedPipelineNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("dual_lidar_fused_pipeline", options),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    declareParameters();
    loadParameters();
    setupPublishers();
    setupDiagnostics();
  }

  // Blocks until the two lidar extrinsics and the z reference offset resolve.
  bool lookupStaticTransforms()
  {
    try {
      const auto tf_a = tf_buffer_.lookupTransform(
        target_frame_, lidar1_frame_, tf2::TimePointZero);
      const auto tf_b = tf_buffer_.lookupTransform(
        target_frame_, lidar2_frame_, tf2::TimePointZero);
      tf_lidar1_ = stretch_core::LinearTransform3f::fromAffine(
        tf2::transformToEigen(tf_a.transform).cast<float>());
      tf_lidar2_ = stretch_core::LinearTransform3f::fromAffine(
        tf2::transformToEigen(tf_b.transform).cast<float>());

      // Every z threshold in the YAML is written relative to param_z_frame
      // (base_footprint), but the pipeline works in target_frame (base_link). Derive the
      // shift from TF rather than hardcoding 0.028, so a URDF change cannot silently
      // desynchronise the two.
      if (param_z_frame_ == target_frame_) {
        z_offset_ = 0.0;
      } else {
        const auto tf_z = tf_buffer_.lookupTransform(
          target_frame_, param_z_frame_, tf2::TimePointZero);
        z_offset_ = tf_z.transform.translation.z;
      }

      applyFrameOffsets();
      RCLCPP_INFO(
        get_logger(),
        "Transforms cached. z thresholds shifted %+.4f m from '%s' into '%s'.",
        z_offset_, param_z_frame_.c_str(), target_frame_.c_str());
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Waiting for TFs: %s", ex.what());
      return false;
    }
  }

  void activateSubscription()
  {
    rclcpp::SensorDataQoS qos;
    sub1_.subscribe(this, lidar1_topic_, qos.get_rmw_qos_profile());
    sub2_.subscribe(this, lidar2_topic_, qos.get_rmw_qos_profile());
    sub1_.registerCallback(&DualLidarFusedPipelineNode::recordLidar1Stamp, this);
    sub2_.registerCallback(&DualLidarFusedPipelineNode::recordLidar2Stamp, this);
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(10), sub1_, sub2_);
    sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(sync_slop_sec_));
    sync_->setInterMessageLowerBound(
      0, rclcpp::Duration::from_seconds(kInterMessageLowerBoundSec));
    sync_->setInterMessageLowerBound(
      1, rclcpp::Duration::from_seconds(kInterMessageLowerBoundSec));
    sync_->registerCallback(
      std::bind(
        &DualLidarFusedPipelineNode::syncedCloudsCallback, this,
        std::placeholders::_1, std::placeholders::_2));
    subscriptions_active_ = true;
    RCLCPP_INFO(
      get_logger(), "Fused pipeline active: %s + %s -> %s (cloud) + %s (scan) in %s",
      lidar1_topic_.c_str(), lidar2_topic_.c_str(), cloud_topic_.c_str(),
      scan_topic_.c_str(), target_frame_.c_str());
  }

private:
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2>;

  static constexpr double kStaleTimeoutSec = 1.0;
  static constexpr double kInterMessageLowerBoundSec = 0.09;

  void declareParameters()
  {
    declare_parameter<std::string>("lidar1_topic", "/lidar_points_right");
    declare_parameter<std::string>("lidar2_topic", "/lidar_points_left");
    declare_parameter<std::string>("lidar1_frame", "lidar_right_link");
    declare_parameter<std::string>("lidar2_frame", "lidar_left_link");
    declare_parameter<std::string>("cloud_topic", "/lidar_points");
    declare_parameter<std::string>("scan_topic", "/scan_filtered");

    declare_parameter<std::string>("target_frame", "base_link");
    declare_parameter<std::string>("scan_frame", "laser");
    declare_parameter<std::string>("param_z_frame", "base_footprint");

    declare_parameter<std::string>("ring_field", "ring");
    declare_parameter<std::string>("timestamp_field", "timestamp");
    declare_parameter<double>("sync_slop_sec", 0.05);

    declare_parameter<bool>("publish_cloud", true);
    declare_parameter<bool>("publish_scan", true);

    declare_parameter("voxel_leaf_size", 0.05);
    declare_parameter("near_field_radius", -1.0);  // <0 -> follow self_filter_gate_radius_m
    declare_parameter("scan_range_max", 30.0);
    declare_parameter("scan_angle_increment_deg", 0.1);
    declare_parameter("z_min", 0.135);
    declare_parameter("z_max", 1.5);

    declare_parameter("enable_self_robot_filter", true);
    declare_parameter("enable_floor_ransac_filter", true);
    declare_parameter("floor_ransac_sample_stride", 16);
    declare_parameter("enable_sor_filter", false);

    declare_parameter("plane_fitting_threshold", 0.1);
    declare_parameter("angle", 10.0);
    declare_parameter("floor_detect_z_min", -0.05);
    declare_parameter("floor_detect_z_max", 0.05);

    declare_parameter("dist_rob", 2.5);
    declare_parameter("leaf_size", 0.05);
    declare_parameter("sor_mean_k", 50);
    declare_parameter("sor_stddev", 0.3);

    declare_parameter("speckle_filter_enabled", true);
    declare_parameter("speckle_min_points", 2);
    declare_parameter("speckle_neighbor_window", 3);
    declare_parameter("speckle_min_neighbors", 2);
    declare_parameter("speckle_range_tolerance", 0.15);

    stretch_core::declareRobotSelfFilterParameters(*this);
    declare_parameter("pub_self_filter_markers", false);
    declare_parameter<std::string>("self_filter_markers_topic", "/self_filter_markers");
    declare_parameter("log_stats_period_sec", 0.0);
  }

  void loadParameters()
  {
    lidar1_topic_ = get_parameter("lidar1_topic").as_string();
    lidar2_topic_ = get_parameter("lidar2_topic").as_string();
    lidar1_frame_ = get_parameter("lidar1_frame").as_string();
    lidar2_frame_ = get_parameter("lidar2_frame").as_string();
    cloud_topic_ = get_parameter("cloud_topic").as_string();
    scan_topic_ = get_parameter("scan_topic").as_string();
    target_frame_ = get_parameter("target_frame").as_string();
    scan_frame_ = get_parameter("scan_frame").as_string();
    param_z_frame_ = get_parameter("param_z_frame").as_string();
    ring_field_ = get_parameter("ring_field").as_string();
    timestamp_field_ = get_parameter("timestamp_field").as_string();
    sync_slop_sec_ = get_parameter("sync_slop_sec").as_double();
    publish_cloud_ = get_parameter("publish_cloud").as_bool();
    publish_scan_ = get_parameter("publish_scan").as_bool();
    pub_self_filter_markers_ = get_parameter("pub_self_filter_markers").as_bool();
    self_filter_markers_topic_ = get_parameter("self_filter_markers_topic").as_string();
    log_stats_period_sec_ = get_parameter("log_stats_period_sec").as_double();

    self_filter_config_ = stretch_core::loadRobotSelfFilterConfig(*this);
    self_filter_.setConfig(self_filter_config_);

    pipeline_config_.voxel_leaf_size =
      static_cast<float>(get_parameter("voxel_leaf_size").as_double());

    const double requested_radius = get_parameter("near_field_radius").as_double();
    pipeline_config_.near_field_radius = (requested_radius > 0.0) ?
      static_cast<float>(requested_radius) :
      self_filter_config_.self_filter_gate_radius_m;

    pipeline_config_.enable_self_filter = get_parameter("enable_self_robot_filter").as_bool();
    pipeline_config_.enable_floor_ransac = get_parameter("enable_floor_ransac_filter").as_bool();
    pipeline_config_.floor_sample_stride =
      static_cast<int>(get_parameter("floor_ransac_sample_stride").as_int());
    pipeline_config_.enable_sor = get_parameter("enable_sor_filter").as_bool();

    pipeline_config_.floor.plane_fitting_threshold =
      get_parameter("plane_fitting_threshold").as_double();
    pipeline_config_.floor.angle_deg = get_parameter("angle").as_double();

    pipeline_config_.sor.dist_rob = get_parameter("dist_rob").as_double();
    pipeline_config_.sor.leaf_size = get_parameter("leaf_size").as_double();
    pipeline_config_.sor.sor_mean_k = static_cast<int>(get_parameter("sor_mean_k").as_int());
    pipeline_config_.sor.sor_stddev = get_parameter("sor_stddev").as_double();

    pipeline_config_.speckle.enabled = get_parameter("speckle_filter_enabled").as_bool();
    pipeline_config_.speckle.min_points =
      static_cast<int>(get_parameter("speckle_min_points").as_int());
    pipeline_config_.speckle.neighbor_window =
      static_cast<int>(get_parameter("speckle_neighbor_window").as_int());
    pipeline_config_.speckle.min_neighbors =
      static_cast<int>(get_parameter("speckle_min_neighbors").as_int());
    pipeline_config_.speckle.range_tolerance =
      static_cast<float>(get_parameter("speckle_range_tolerance").as_double());

    configureScanProjection(get_parameter("scan_angle_increment_deg").as_double());
  }

  void configureScanProjection(double angle_increment_deg)
  {
    if (!std::isfinite(angle_increment_deg) || angle_increment_deg <= 0.0 ||
      angle_increment_deg > 5.0)
    {
      RCLCPP_WARN(
        get_logger(), "scan_angle_increment_deg=%.4f out of (0, 5]; keeping %.4f",
        angle_increment_deg, scan_angle_increment_deg_);
      angle_increment_deg = scan_angle_increment_deg_;
    }
    scan_angle_increment_deg_ = angle_increment_deg;
    scan_cfg_.angle_min = -static_cast<float>(M_PI);
    scan_cfg_.angle_max = static_cast<float>(M_PI);
    scan_cfg_.angle_increment =
      static_cast<float>(angle_increment_deg * static_cast<double>(M_PI) / 180.0);
    scan_cfg_.range_max = static_cast<float>(get_parameter("scan_range_max").as_double());
    const float span = scan_cfg_.angle_max - scan_cfg_.angle_min;
    scan_cfg_.num_ranges =
      std::max(1, static_cast<int>(std::round(span / scan_cfg_.angle_increment)));
  }

  // Shifts every base_footprint-relative threshold into target_frame and derives the
  // height fast lane's bounds.
  void applyFrameOffsets()
  {
    const float dz = static_cast<float>(z_offset_);

    const float z_min = static_cast<float>(get_parameter("z_min").as_double()) + dz;
    const float z_max = static_cast<float>(get_parameter("z_max").as_double()) + dz;
    const float floor_lo =
      static_cast<float>(get_parameter("floor_detect_z_min").as_double()) + dz;
    const float floor_hi =
      static_cast<float>(get_parameter("floor_detect_z_max").as_double()) + dz;

    pipeline_config_.floor.floor_detect_z_min = floor_lo;
    pipeline_config_.floor.floor_detect_z_max = floor_hi;
    pipeline_config_.scan_z_max = z_max;
    pipeline_config_.scan_z_min = pipeline_config_.enable_floor_ransac ? floor_lo : z_min;

    const float gate_lo = self_filter_config_.self_filter_gate_z_min_m + dz;
    const float gate_hi = self_filter_config_.self_filter_gate_z_max_m + dz;

    pipeline_config_.filter_z_top = std::max(gate_hi, z_max);
    pipeline_config_.filter_z_bot = std::min({gate_lo, pipeline_config_.scan_z_min, floor_lo});

    if (pipeline_config_.filter_z_top < pipeline_config_.scan_z_max ||
      pipeline_config_.filter_z_bot > pipeline_config_.scan_z_min)
    {
      RCLCPP_ERROR(
        get_logger(),
        "Height fast lane [%.3f, %.3f] does not contain the scan band [%.3f, %.3f]; "
        "scan returns would be silently dropped.",
        pipeline_config_.filter_z_bot, pipeline_config_.filter_z_top,
        pipeline_config_.scan_z_min, pipeline_config_.scan_z_max);
    }

    pipeline_.setConfig(pipeline_config_);

    RCLCPP_INFO(
      get_logger(),
      "Pipeline in %s: fast lane outside z [%.3f, %.3f] | scan band z [%.3f, %.3f] "
      "r < %.1f | self-filter r < %.2f | voxel %.3f m | floor=%s sor=%s",
      target_frame_.c_str(),
      pipeline_config_.filter_z_bot, pipeline_config_.filter_z_top,
      pipeline_config_.scan_z_min, pipeline_config_.scan_z_max, scan_cfg_.range_max,
      pipeline_config_.near_field_radius, pipeline_config_.voxel_leaf_size,
      pipeline_config_.enable_floor_ransac ? "ransac" : "z_min",
      pipeline_config_.enable_sor ? "on" : "off");
  }

  void setupPublishers()
  {
    rclcpp::SensorDataQoS qos;
    if (publish_cloud_) {
      pub_cloud_ = create_publisher<sensor_msgs::msg::PointCloud2>(cloud_topic_, qos);
    }
    if (publish_scan_) {
      pub_scan_ = create_publisher<sensor_msgs::msg::LaserScan>(scan_topic_, qos);
    }
    if (pub_self_filter_markers_) {
      pub_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        self_filter_markers_topic_, rclcpp::QoS(1).reliable());
    }
  }

  void setupDiagnostics()
  {
    pub_diag_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 1);
    diag_timer_ = create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&DualLidarFusedPipelineNode::publishHealth, this));
  }

  void recordLidar1Stamp(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
  {
    last_msg1_stamp_ = rclcpp::Time(msg->header.stamp);
    msg1_received_ = true;
  }

  void recordLidar2Stamp(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
  {
    last_msg2_stamp_ = rclcpp::Time(msg->header.stamp);
    msg2_received_ = true;
  }

  bool hasRequiredFields(const sensor_msgs::msg::PointCloud2 & cloud) const
  {
    if (!stretch_core::hasField(cloud, ring_field_) ||
      !stretch_core::hasField(cloud, timestamp_field_))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Cloud from %s is missing '%s' or '%s', which GLIM needs to deskew. Fields: [%s]",
        cloud.header.frame_id.c_str(), ring_field_.c_str(), timestamp_field_.c_str(),
        stretch_core::fieldNames(cloud).c_str());
      return false;
    }
    return true;
  }

  void syncedCloudsCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg1,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg2)
  {
    if (!hasRequiredFields(*msg1) || !hasRequiredFields(*msg2)) {
      return;
    }
    if (msg1->point_step != msg2->point_step ||
      !stretch_core::fieldsMatch(msg1->fields, msg2->fields))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Left/right clouds have mismatched fields or point_step; skipping this pair");
      return;
    }

    if (!layout_cached_) {
      layout_ = stretch_core::PointFieldLayout::fromCloud(*msg1);
      if (!layout_.valid) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000, "Could not cache x/y/z field offsets");
        return;
      }
      layout_cached_ = true;
      RCLCPP_INFO(
        get_logger(), "Point layout: step=%u x=%d y=%d z=%d contiguous=%s fields=[%s]",
        layout_.point_step, layout_.x_offset, layout_.y_offset, layout_.z_offset,
        layout_.xyzContiguous() ? "true" : "false",
        stretch_core::fieldNames(*msg1).c_str());
    }

    // Arm and box poses move with the joints, so they are refreshed every frame.
    if (pipeline_config_.enable_self_filter) {
      self_filter_.updateArmSegment(
        tf_buffer_, target_frame_, tf2::TimePointZero, get_logger(), *get_clock());
      self_filter_.updateSelfFilterBoxes(
        tf_buffer_, target_frame_, tf2::TimePointZero, get_logger(), *get_clock());
    }

    std_msgs::msg::Header header;
    header.frame_id = target_frame_;
    header.stamp = stretch_core::olderStamp(msg1->header.stamp, msg2->header.stamp);

    pipeline_.process(
      *msg1, *msg2, tf_lidar1_, tf_lidar2_, layout_, self_filter_, scan_cfg_,
      header, publish_cloud_, get_logger(), output_);

    if (publish_cloud_ && pub_cloud_) {
      pub_cloud_->publish(output_.cloud);
    }

    if (publish_scan_ && pub_scan_) {
      sensor_msgs::msg::LaserScan scan;
      scan.header.stamp = header.stamp;
      scan.header.frame_id = scan_frame_;
      scan.angle_min = scan_cfg_.angle_min;
      scan.angle_max = scan_cfg_.angle_max;
      scan.angle_increment = scan_cfg_.angle_increment;
      scan.range_min = self_filter_config_.base_radius;
      scan.range_max = scan_cfg_.range_max;
      scan.ranges = output_.scan.ranges;
      pub_scan_->publish(scan);
    }

    if (pub_markers_) {
      visualization_msgs::msg::MarkerArray markers;
      self_filter_.appendSelfFilterMarkers(markers, target_frame_, now());
      pub_markers_->publish(markers);
    }

    last_output_stamp_ = rclcpp::Time(header.stamp);
    output_received_ = true;
    logStats();
  }

  void logStats()
  {
    if (log_stats_period_sec_ <= 0.0) {
      return;
    }
    const auto current = now();
    if (last_stats_log_.nanoseconds() != 0 &&
      (current - last_stats_log_).seconds() < log_stats_period_sec_)
    {
      return;
    }
    last_stats_log_ = current;
    const auto & s = output_.stats;
    RCLCPP_INFO(
      get_logger(),
      "in=%zu -> cloud=%zu (%.1f%%) scan=%zu | fast-lane=%zu self-filtered=%zu",
      s.input_points, s.cloud_points,
      s.input_points ? (100.0 * static_cast<double>(s.cloud_points) /
      static_cast<double>(s.input_points)) : 0.0,
      s.scan_points, s.fast_lane_points, s.self_filtered_points);
  }

  void publishHealth()
  {
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "lidar_health";
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "OK";

    const auto current = now();
    std::vector<std::string> faults;

    auto addValue = [&status](const std::string & key, double value) {
        diagnostic_msgs::msg::KeyValue kv;
        kv.key = key;
        char buf[32];
        std::snprintf(buf, sizeof(buf), "%.3f", value);
        kv.value = buf;
        status.values.push_back(kv);
      };

    if (!subscriptions_active_) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "Waiting for lidar transforms";
    } else {
      bool any_stale = false;
      auto checkInput =
        [&](const std::string & label, bool received, const rclcpp::Time & stamp) {
          if (!received) {
            faults.push_back(label + ": no data yet");
            any_stale = true;
            return;
          }
          const double age = (current - stamp).seconds();
          addValue(label + "/age_s", age);
          if (age > kStaleTimeoutSec) {
            char buf[64];
            std::snprintf(buf, sizeof(buf), "%s: stale (%.1f s)", label.c_str(), age);
            faults.push_back(buf);
            any_stale = true;
          }
        };
      checkInput("lidar1", msg1_received_, last_msg1_stamp_);
      checkInput("lidar2", msg2_received_, last_msg2_stamp_);

      if (!output_received_) {
        faults.push_back("fused output: not published yet");
        any_stale = true;
      } else {
        const double age = (current - last_output_stamp_).seconds();
        addValue("fused/age_s", age);
        addValue("fused/cloud_points", static_cast<double>(output_.stats.cloud_points));
        addValue("fused/input_points", static_cast<double>(output_.stats.input_points));
        if (age > kStaleTimeoutSec) {
          char buf[64];
          std::snprintf(buf, sizeof(buf), "fused output: stale (%.1f s)", age);
          faults.push_back(buf);
          any_stale = true;
        }
      }

      if (!faults.empty()) {
        status.level = any_stale ?
          diagnostic_msgs::msg::DiagnosticStatus::STALE :
          diagnostic_msgs::msg::DiagnosticStatus::WARN;
        status.message = joinStrings(faults);
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000, "lidar_health: %s", status.message.c_str());
      }
    }

    diagnostic_msgs::msg::DiagnosticArray msg;
    msg.header.stamp = current;
    msg.status.push_back(status);
    pub_diag_->publish(msg);
  }

  stretch_core::FusedLidarPipeline pipeline_;
  stretch_core::FusedPipelineConfig pipeline_config_;
  stretch_core::FusedPipelineOutput output_;
  stretch_core::FusedScanConfig scan_cfg_;
  stretch_core::RobotSelfFilter self_filter_;
  stretch_core::RobotSelfFilterConfig self_filter_config_;
  stretch_core::PointFieldLayout layout_;
  stretch_core::LinearTransform3f tf_lidar1_;
  stretch_core::LinearTransform3f tf_lidar2_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub1_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub2_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_scan_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_diag_;
  rclcpp::TimerBase::SharedPtr diag_timer_;

  std::string lidar1_topic_;
  std::string lidar2_topic_;
  std::string lidar1_frame_;
  std::string lidar2_frame_;
  std::string cloud_topic_;
  std::string scan_topic_;
  std::string target_frame_;
  std::string scan_frame_;
  std::string param_z_frame_;
  std::string ring_field_;
  std::string timestamp_field_;
  std::string self_filter_markers_topic_;

  double sync_slop_sec_{0.05};
  double scan_angle_increment_deg_{0.1};
  double z_offset_{0.0};
  double log_stats_period_sec_{0.0};

  bool publish_cloud_{true};
  bool publish_scan_{true};
  bool pub_self_filter_markers_{false};
  bool layout_cached_{false};
  bool subscriptions_active_{false};
  bool msg1_received_{false};
  bool msg2_received_{false};
  bool output_received_{false};

  rclcpp::Time last_msg1_stamp_;
  rclcpp::Time last_msg2_stamp_;
  rclcpp::Time last_output_stamp_;
  rclcpp::Time last_stats_log_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DualLidarFusedPipelineNode>();

  rclcpp::Rate rate(10);
  while (rclcpp::ok() && !node->lookupStaticTransforms()) {
    rclcpp::spin_some(node);
    rate.sleep();
  }

  if (rclcpp::ok()) {
    node->activateSubscription();
    rclcpp::spin(node);
  }
  rclcpp::shutdown();
  return 0;
}
