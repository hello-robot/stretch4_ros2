#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <message_filters/subscriber.hpp>
#include <message_filters/sync_policies/approximate_time.hpp>
#include <message_filters/synchronizer.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "stretch_core/dual_lidar_pipeline.hpp"
#include "stretch_core/pipeline_stages.hpp"
#include "stretch_core/robot_self_filter.hpp"
#include "stretch_core/robot_self_filter_params.hpp"

namespace
{

std::string joinStageNames(const std::vector<std::string> & names)
{
  std::ostringstream oss;
  for (size_t i = 0; i < names.size(); ++i) {
    if (i > 0) {
      oss << ", ";
    }
    oss << names[i];
  }
  return oss.str();
}

builtin_interfaces::msg::Time olderStamp(
  const builtin_interfaces::msg::Time & a,
  const builtin_interfaces::msg::Time & b)
{
  if (a.sec != b.sec) {
    return (a.sec < b.sec) ? a : b;
  }
  return (a.nanosec < b.nanosec) ? a : b;
}

bool hasAnyScanHits(const stretch_core::PipelineOutput & output)
{
  for (int count : output.hit_counts) {
    if (count > 0) {
      return true;
    }
  }
  return false;
}

}  // namespace

class DualLidarLaserScanNode : public rclcpp::Node
{
public:
  DualLidarLaserScanNode()
  : Node("dual_pointcloud_to_laserscan"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    declareParameters();
    loadParameters();
    configurePipeline();
    configureScanProjection(scan_angle_increment_deg_);
    setupPublishers();

    setupMarkerPublisher();
    setupDiagnostics();

    // on_set only validates; post_set is what applies a change to the node.
    param_callback_handle_ = add_on_set_parameters_callback(
      std::bind(&DualLidarLaserScanNode::validateParameters, this, std::placeholders::_1));
    post_param_callback_handle_ = add_post_set_parameters_callback(
      std::bind(&DualLidarLaserScanNode::applyParameters, this, std::placeholders::_1));
  }

  bool lookupStaticTransforms()
  {
    try {
      RCLCPP_INFO(
        get_logger(),
        "Looking up transforms: target='%s', lidar1='%s', lidar2='%s'",
        target_frame_.c_str(), lidar1_frame_.c_str(), lidar2_frame_.c_str());

      const auto tf1 = tf_buffer_.lookupTransform(
        target_frame_, lidar1_frame_, tf2::TimePointZero);
      const auto tf2 = tf_buffer_.lookupTransform(
        target_frame_, lidar2_frame_, tf2::TimePointZero);
      tf_lidar1_ = tf2::transformToEigen(tf1.transform).matrix().cast<float>();
      tf_lidar2_ = tf2::transformToEigen(tf2.transform).matrix().cast<float>();
      RCLCPP_INFO(get_logger(), "Transforms cached.");
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(get_logger(), "Waiting for TFs: %s", ex.what());
      return false;
    }
  }

  void activateSubscription()
  {
    // best-effort sensor QoS.
    sub1_.subscribe(this, lidar1_topic_, rmw_qos_profile_sensor_data);
    sub2_.subscribe(this, lidar2_topic_, rmw_qos_profile_sensor_data);
    sub1_.registerCallback(&DualLidarLaserScanNode::recordLidar1Stamp, this);
    sub2_.registerCallback(&DualLidarLaserScanNode::recordLidar2Stamp, this);
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(10), sub1_, sub2_);
    // Maximum time between two pairs of clouds.
    sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(kMaxPairIntervalSec));
    // Minimum spacing between clouds per topic. Helps ApproximateTime decide on the optimal pair faster.
    sync_->setInterMessageLowerBound(
      0, rclcpp::Duration::from_seconds(kInterMessageLowerBoundSec));
    sync_->setInterMessageLowerBound(
      1, rclcpp::Duration::from_seconds(kInterMessageLowerBoundSec));
    sync_->registerCallback(
      std::bind(
        &DualLidarLaserScanNode::syncedCloudsCallback, this,
        std::placeholders::_1, std::placeholders::_2));
    subscriptions_active_ = true;
  }

private:
  void declareParameters()
  {
    declare_parameter<std::string>("filter_type", "region");
    declare_parameter<std::string>("lidar1_topic", "/lidar_points_right");
    declare_parameter<std::string>("lidar2_topic", "/lidar_points_left");
    declare_parameter<std::string>("lidar1_frame", "lidar_right_link");
    declare_parameter<std::string>("lidar2_frame", "lidar_left_link");
    declare_parameter<std::string>("frame_id", "base_footprint");
    declare_parameter<std::string>("output_topic", "/scan_filtered");
    declare_parameter<bool>("pub_pointcloud", false);
    declare_parameter<std::string>("pointcloud_topic", "/lidar_pointcloud");

    declare_parameter<bool>("enable_self_robot_filter", true);
    declare_parameter<bool>("enable_region_filter", true);
    declare_parameter<bool>("enable_sor_filter", false);
    declare_parameter<bool>("enable_floor_ransac_filter", false);

    declare_parameter("z_min", 0.135);
    declare_parameter("z_max", 1.5);
    declare_parameter("range_max", 30.0);
    declare_parameter("scan_angle_increment_deg", 0.05);

    stretch_core::declareRobotSelfFilterParameters(*this);
    declare_parameter("pub_self_filter_markers", false);
    declare_parameter<std::string>("self_filter_markers_topic", "/self_filter_markers");

    declare_parameter("dist_rob", 2.5);
    declare_parameter("leaf_size", 0.05);
    declare_parameter("sor_mean_k", 50);
    declare_parameter("sor_stddev", 0.3);

    declare_parameter("plane_fitting_threshold", 0.1);
    declare_parameter("angle", 10.0);
    declare_parameter("floor_detect_z_min", -0.4);
    declare_parameter("floor_detect_z_max", 0.1);

    declare_parameter("speckle_filter_enabled", true);
    declare_parameter("speckle_min_points", 2);
    declare_parameter("speckle_neighbor_window", 3);
    declare_parameter("speckle_min_neighbors", 2);
    declare_parameter("speckle_range_tolerance", 0.15);
  }

  void loadParameters()
  {
    filter_type_ = get_parameter("filter_type").as_string();
    lidar1_topic_ = get_parameter("lidar1_topic").as_string();
    lidar2_topic_ = get_parameter("lidar2_topic").as_string();
    lidar1_frame_ = get_parameter("lidar1_frame").as_string();
    lidar2_frame_ = get_parameter("lidar2_frame").as_string();
    target_frame_ = get_parameter("frame_id").as_string();
    scan_topic_ = get_parameter("output_topic").as_string();
    pub_pointcloud_ = get_parameter("pub_pointcloud").as_bool();
    pointcloud_topic_ = get_parameter("pointcloud_topic").as_string();

    enable_self_robot_filter_ = get_parameter("enable_self_robot_filter").as_bool();
    enable_region_filter_ = get_parameter("enable_region_filter").as_bool();
    enable_sor_filter_ = get_parameter("enable_sor_filter").as_bool();
    enable_floor_ransac_filter_ = get_parameter("enable_floor_ransac_filter").as_bool();

    pub_self_filter_markers_ = get_parameter("pub_self_filter_markers").as_bool();
    self_filter_markers_topic_ = get_parameter("self_filter_markers_topic").as_string();

    pipeline_config_.region.z_min = static_cast<float>(get_parameter("z_min").as_double());
    pipeline_config_.region.z_max = static_cast<float>(get_parameter("z_max").as_double());
    pipeline_config_.region.range_max = static_cast<float>(get_parameter("range_max").as_double());
    scan_angle_increment_deg_ = get_parameter("scan_angle_increment_deg").as_double();

    pipeline_config_.sor.dist_rob = get_parameter("dist_rob").as_double();
    pipeline_config_.sor.leaf_size = get_parameter("leaf_size").as_double();
    pipeline_config_.sor.sor_mean_k = get_parameter("sor_mean_k").as_int();
    pipeline_config_.sor.sor_stddev = get_parameter("sor_stddev").as_double();

    pipeline_config_.floor.plane_fitting_threshold =
      get_parameter("plane_fitting_threshold").as_double();
    pipeline_config_.floor.angle_deg = get_parameter("angle").as_double();
    pipeline_config_.floor.floor_detect_z_min =
      static_cast<float>(get_parameter("floor_detect_z_min").as_double());
    pipeline_config_.floor.floor_detect_z_max =
      static_cast<float>(get_parameter("floor_detect_z_max").as_double());

    pipeline_config_.speckle_filter_enabled =
      get_parameter("speckle_filter_enabled").as_bool();
    pipeline_config_.speckle_min_points = get_parameter("speckle_min_points").as_int();
    pipeline_config_.speckle_neighbor_window =
      get_parameter("speckle_neighbor_window").as_int();
    pipeline_config_.speckle_min_neighbors =
      get_parameter("speckle_min_neighbors").as_int();
    pipeline_config_.speckle_range_tolerance =
      static_cast<float>(get_parameter("speckle_range_tolerance").as_double());

    loadSelfFilterParams();
  }

  void setupPublishers()
  {
    
    pub_ = create_publisher<sensor_msgs::msg::LaserScan>(scan_topic_, rclcpp::SensorDataQoS());
   
    if (pub_pointcloud_) {
      pub_cloud_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        pointcloud_topic_, rclcpp::SensorDataQoS());
    } else {
      pub_cloud_.reset();
    }
  }

  void setupMarkerPublisher()
  {
    if (pub_self_filter_markers_) {
      pub_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        self_filter_markers_topic_, rclcpp::QoS(1).reliable());
      RCLCPP_INFO(
        get_logger(), "Self-filter markers publishing on %s",
        self_filter_markers_topic_.c_str());
    } else {
      pub_markers_.reset();
      RCLCPP_INFO(get_logger(), "Self-filter markers disabled");
    }
  }

  static bool isValidScanAngleIncrement(double angle_increment_deg)
  {
    return std::isfinite(angle_increment_deg) && angle_increment_deg > 0.0 &&
           angle_increment_deg <= 5.0;
  }

  bool configureScanProjection(double angle_increment_deg)
  {
    if (!isValidScanAngleIncrement(angle_increment_deg)) {
      return false;
    }

    scan_angle_increment_deg_ = angle_increment_deg;
    scan_cfg_.angle_min = -static_cast<float>(M_PI);
    scan_cfg_.angle_max = static_cast<float>(M_PI);
    scan_cfg_.angle_increment = static_cast<float>(
      angle_increment_deg * static_cast<double>(M_PI) / 180.0);
    scan_cfg_.range_max = pipeline_config_.region.range_max;

    const float scan_span = scan_cfg_.angle_max - scan_cfg_.angle_min;
    scan_cfg_.num_ranges = std::max(
      1, static_cast<int>(std::round(scan_span / scan_cfg_.angle_increment)));
    return true;
  }

  void loadSelfFilterParams()
  {
    self_filter_config_ = stretch_core::loadRobotSelfFilterConfig(*this);
    self_filter_.setConfig(self_filter_config_);
  }

  void updateSelfFilterTransforms(const tf2::TimePoint & tf_time)
  {
    self_filter_.updateArmSegment(
      tf_buffer_, target_frame_, tf_time, get_logger(), *get_clock());
    self_filter_.updateSelfFilterBoxes(
      tf_buffer_, target_frame_, tf_time, get_logger(), *get_clock());
  }

  void configurePipeline()
  {
    if (filter_type_ == "custom") {
      stages_ = stretch_core::stagesFromEnables(
        enable_self_robot_filter_,
        enable_region_filter_,
        enable_sor_filter_,
        enable_floor_ransac_filter_);
    } else {
      stages_ = stretch_core::stagesFromFilterType(filter_type_);
    }
    pipeline_.setStages(stages_);
    pipeline_.setConfig(pipeline_config_);

    const auto names = stretch_core::stageNames(stages_);
    std::string z_min_note;
    if (stretch_core::hasStage(stages_, stretch_core::PipelineStage::FloorRansac)) {
      z_min_note = " (region z_min disabled; floor handled by FloorRansac)";
    }
    RCLCPP_INFO(
      get_logger(),
      "Pipeline: filter_type=%s stages=[%s]%s | pub_pointcloud=%s",
      filter_type_.c_str(),
      joinStageNames(names).c_str(),
      z_min_note.c_str(),
      pub_pointcloud_ ? "true" : "false");
  }

  void recordLidar1Stamp(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
  {
    last_msg1_stamp_ = rclcpp::Time(msg->header.stamp);
    last_msg1_latency_ = (now() - last_msg1_stamp_).seconds();
    msg1_received_ = true;
  }

  void recordLidar2Stamp(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
  {
    last_msg2_stamp_ = rclcpp::Time(msg->header.stamp);
    last_msg2_latency_ = (now() - last_msg2_stamp_).seconds();
    msg2_received_ = true;
  }

  void setupDiagnostics()
  {
    pub_diag_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 1);
    diag_timer_ = create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&DualLidarLaserScanNode::publishLidarHealth, this));
  }

  void publishLidarHealth()
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
        [&](const std::string & label, bool received, const rclcpp::Time & stamp,
        double latency) {
          if (!received) {
            faults.push_back(label + ": no data yet");
            any_stale = true;
            return;
          }
          const double age = (current - stamp).seconds();
          addValue(label + "/age_s", age);
          addValue(label + "/latency_s", latency);
          if (age > kStaleTimeoutSec) {
            char buf[64];
            std::snprintf(buf, sizeof(buf), "%s: stale (%.1f s)", label.c_str(), age);
            faults.push_back(buf);
            any_stale = true;
          }
        };
      checkInput("lidar1", msg1_received_, last_msg1_stamp_, last_msg1_latency_);
      checkInput("lidar2", msg2_received_, last_msg2_stamp_, last_msg2_latency_);

      if (!sync_received_) {
        faults.push_back("scan: not published yet");
        any_stale = true;
      } else {
        const double scan_age = (current - last_scan_stamp_).seconds();
        addValue("scan/age_s", scan_age);
        if (scan_age > kStaleTimeoutSec) {
          char buf[64];
          std::snprintf(buf, sizeof(buf), "scan: stale (%.1f s)", scan_age);
          faults.push_back(buf);
          any_stale = true;
        } else if (scan_age > kScanDelayWarnSec) {
          char buf[64];
          std::snprintf(buf, sizeof(buf), "scan: delayed (%.2f s)", scan_age);
          faults.push_back(buf);
        }
      }

      if (!faults.empty()) {
        status.level = any_stale ?
          diagnostic_msgs::msg::DiagnosticStatus::STALE :
          diagnostic_msgs::msg::DiagnosticStatus::WARN;
        status.message = joinStageNames(faults);
        
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000, "lidar_health: %s", status.message.c_str());
      }
    }

    diagnostic_msgs::msg::DiagnosticArray diag_msg;
    diag_msg.header.stamp = current;
    diag_msg.status.push_back(status);
    pub_diag_->publish(diag_msg);
  }

  void syncedCloudsCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg1,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg2)
  {
    if (self_filter_reload_pending_) {
      loadSelfFilterParams();
      self_filter_reload_pending_ = false;
    }

    const tf2::TimePoint tf_time = tf2::TimePointZero;
    updateSelfFilterTransforms(tf_time);

    std_msgs::msg::Header output_header;
    output_header.frame_id = target_frame_;
    output_header.stamp = olderStamp(msg1->header.stamp, msg2->header.stamp);
    last_scan_stamp_ = rclcpp::Time(output_header.stamp);
    sync_received_ = true;

    const auto output = pipeline_.process(
      msg1, msg2, tf_lidar1_, tf_lidar2_, self_filter_, scan_cfg_, pub_pointcloud_, output_header, get_logger());
    if (!hasAnyScanHits(output)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Pipeline produced no scan hits this cycle.");
    }

    if (pub_pointcloud_ && output.merged_cloud && pub_cloud_) {
      pub_cloud_->publish(*output.merged_cloud);
    }

    if (pub_self_filter_markers_ && pub_markers_) {
      visualization_msgs::msg::MarkerArray marker_array;
      self_filter_.appendSelfFilterMarkers(marker_array, target_frame_, now());
      pub_markers_->publish(marker_array);
    }

    auto scan = std::make_shared<sensor_msgs::msg::LaserScan>();
    // scan->header.stamp = now();
    scan->header.stamp = output_header.stamp;
    scan->header.frame_id = "laser";
    scan->angle_min = scan_cfg_.angle_min;
    scan->angle_max = scan_cfg_.angle_max;
    scan->angle_increment = scan_cfg_.angle_increment;
    scan->range_min = self_filter_config_.base_radius;
    scan->range_max = scan_cfg_.range_max;
    scan->ranges = output.ranges;
    pub_->publish(*scan);
  }

  static bool isSelfFilterGeometryParameter(const std::string & name)
  {
    return name == "frame_id" || name == "arm_line_start_frame" ||
           name == "arm_line_start_height_frame" || name == "arm_line_end_frame" ||
           name == "self_filter_box_frames" || name == "self_filter_box_names" ||
           name == "self_filter_box_groups" || name == "self_filter_box_origin_x" ||
           name == "self_filter_box_origin_y" || name == "self_filter_box_origin_z" ||
           name == "self_filter_box_rpy_roll" || name == "self_filter_box_rpy_pitch" ||
           name == "self_filter_box_rpy_yaw" || name == "self_filter_box_half_extents_x" ||
           name == "self_filter_box_half_extents_y" || name == "self_filter_box_half_extents_z";
  }

  static bool isLidarInputParameter(const std::string & name)
  {
    return name == "lidar1_topic" || name == "lidar2_topic" ||
           name == "lidar1_frame" || name == "lidar2_frame";
  }

  static rcl_interfaces::msg::SetParametersResult accept()
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    return result;
  }

  // Every rejection names the parameter, echoes the value that was sent and
  // states the constraint, so `ros2 param set` prints something actionable.
  template<typename T>
  static rcl_interfaces::msg::SetParametersResult reject(
    const std::string & name, const T & value, const std::string & requirement)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = false;
    std::ostringstream oss;
    oss << name << " = " << value << " is invalid: " << requirement << '.';
    result.reason = oss.str();
    return result;
  }

  static rcl_interfaces::msg::SetParametersResult rejectImmutable(
    const std::string & name, const std::string & why)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = false;
    result.reason = name + " cannot be changed at runtime: " + why;
    return result;
  }

  double valueAfterSet(
    const std::vector<rclcpp::Parameter> & parameters,
    const std::string & name) const
  {
    for (const auto & param : parameters) {
      if (param.get_name() == name) {
        return param.as_double();
      }
    }
    return get_parameter(name).as_double();
  }

  rcl_interfaces::msg::SetParametersResult validateParameters(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    try {
      for (const auto & param : parameters) {
        const auto & name = param.get_name();

        if (isSelfFilterGeometryParameter(name)) {
          return rejectImmutable(
            name, "the self-filter geometry is generated from the URDF when the node "
            "launches. Restart the node with the new value");
        }

        if (isLidarInputParameter(name)) {
          return rejectImmutable(
            name, "This cant be modified at runtime, change the launch file or parameter YAML and restart the node");
        }

        if (name == "filter_type") {
          const auto value = param.as_string();
          try {
            (void)stretch_core::stagesFromFilterType(value);
          } catch (const std::exception &) {
            return reject(
              name, value,
              "must be one of: region, sor, sor_ransac, self, none, custom");
          }
        } else if (name == "scan_angle_increment_deg") {
          const double value = param.as_double();
          if (!isValidScanAngleIncrement(value)) {
            return reject(name, value, "must be a finite value in (0, 5] degrees");
          }
        } else if (name == "angle") {
          const double value = param.as_double();
          if (!std::isfinite(value) || value <= 0.0 || value > 90.0) {
            return reject(
              name, value,
              "the floor-plane angle tolerance must be a finite value in (0, 90] degrees");
          }
        } else if (name == "leaf_size" || name == "sor_stddev" || name == "range_max" ||
          name == "plane_fitting_threshold" || name == "speckle_range_tolerance")
        {
          const double value = param.as_double();
          if (!std::isfinite(value) || value <= 0.0) {
            return reject(name, value, "must be a finite value greater than 0");
          }
        } else if (name == "dist_rob" || name == "base_radius") {
          const double value = param.as_double();
          if (!std::isfinite(value) || value < 0.0) {
            return reject(name, value, "must be a finite value of 0 or more");
          }
        } else if (name == "sor_mean_k" || name == "speckle_min_points" ||
          name == "speckle_neighbor_window")
        {
          const int64_t value = param.as_int();
          if (value < 1) {
            return reject(name, value, "must be 1 or more");
          }
        } else if (name == "speckle_min_neighbors") {
          const int64_t value = param.as_int();
          if (value < 0) {
            return reject(name, value, "must be 0 or more");
          }
        } else if (name == "output_topic" || name == "pointcloud_topic" ||
          name == "self_filter_markers_topic")
        {
          const auto value = param.as_string();
          if (value.empty()) {
            return reject(
              name, "''", "a publisher cannot be created on an empty topic name");
          }
        }

        if (name == "z_min" || name == "z_max") {
          const double z_min = valueAfterSet(parameters, "z_min");
          const double z_max = valueAfterSet(parameters, "z_max");
          if (!std::isfinite(z_min) || !std::isfinite(z_max) || z_min >= z_max) {
            return reject(
              name, param.as_double(),
              "the region filter needs z_min < z_max (this set would give z_min=" +
              std::to_string(z_min) + ", z_max=" + std::to_string(z_max) + ")");
          }
        } else if (name == "floor_detect_z_min" || name == "floor_detect_z_max") {
          const double lo = valueAfterSet(parameters, "floor_detect_z_min");
          const double hi = valueAfterSet(parameters, "floor_detect_z_max");
          if (!std::isfinite(lo) || !std::isfinite(hi) || lo >= hi) {
            return reject(
              name, param.as_double(),
              "floor detection needs floor_detect_z_min < floor_detect_z_max (this set "
              "would give " + std::to_string(lo) + " and " + std::to_string(hi) + ")");
          }
        }

        // Anything without a check above is accepted. Rejecting unrecognised
        // names would block other callbacks from handling them. (as per ros node doc)
      }
    } catch (const std::exception & ex) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = false;
      result.reason = std::string("could not validate the requested parameters: ") + ex.what();
      return result;
    }
    return accept();
  }

  void applyParameters(const std::vector<rclcpp::Parameter> & parameters)
  {
    for (const auto & param : parameters) {
      const auto & name = param.get_name();

      RCLCPP_INFO(
        get_logger(), "Parameter %s set to %s", name.c_str(),
        param.value_to_string().c_str());

      if (name == "filter_type") {
        filter_type_ = param.as_string();
        try {
          configurePipeline();
        } catch (const std::exception & ex) {
          RCLCPP_ERROR(get_logger(), "Failed to apply filter_type: %s", ex.what());
        }
      } else if (name == "enable_self_robot_filter") {
        enable_self_robot_filter_ = param.as_bool();
        configurePipeline();
      } else if (name == "enable_region_filter") {
        enable_region_filter_ = param.as_bool();
        configurePipeline();
      } else if (name == "enable_sor_filter") {
        enable_sor_filter_ = param.as_bool();
        configurePipeline();
      } else if (name == "enable_floor_ransac_filter") {
        enable_floor_ransac_filter_ = param.as_bool();
        configurePipeline();
      } else if (name == "z_min") {
        pipeline_config_.region.z_min = static_cast<float>(param.as_double());
        pipeline_.setConfig(pipeline_config_);
      } else if (name == "z_max") {
        pipeline_config_.region.z_max = static_cast<float>(param.as_double());
        pipeline_.setConfig(pipeline_config_);
      } else if (name == "range_max") {
        pipeline_config_.region.range_max = static_cast<float>(param.as_double());
        scan_cfg_.range_max = pipeline_config_.region.range_max;
        pipeline_.setConfig(pipeline_config_);
      } else if (name == "scan_angle_increment_deg") {
        configureScanProjection(param.as_double());
      } else if (name == "base_radius") {
        self_filter_config_.base_radius = static_cast<float>(param.as_double());
        self_filter_.setConfig(self_filter_config_);
      } else if (stretch_core::isRobotSelfFilterParameter(name))
      {
        self_filter_reload_pending_ = true;
      } else if (name == "pub_self_filter_markers") {
        pub_self_filter_markers_ = param.as_bool();
        setupMarkerPublisher();
      } else if (name == "self_filter_markers_topic") {
        self_filter_markers_topic_ = param.as_string();
        setupMarkerPublisher();
      } else if (name == "dist_rob") {
        pipeline_config_.sor.dist_rob = param.as_double();
        pipeline_.setConfig(pipeline_config_);
      } else if (name == "leaf_size") {
        pipeline_config_.sor.leaf_size = param.as_double();
        pipeline_.setConfig(pipeline_config_);
      } else if (name == "sor_mean_k") {
        pipeline_config_.sor.sor_mean_k = static_cast<int>(param.as_int());
        pipeline_.setConfig(pipeline_config_);
      } else if (name == "sor_stddev") {
        pipeline_config_.sor.sor_stddev = param.as_double();
        pipeline_.setConfig(pipeline_config_);
      } else if (name == "plane_fitting_threshold") {
        pipeline_config_.floor.plane_fitting_threshold = param.as_double();
        pipeline_.setConfig(pipeline_config_);
      } else if (name == "angle") {
        pipeline_config_.floor.angle_deg = param.as_double();
        pipeline_.setConfig(pipeline_config_);
      } else if (name == "floor_detect_z_min") {
        pipeline_config_.floor.floor_detect_z_min = static_cast<float>(param.as_double());
        pipeline_.setConfig(pipeline_config_);
      } else if (name == "floor_detect_z_max") {
        pipeline_config_.floor.floor_detect_z_max = static_cast<float>(param.as_double());
        pipeline_.setConfig(pipeline_config_);
      } else if (name == "speckle_filter_enabled") {
        pipeline_config_.speckle_filter_enabled = param.as_bool();
        pipeline_.setConfig(pipeline_config_);
      } else if (name == "speckle_min_points") {
        pipeline_config_.speckle_min_points = param.as_int();
        pipeline_.setConfig(pipeline_config_);
      } else if (name == "speckle_neighbor_window") {
        pipeline_config_.speckle_neighbor_window = param.as_int();
        pipeline_.setConfig(pipeline_config_);
      } else if (name == "speckle_min_neighbors") {
        pipeline_config_.speckle_min_neighbors = param.as_int();
        pipeline_.setConfig(pipeline_config_);
      } else if (name == "speckle_range_tolerance") {
        pipeline_config_.speckle_range_tolerance = static_cast<float>(param.as_double());
        pipeline_.setConfig(pipeline_config_);
      } else if (name == "pub_pointcloud") {
        pub_pointcloud_ = param.as_bool();
        if (pub_pointcloud_) {
          pub_cloud_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            pointcloud_topic_, rclcpp::SensorDataQoS());
        } else {
          pub_cloud_.reset();
        }
      } else if (name == "pointcloud_topic") {
        pointcloud_topic_ = param.as_string();
        if (pub_pointcloud_) {
          pub_cloud_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            pointcloud_topic_, rclcpp::SensorDataQoS());
        }
      } else if (name == "output_topic") {
        scan_topic_ = param.as_string();
        pub_ = create_publisher<sensor_msgs::msg::LaserScan>(scan_topic_, rclcpp::SensorDataQoS());
      } else {
        RCLCPP_WARN(
          get_logger(),
          "%s was set but this node has no runtime handler for it.", name.c_str());
      }
    }
  }

  stretch_core::DualLidarPipeline pipeline_;
  stretch_core::DualLidarPipelineConfig pipeline_config_;
  stretch_core::PipelineStages stages_{0};
  stretch_core::RobotSelfFilter self_filter_;
  stretch_core::RobotSelfFilterConfig self_filter_config_;
  stretch_core::ScanProjectionConfig scan_cfg_;

  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2>;

  static constexpr double kStaleTimeoutSec = 1.0;
  static constexpr double kScanDelayWarnSec = 0.5;
  static constexpr double kMaxPairIntervalSec = 0.2;
  static constexpr double kInterMessageLowerBoundSec = 0.09;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_diag_;
  rclcpp::TimerBase::SharedPtr diag_timer_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub1_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub2_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr post_param_callback_handle_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;

  rclcpp::Time last_msg1_stamp_;
  rclcpp::Time last_msg2_stamp_;
  rclcpp::Time last_scan_stamp_;
  double last_msg1_latency_{0.0};
  double last_msg2_latency_{0.0};
  bool subscriptions_active_{false};
  bool msg1_received_{false};
  bool msg2_received_{false};
  bool sync_received_{false};

  std::string filter_type_;
  std::string lidar1_topic_;
  std::string lidar2_topic_;
  std::string lidar1_frame_;
  std::string lidar2_frame_;
  std::string scan_topic_;
  std::string pointcloud_topic_;
  std::string target_frame_;
  std::string self_filter_markers_topic_;
  double scan_angle_increment_deg_{0.05};

  Eigen::Matrix4f tf_lidar1_;
  Eigen::Matrix4f tf_lidar2_;
  bool pub_pointcloud_{false};
  bool enable_self_robot_filter_{true};
  bool enable_region_filter_{true};
  bool enable_sor_filter_{false};
  bool enable_floor_ransac_filter_{false};
  bool pub_self_filter_markers_{false};
  bool self_filter_reload_pending_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DualLidarLaserScanNode>();

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
