#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <chrono>
#include <cmath>
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
    setupPublishers();

    scan_cfg_.angle_min = -static_cast<float>(M_PI);
    scan_cfg_.angle_max = static_cast<float>(M_PI);
    scan_cfg_.angle_increment = 0.05f * static_cast<float>(M_PI) / 180.0f;
    scan_cfg_.range_max = pipeline_config_.region.range_max;
    scan_cfg_.num_ranges = static_cast<int>(
      (scan_cfg_.angle_max - scan_cfg_.angle_min) / scan_cfg_.angle_increment);

    if (pub_self_filter_markers_) {
      pub_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        self_filter_markers_topic_, rclcpp::QoS(1).reliable());
    }

    param_callback_handle_ = add_on_set_parameters_callback(
      std::bind(&DualLidarLaserScanNode::onParameterChange, this, std::placeholders::_1));
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
      tf_available_ = true;
      RCLCPP_INFO(get_logger(), "Transforms cached.");
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(get_logger(), "Waiting for TFs: %s", ex.what());
      return false;
    }
  }

  void activateSubscription()
  {
    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.reliable();
    sub1_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      lidar1_topic_, qos,
      std::bind(&DualLidarLaserScanNode::pointcloudCallback1, this, std::placeholders::_1));
    sub2_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      lidar2_topic_, qos,
      std::bind(&DualLidarLaserScanNode::pointcloudCallback2, this, std::placeholders::_1));
    timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&DualLidarLaserScanNode::timerCallback, this));
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

  void loadSelfFilterParams()
  {
    self_filter_config_ = stretch_core::loadRobotSelfFilterConfig(*this);
    self_filter_.setConfig(self_filter_config_);
  }

  void updateSelfFilterTransforms(const tf2::TimePoint & tf_time, bool force_markers)
  {
    if (self_filter_config_.filter_arm || force_markers) {
      self_filter_.updateArmSegment(
        tf_buffer_, target_frame_, tf_time, get_logger(), *get_clock(), force_markers);
    }
    if (self_filter_config_.filter_arm_shoulder || force_markers) {
      self_filter_.updateArmShoulderBox(
        tf_buffer_, target_frame_, tf_time, get_logger(), *get_clock(), force_markers);
    }
    if (self_filter_config_.filter_wrist || force_markers) {
      self_filter_.updateWristChain(
        tf_buffer_, target_frame_, tf_time, get_logger(), *get_clock(), force_markers);
    }
    if (self_filter_config_.filter_attachment || force_markers) {
      self_filter_.updateAttachmentBox(
        tf_buffer_, target_frame_, tf_time, get_logger(), *get_clock(),
        self_filter_config_.filter_attachment && force_markers);
    }
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

  tf2::TimePoint lidarTfTime() const
  {
    if (!msg1_) {
      return tf2::TimePointZero;
    }
    const rclcpp::Time stamp(msg1_->header.stamp);
    if (stamp.nanoseconds() <= 0) {
      return tf2::TimePointZero;
    }
    return tf2::TimePoint(std::chrono::nanoseconds(stamp.nanoseconds()));
  }

  void pointcloudCallback1(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (!tf_available_) {
      return;
    }
    msg1_ = msg;
  }

  void pointcloudCallback2(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (!tf_available_) {
      return;
    }
    msg2_ = msg;
  }


  void timerCallback()
  {
    if (!msg1_ || !msg2_) {
      RCLCPP_WARN(get_logger(), "One or both LIDAR messages not yet received.");
      return;
    }

    // const tf2::TimePoint tf_time = lidarTfTime();
    const tf2::TimePoint tf_time = tf2::TimePointZero;
    const bool force_markers = pub_self_filter_markers_;
    updateSelfFilterTransforms(tf_time, force_markers);

    std_msgs::msg::Header output_header;
    output_header.frame_id = target_frame_;
    output_header.stamp = olderStamp(msg1_->header.stamp, msg2_->header.stamp);

    const auto output = pipeline_.process(
      msg1_, msg2_, tf_lidar1_, tf_lidar2_, self_filter_, scan_cfg_, pub_pointcloud_, output_header, get_logger());
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
      self_filter_.appendSelfFilterMarkers(marker_array, target_frame_, now(), force_markers);
      pub_markers_->publish(marker_array);
    }

    auto scan = std::make_shared<sensor_msgs::msg::LaserScan>();
    scan->header.stamp = now();
    scan->header.frame_id = "laser";
    scan->angle_min = scan_cfg_.angle_min;
    scan->angle_max = scan_cfg_.angle_max;
    scan->angle_increment = scan_cfg_.angle_increment;
    scan->range_min = self_filter_config_.base_radius;
    scan->range_max = scan_cfg_.range_max;
    scan->ranges = output.ranges;
    pub_->publish(*scan);
  }

  rcl_interfaces::msg::SetParametersResult onParameterChange(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";

    for (const auto & param : parameters) {
      const auto & name = param.get_name();
      if (name == "filter_type") {
        filter_type_ = param.as_string();
        try {
          configurePipeline();
        } catch (const std::exception & ex) {
          result.successful = false;
          result.reason = ex.what();
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
      } else if (name == "base_radius") {
        self_filter_config_.base_radius = static_cast<float>(param.as_double());
        self_filter_.setConfig(self_filter_config_);
      } else if (name == "filter_base") {
        self_filter_config_.filter_base = param.as_bool();
        self_filter_.setConfig(self_filter_config_);
      } else if (stretch_core::isRobotSelfFilterParameter(name))
      {
        loadSelfFilterParams();
      } else if (name == "pub_self_filter_markers") {
        pub_self_filter_markers_ = param.as_bool();
        if (pub_self_filter_markers_) {
          pub_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            self_filter_markers_topic_, rclcpp::QoS(1).reliable());
        } else {
          pub_markers_.reset();
        }
      } else if (name == "dist_rob" || name == "leaf_size" || name == "sor_mean_k" ||
        name == "sor_stddev" || name == "plane_fitting_threshold" || name == "angle" ||
        name == "floor_detect_z_min" || name == "floor_detect_z_max")
      {
        loadParameters();
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
        setupPublishers();
      } else if (name == "pointcloud_topic") {
        pointcloud_topic_ = param.as_string();
        if (pub_pointcloud_) {
          pub_cloud_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            pointcloud_topic_, rclcpp::SensorDataQoS());
        }
      } else if (name == "output_topic") {
        scan_topic_ = param.as_string();
        pub_ = create_publisher<sensor_msgs::msg::LaserScan>(scan_topic_, rclcpp::SensorDataQoS());
      } else if (name == "frame_id" || name == "arm_line_start_frame" ||
        name == "arm_line_start_height_frame" || name == "arm_line_end_frame" ||
        name == "arm_shoulder_box_frame" || name == "wrist_chain_frames" ||
        name == "attachment_frame")
      {
        result.successful = false;
        result.reason = "Parameter cannot be changed at runtime.";
      } else {
        result.successful = false;
        result.reason = "Unknown parameter";
      }
    }
    return result;
  }

  stretch_core::DualLidarPipeline pipeline_;
  stretch_core::DualLidarPipelineConfig pipeline_config_;
  stretch_core::PipelineStages stages_{0};
  stretch_core::RobotSelfFilter self_filter_;
  stretch_core::RobotSelfFilterConfig self_filter_config_;
  stretch_core::ScanProjectionConfig scan_cfg_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub1_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub2_;
  rclcpp::TimerBase::SharedPtr timer_;
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;

  sensor_msgs::msg::PointCloud2::ConstSharedPtr msg1_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr msg2_;

  std::string filter_type_;
  std::string lidar1_topic_;
  std::string lidar2_topic_;
  std::string lidar1_frame_;
  std::string lidar2_frame_;
  std::string scan_topic_;
  std::string pointcloud_topic_;
  std::string target_frame_;
  std::string self_filter_markers_topic_;

  Eigen::Matrix4f tf_lidar1_;
  Eigen::Matrix4f tf_lidar2_;
  bool tf_available_{false};
  bool pub_pointcloud_{false};
  bool enable_self_robot_filter_{true};
  bool enable_region_filter_{true};
  bool enable_sor_filter_{false};
  bool enable_floor_ransac_filter_{false};
  bool pub_self_filter_markers_{false};
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

  node->activateSubscription();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
