#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/logger.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

#include "stretch_core/dual_lidar_output_pipeline.hpp"
#include "stretch_core/dual_lidar_pipeline.hpp"
#include "stretch_core/floor_plane_filter.hpp"
#include "stretch_core/pipeline_stages.hpp"
#include "stretch_core/robot_self_filter.hpp"

namespace stretch_core
{

struct DualLidarHazardPipelineConfig
{
  RegionFilterConfig region;
  SorFilterConfig sor;
  FloorPlaneFilterConfig floor;
  PointCloudOutputFilterConfig pointcloud;
  bool speckle_filter_enabled{true};
  int speckle_min_points{2};
  int speckle_neighbor_window{3};
  int speckle_min_neighbors{2};
  float speckle_range_tolerance{0.15f};
};

class DualLidarHazardPipeline
{
public:
  void setConfig(const DualLidarHazardPipelineConfig & config);
  void setStages(PipelineStages stages);

  PipelineOutput process(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg1,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg2,
    const Eigen::Matrix4f & tf_lidar1,
    const Eigen::Matrix4f & tf_lidar2,
    RobotSelfFilter & self_filter,
    const ScanProjectionConfig & scan_cfg,
    bool pub_laserscan,
    bool pub_pointcloud,
    const std_msgs::msg::Header & output_header,
    rclcpp::Logger logger) const;

private:
  void projectPointsFused(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
    const ScanProjectionConfig & scan_cfg,
    PipelineOutput & output) const;

  void applySpeckleFilter(
    const ScanProjectionConfig & scan_cfg,
    PipelineOutput & output) const;

  DualLidarHazardPipelineConfig config_;
  PipelineStages stages_{0};
  FloorPlaneFilter floor_filter_;
  DualLidarOutputPipeline output_pipeline_;
};

}  // namespace stretch_core
