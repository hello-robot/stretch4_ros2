#pragma once

#include <array>
#include <optional>

#include <rclcpp/logger.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <cmath>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>

#include "stretch_core/floor_plane_filter.hpp"
#include "stretch_core/pipeline_stages.hpp"
#include "stretch_core/region_filter.hpp"
#include "stretch_core/robot_self_filter.hpp"
#include "stretch_core/voxel_sor_filter.hpp"

namespace stretch_core
{

struct ScanProjectionConfig
{
  float angle_min{-static_cast<float>(M_PI)};
  float angle_max{static_cast<float>(M_PI)};
  float angle_increment{0.05f * static_cast<float>(M_PI) / 180.0f};
  float range_max{30.0f};
  int num_ranges{0};
};

struct DualLidarPipelineConfig
{
  RegionFilterConfig region;
  VoxelSorFilterConfig voxel_sor;
  FloorPlaneFilterConfig floor;
  bool speckle_filter_enabled{true};
  int speckle_min_points{2};
  int speckle_neighbor_window{3};
  int speckle_min_neighbors{2};
  float speckle_range_tolerance{0.15f};
};

struct PipelineOutput
{
  std::vector<float> ranges;
  std::vector<int> hit_counts;
  std::optional<sensor_msgs::msg::PointCloud2> merged_cloud;
};

class DualLidarPipeline
{
public:
  void setConfig(const DualLidarPipelineConfig & config);
  void setStages(PipelineStages stages) {stages_ = stages;}

  PipelineOutput process(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg1,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg2,
    const Eigen::Matrix4f & tf_lidar1,
    const Eigen::Matrix4f & tf_lidar2,
    RobotSelfFilter & self_filter,
    const ScanProjectionConfig & scan_cfg,
    bool pub_pointcloud,
    const std_msgs::msg::Header & output_header,
    rclcpp::Logger logger) const;

private:
  bool passesPointFilters(const Eigen::Vector3f & point, RobotSelfFilter & self_filter) const;

  pcl::PointCloud<pcl::PointXYZ>::Ptr filterAndCompactXyz(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg,
    const Eigen::Matrix4f & tf_matrix,
    RobotSelfFilter & self_filter) const;

  void projectPointsFused(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
    const ScanProjectionConfig & scan_cfg,
    PipelineOutput & output) const;

  void applySpeckleFilter(
    const ScanProjectionConfig & scan_cfg,
    PipelineOutput & output) const;

  DualLidarPipelineConfig config_;
  PipelineStages stages_{0};
  VoxelSorFilter voxel_sor_filter_;
  FloorPlaneFilter floor_filter_;
  RegionFilter region_filter_;
};

}  // namespace stretch_core
