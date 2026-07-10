#pragma once

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <limits>

#include "stretch_core/pipeline_stages.hpp"
#include "stretch_core/region_filter.hpp"
#include "stretch_core/robot_self_filter.hpp"
#include "stretch_core/sor_filter.hpp"

namespace stretch_core
{

struct PointCloudOutputFilterConfig
{
  float z_min{std::numeric_limits<float>::quiet_NaN()};
  float z_max{1.5f};
  float range_max{2.0f};
};

struct PreparedDualLidarCloud
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2;
  pcl::PointCloud<pcl::PointXYZ>::Ptr merged;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_holdback;
};

class DualLidarOutputPipeline
{
public:
  void setConfig(
    const RegionFilterConfig & region_config,
    const SorFilterConfig & sor_config,
    const PointCloudOutputFilterConfig & pointcloud_config);
  void setStages(PipelineStages stages) {stages_ = stages;}

  PreparedDualLidarCloud prepareLaserScanCloud(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg1,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg2,
    const Eigen::Matrix4f & tf_lidar1,
    const Eigen::Matrix4f & tf_lidar2,
    RobotSelfFilter & self_filter) const;

  pcl::PointCloud<pcl::PointXYZ>::Ptr makePointCloudOutput(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & filtered_scan_cloud,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & z_holdback_cloud,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & floor_holdback_cloud) const;

private:
  struct ScanSplitCloud
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr scan;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_holdback;
  };

  pcl::PointCloud<pcl::PointXYZ>::Ptr transformFiniteCloud(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg,
    const Eigen::Matrix4f & tf_matrix) const;

  ScanSplitCloud splitScanCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud) const;

  bool pointPassesPointCloudZ(float z) const;

  pcl::PointCloud<pcl::PointXYZ>::Ptr applySelfFilter(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
    RobotSelfFilter & self_filter) const;

  pcl::PointCloud<pcl::PointXYZ>::Ptr applyPointCloudOutputFilter(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud) const;

  PreparedDualLidarCloud applySharedExpensiveFilters(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1,
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2,
    RobotSelfFilter & self_filter) const;

  pcl::PointCloud<pcl::PointXYZ>::Ptr mergeClouds(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_1,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_2) const;

  RegionFilter region_filter_;
  RegionFilterConfig region_config_;
  SorFilter sor_filter_;
  PointCloudOutputFilterConfig pointcloud_config_;
  PipelineStages stages_{0};
};

void finalizePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud);

}  // namespace stretch_core
