#include "stretch_core/dual_lidar_output_pipeline.hpp"

#include <algorithm>
#include <cmath>

#include <omp.h>

namespace stretch_core
{

void finalizePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud)
{
  cloud->width = static_cast<uint32_t>(cloud->points.size());
  cloud->height = 1;
  cloud->is_dense = true;
}

void DualLidarOutputPipeline::setConfig(
  const RegionFilterConfig & region_config,
  const SorFilterConfig & sor_config,
  const PointCloudOutputFilterConfig & pointcloud_config)
{
  region_filter_.setConfig(region_config);
  region_config_ = region_config;
  sor_filter_.setConfig(sor_config);
  pointcloud_config_ = pointcloud_config;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr DualLidarOutputPipeline::transformFiniteCloud(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg,
  const Eigen::Matrix4f & tf_matrix) const
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  const size_t input_count = static_cast<size_t>(msg->width) * static_cast<size_t>(msg->height);
  cloud->points.reserve(input_count);
  const Eigen::Matrix3f rotation = tf_matrix.block<3, 3>(0, 0);
  const Eigen::Vector3f translation = tf_matrix.block<3, 1>(0, 3);

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y) || !std::isfinite(*iter_z)) {
      continue;
    }
    const Eigen::Vector3f p3 = rotation * Eigen::Vector3f(*iter_x, *iter_y, *iter_z) + translation;
    cloud->points.emplace_back(p3.x(), p3.y(), p3.z());
  }

  finalizePointCloud(cloud);
  return cloud;
}

bool DualLidarOutputPipeline::pointPassesPointCloudZ(float z) const
{
  if (std::isfinite(pointcloud_config_.z_min) && z < pointcloud_config_.z_min) {
    return false;
  }
  if (std::isfinite(pointcloud_config_.z_max) && z > pointcloud_config_.z_max) {
    return false;
  }
  return true;
}

DualLidarOutputPipeline::ScanSplitCloud DualLidarOutputPipeline::splitScanCloud(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud) const
{
  ScanSplitCloud split;
  if (!cloud) {
    return split;
  }

  split.scan.reset(new pcl::PointCloud<pcl::PointXYZ>());
  split.pointcloud_holdback.reset(new pcl::PointCloud<pcl::PointXYZ>());
  split.scan->points.reserve(cloud->points.size());

  const bool region_enabled = hasStage(stages_, PipelineStage::Region);
  const bool scan_z_min_enabled =
    region_enabled && !hasStage(stages_, PipelineStage::FloorRansac) &&
    std::isfinite(region_config_.z_min);
  const bool scan_z_max_enabled = region_enabled && std::isfinite(region_config_.z_max);
  const bool scan_range_enabled = region_enabled && std::isfinite(region_config_.range_max);

  for (const auto & pt : cloud->points) {
    const bool rejected_by_scan_z_min = scan_z_min_enabled && pt.z < region_config_.z_min;
    const bool rejected_by_scan_z_max = scan_z_max_enabled && pt.z > region_config_.z_max;
    const bool rejected_by_scan_range =
      scan_range_enabled && std::hypot(pt.x, pt.y) > region_config_.range_max;

    if (!rejected_by_scan_z_min && !rejected_by_scan_z_max && !rejected_by_scan_range) {
      split.scan->points.push_back(pt);
      continue;
    }

    if ((rejected_by_scan_z_min || rejected_by_scan_z_max || rejected_by_scan_range) &&
      pointPassesPointCloudZ(pt.z))
    {
      split.pointcloud_holdback->points.push_back(pt);
    }
  }

  finalizePointCloud(split.scan);
  finalizePointCloud(split.pointcloud_holdback);
  return split;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr DualLidarOutputPipeline::applySelfFilter(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
  RobotSelfFilter & self_filter) const
{
  if (!cloud) {
    return cloud;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>());
  filtered->points.reserve(cloud->points.size());
  for (const auto & pt : cloud->points) {
    const Eigen::Vector3f point(pt.x, pt.y, pt.z);
    if (self_filter.isWithinSelfFilterGate(point) && self_filter.isSelfFiltered(point)) {
      continue;
    }
    filtered->points.push_back(pt);
  }

  finalizePointCloud(filtered);
  return filtered;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr DualLidarOutputPipeline::applyPointCloudOutputFilter(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud) const
{
  if (!cloud) {
    return cloud;
  }

  const float range_max = pointcloud_config_.range_max;
  const float range_max_sq = range_max * range_max;
  const bool use_range_limit = std::isfinite(range_max) && range_max > 0.0f;

  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>());
  filtered->points.reserve(cloud->points.size());
  for (const auto & pt : cloud->points) {
    if (!pointPassesPointCloudZ(pt.z)) {
      continue;
    }
    if (use_range_limit && ((pt.x * pt.x) + (pt.y * pt.y) > range_max_sq)) {
      continue;
    }
    filtered->points.push_back(pt);
  }

  finalizePointCloud(filtered);
  return filtered;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr DualLidarOutputPipeline::mergeClouds(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_1,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_2) const
{
  if (!cloud_1 || !cloud_2) {
    return nullptr;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr merged(new pcl::PointCloud<pcl::PointXYZ>());
  merged->points.reserve(cloud_1->points.size() + cloud_2->points.size());
  merged->points.insert(merged->points.end(), cloud_1->points.begin(), cloud_1->points.end());
  merged->points.insert(merged->points.end(), cloud_2->points.begin(), cloud_2->points.end());
  finalizePointCloud(merged);
  return merged;
}

PreparedDualLidarCloud DualLidarOutputPipeline::applySharedExpensiveFilters(
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1,
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2,
  RobotSelfFilter & self_filter) const
{
  PreparedDualLidarCloud prepared;
  prepared.cloud_1 = cloud_1;
  prepared.cloud_2 = cloud_2;

  const bool run_internal_voxel =
    hasStage(stages_, PipelineStage::SelfRobot) || hasStage(stages_, PipelineStage::Sor);
  if (run_internal_voxel) {
#pragma omp parallel sections
    {
#pragma omp section
      {
        prepared.cloud_1 = sor_filter_.voxelDownsampleNearRobot(prepared.cloud_1);
      }
#pragma omp section
      {
        prepared.cloud_2 = sor_filter_.voxelDownsampleNearRobot(prepared.cloud_2);
      }
    }
  }

  if (hasStage(stages_, PipelineStage::SelfRobot)) {
#pragma omp parallel sections
    {
#pragma omp section
      {
        prepared.cloud_1 = applySelfFilter(prepared.cloud_1, self_filter);
      }
#pragma omp section
      {
        prepared.cloud_2 = applySelfFilter(prepared.cloud_2, self_filter);
      }
    }
  }

  if (hasStage(stages_, PipelineStage::Sor)) {
#pragma omp parallel sections
    {
#pragma omp section
      {
        prepared.cloud_1 = sor_filter_.removeStatisticalOutliersNearRobot(prepared.cloud_1);
      }
#pragma omp section
      {
        prepared.cloud_2 = sor_filter_.removeStatisticalOutliersNearRobot(prepared.cloud_2);
      }
    }
  }

  prepared.merged = mergeClouds(prepared.cloud_1, prepared.cloud_2);
  return prepared;
}

PreparedDualLidarCloud DualLidarOutputPipeline::prepareLaserScanCloud(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg1,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg2,
  const Eigen::Matrix4f & tf_lidar1,
  const Eigen::Matrix4f & tf_lidar2,
  RobotSelfFilter & self_filter) const
{
  ScanSplitCloud split_1;
  ScanSplitCloud split_2;

#pragma omp parallel sections
  {
#pragma omp section
    {
      split_1 = splitScanCloud(transformFiniteCloud(msg1, tf_lidar1));
    }
#pragma omp section
    {
      split_2 = splitScanCloud(transformFiniteCloud(msg2, tf_lidar2));
    }
  }

  PreparedDualLidarCloud prepared = applySharedExpensiveFilters(split_1.scan, split_2.scan, self_filter);
  prepared.pointcloud_holdback = mergeClouds(split_1.pointcloud_holdback, split_2.pointcloud_holdback);
  return prepared;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr DualLidarOutputPipeline::makePointCloudOutput(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & filtered_scan_cloud,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & z_holdback_cloud,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & floor_holdback_cloud) const
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr combined(new pcl::PointCloud<pcl::PointXYZ>());
  const auto reserve_size =
    (filtered_scan_cloud ? filtered_scan_cloud->points.size() : 0) +
    (z_holdback_cloud ? z_holdback_cloud->points.size() : 0) +
    (floor_holdback_cloud ? floor_holdback_cloud->points.size() : 0);
  combined->points.reserve(reserve_size);

  const auto append_cloud = [&combined](const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud) {
    if (!cloud) {
      return;
    }
    combined->points.insert(combined->points.end(), cloud->points.begin(), cloud->points.end());
  };
  append_cloud(filtered_scan_cloud);
  append_cloud(z_holdback_cloud);
  append_cloud(floor_holdback_cloud);

  finalizePointCloud(combined);
  return applyPointCloudOutputFilter(combined);
}

}  // namespace stretch_core
