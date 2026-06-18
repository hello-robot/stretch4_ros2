#include "stretch_core/sor_filter.hpp"

#include <algorithm>
#include <cstdint>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

namespace stretch_core
{

namespace
{

struct RoiSplit
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr near_robot;
  pcl::PointCloud<pcl::PointXYZ>::Ptr outside_roi;
};

void finalizeCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud)
{
  cloud->width = static_cast<uint32_t>(cloud->points.size());
  cloud->height = 1;
  cloud->is_dense = true;
}

bool inNearRobotRoi(const pcl::PointXYZ & pt, float dist)
{
  return pt.x >= -dist && pt.x <= dist && pt.y >= -dist && pt.y <= dist;
}

RoiSplit splitNearRobotRoi(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_in,
  float dist)
{
  RoiSplit split;
  split.near_robot.reset(new pcl::PointCloud<pcl::PointXYZ>());
  split.outside_roi.reset(new pcl::PointCloud<pcl::PointXYZ>());
  split.near_robot->points.reserve(cloud_in->points.size());
  split.outside_roi->points.reserve(cloud_in->points.size());

  for (const auto & pt : cloud_in->points) {
    if (inNearRobotRoi(pt, dist)) {
      split.near_robot->points.push_back(pt);
    } else {
      split.outside_roi->points.push_back(pt);
    }
  }

  finalizeCloud(split.near_robot);
  finalizeCloud(split.outside_roi);
  return split;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr mergeNearAndOutside(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & near_robot,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & outside_roi)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr merged(new pcl::PointCloud<pcl::PointXYZ>());
  merged->points.reserve(near_robot->points.size() + outside_roi->points.size());
  merged->points.insert(merged->points.end(), near_robot->points.begin(), near_robot->points.end());
  merged->points.insert(merged->points.end(), outside_roi->points.begin(), outside_roi->points.end());
  finalizeCloud(merged);
  return merged;
}

}  // namespace

pcl::PointCloud<pcl::PointXYZ>::Ptr SorFilter::filter(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_in) const
{
  return removeStatisticalOutliersNearRobot(voxelDownsampleNearRobot(cloud_in));
}

pcl::PointCloud<pcl::PointXYZ>::Ptr SorFilter::voxelDownsampleNearRobot(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_in) const
{
  if (!cloud_in || cloud_in->empty()) {
    return cloud_in;
  }

  const float dist = static_cast<float>(config_.dist_rob);
  const RoiSplit split = splitNearRobotRoi(cloud_in, dist);

  if (split.near_robot->empty()) {
    return cloud_in;
  }

  pcl::VoxelGrid<pcl::PointXYZ> voxel;
  voxel.setInputCloud(split.near_robot);
  const float leaf = static_cast<float>(config_.leaf_size);
  voxel.setLeafSize(leaf, leaf, leaf);
  pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_filtered(new pcl::PointCloud<pcl::PointXYZ>());
  voxel.filter(*voxel_filtered);

  if (voxel_filtered->empty()) {
    return split.outside_roi;
  }

  finalizeCloud(voxel_filtered);
  return mergeNearAndOutside(voxel_filtered, split.outside_roi);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr SorFilter::removeStatisticalOutliersNearRobot(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_in) const
{
  if (!cloud_in || cloud_in->empty()) {
    return cloud_in;
  }

  const float dist = static_cast<float>(config_.dist_rob);
  const RoiSplit split = splitNearRobotRoi(cloud_in, dist);

  if (split.near_robot->empty()) {
    return cloud_in;
  }

  if (split.near_robot->points.size() <= static_cast<size_t>(std::max(config_.sor_mean_k, 1))) {
    return cloud_in;
  }

  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(split.near_robot);
  sor.setMeanK(config_.sor_mean_k);
  sor.setStddevMulThresh(config_.sor_stddev);
  pcl::PointCloud<pcl::PointXYZ>::Ptr sor_filtered(new pcl::PointCloud<pcl::PointXYZ>());
  sor.filter(*sor_filtered);

  finalizeCloud(sor_filtered);
  return mergeNearAndOutside(sor_filtered, split.outside_roi);
}

}  // namespace stretch_core
