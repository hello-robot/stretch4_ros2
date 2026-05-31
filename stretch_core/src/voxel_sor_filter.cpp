#include "stretch_core/voxel_sor_filter.hpp"

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

namespace stretch_core
{

pcl::PointCloud<pcl::PointXYZ>::Ptr VoxelSorFilter::filter(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_in) const
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr roi_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr outside_roi_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  const float dist = static_cast<float>(config_.dist_rob);

  for (const auto & pt : cloud_in->points) {
    if (pt.x >= -dist && pt.x <= dist && pt.y >= -dist && pt.y <= dist) {
      roi_cloud->points.push_back(pt);
    } else {
      outside_roi_cloud->points.push_back(pt);
    }
  }

  if (roi_cloud->empty()) {
    return cloud_in;
  }

  pcl::VoxelGrid<pcl::PointXYZ> voxel;
  voxel.setInputCloud(roi_cloud);
  const float leaf = static_cast<float>(config_.leaf_size);
  voxel.setLeafSize(leaf, leaf, leaf);
  pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_filtered(new pcl::PointCloud<pcl::PointXYZ>());
  voxel.filter(*voxel_filtered);

  if (voxel_filtered->empty()) {
    return nullptr;
  }

  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(voxel_filtered);
  sor.setMeanK(config_.sor_mean_k);
  sor.setStddevMulThresh(config_.sor_stddev);
  pcl::PointCloud<pcl::PointXYZ>::Ptr sor_filtered(new pcl::PointCloud<pcl::PointXYZ>());
  sor.filter(*sor_filtered);

  pcl::PointCloud<pcl::PointXYZ>::Ptr merged(new pcl::PointCloud<pcl::PointXYZ>);
  *merged = *sor_filtered;
  *merged += *outside_roi_cloud;
  return merged;
}

}  // namespace stretch_core
