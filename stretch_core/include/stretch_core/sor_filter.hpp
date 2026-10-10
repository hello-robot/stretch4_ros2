#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/types.h>

namespace stretch_core
{

struct SorFilterConfig
{
  double dist_rob{2.5};
  double leaf_size{0.05};
  int sor_mean_k{50};
  double sor_stddev{0.3};
};

class SorFilter
{
public:
  void setConfig(const SorFilterConfig & config) {config_ = config;}

  pcl::PointCloud<pcl::PointXYZ>::Ptr filter(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_in) const;

  pcl::PointCloud<pcl::PointXYZ>::Ptr voxelDownsampleNearRobot(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_in) const;

  pcl::PointCloud<pcl::PointXYZ>::Ptr removeStatisticalOutliersNearRobot(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_in) const;

  bool statisticalInlierIndices(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_in,
    pcl::Indices & inliers) const;

private:
  SorFilterConfig config_;
};

}  // namespace stretch_core
