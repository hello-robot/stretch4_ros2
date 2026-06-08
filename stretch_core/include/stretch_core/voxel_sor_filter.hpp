#pragma once

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace stretch_core
{

struct VoxelSorFilterConfig
{
  double dist_rob{2.5};
  double leaf_size{0.05};
  int sor_mean_k{50};
  double sor_stddev{0.3};
};

class VoxelSorFilter
{
public:
  void setConfig(const VoxelSorFilterConfig & config) {config_ = config;}

  pcl::PointCloud<pcl::PointXYZ>::Ptr filter(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_in) const;

  sensor_msgs::msg::PointCloud2 filterPointCloud2(
    const sensor_msgs::msg::PointCloud2 & cloud_in) const;

private:
  VoxelSorFilterConfig config_;
};

}  // namespace stretch_core
