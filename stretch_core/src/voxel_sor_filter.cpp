#include "stretch_core/voxel_sor_filter.hpp"

#include <cmath>
#include <unordered_map>
#include <vector>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

#include "stretch_core/pointcloud2_utils.hpp"

namespace stretch_core
{

namespace
{

int64_t voxelKey(const float x, const float y, const float z, const float leaf)
{
  const int64_t ix = static_cast<int64_t>(std::floor(x / leaf));
  const int64_t iy = static_cast<int64_t>(std::floor(y / leaf));
  const int64_t iz = static_cast<int64_t>(std::floor(z / leaf));
  return (ix << 42) ^ (iy << 21) ^ iz;
}

}  // namespace

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

sensor_msgs::msg::PointCloud2 VoxelSorFilter::filterPointCloud2(
  const sensor_msgs::msg::PointCloud2 & cloud_in) const
{
  const size_t count = pointCount(cloud_in);
  if (count == 0) {
    return cloud_in;
  }

  const float dist = static_cast<float>(config_.dist_rob);
  const float leaf = static_cast<float>(config_.leaf_size);

  std::vector<size_t> near_indices;
  std::vector<size_t> far_indices;
  near_indices.reserve(count);
  far_indices.reserve(count);

  for (size_t i = 0; i < count; ++i) {
    float x = 0.0F;
    float y = 0.0F;
    float z = 0.0F;
    readPointXyz(cloud_in, i, x, y, z);
    if (x >= -dist && x <= dist && y >= -dist && y <= dist) {
      near_indices.push_back(i);
    } else {
      far_indices.push_back(i);
    }
  }

  if (near_indices.empty()) {
    return cloud_in;
  }

  std::unordered_map<int64_t, size_t> voxel_first_index;
  voxel_first_index.reserve(near_indices.size());
  for (const size_t index : near_indices) {
    float x = 0.0F;
    float y = 0.0F;
    float z = 0.0F;
    readPointXyz(cloud_in, index, x, y, z);
    const int64_t key = voxelKey(x, y, z, leaf);
    if (voxel_first_index.find(key) == voxel_first_index.end()) {
      voxel_first_index.emplace(key, index);
    }
  }

  std::vector<size_t> voxel_indices;
  voxel_indices.reserve(voxel_first_index.size());
  for (const auto & entry : voxel_first_index) {
    voxel_indices.push_back(entry.second);
  }

  if (voxel_indices.empty()) {
    return cloud_in;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr roi_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  roi_cloud->points.reserve(voxel_indices.size());
  for (const size_t index : voxel_indices) {
    float x = 0.0F;
    float y = 0.0F;
    float z = 0.0F;
    readPointXyz(cloud_in, index, x, y, z);
    roi_cloud->points.emplace_back(x, y, z);
  }
  roi_cloud->width = static_cast<uint32_t>(roi_cloud->points.size());
  roi_cloud->height = 1;
  roi_cloud->is_dense = true;

  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(roi_cloud);
  sor.setMeanK(config_.sor_mean_k);
  sor.setStddevMulThresh(config_.sor_stddev);
  std::vector<int> sor_indices;
  sor.filter(sor_indices);

  std::vector<size_t> kept_indices;
  kept_indices.reserve(sor_indices.size() + far_indices.size());
  for (const int sor_index : sor_indices) {
    if (sor_index >= 0 && static_cast<size_t>(sor_index) < voxel_indices.size()) {
      kept_indices.push_back(voxel_indices[static_cast<size_t>(sor_index)]);
    }
  }
  kept_indices.insert(kept_indices.end(), far_indices.begin(), far_indices.end());

  sensor_msgs::msg::PointCloud2 output = makeCompactCloudTemplate(cloud_in);
  output.width = static_cast<uint32_t>(kept_indices.size());
  output.row_step = output.point_step * output.width;
  output.data.resize(output.row_step);

  for (size_t i = 0; i < kept_indices.size(); ++i) {
    copyPoint(cloud_in, kept_indices[i], output, i);
  }

  return output;
}

}  // namespace stretch_core
