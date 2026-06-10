#include "stretch_core/dual_lidar_pipeline.hpp"

#include <algorithm>
#include <cmath>

#include <omp.h>
#include <pcl/filters/filter.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/logging.hpp>

namespace stretch_core
{

namespace
{

bool hasValidHit(
  const PipelineOutput & output,
  const ScanProjectionConfig & scan_cfg,
  size_t index)
{
  return output.hit_counts[index] > 0 &&
         std::isfinite(output.ranges[index]) &&
         output.ranges[index] < scan_cfg.range_max;
}

bool isFullCircleScan(const ScanProjectionConfig & scan_cfg)
{
  const float span = scan_cfg.angle_max - scan_cfg.angle_min;
  const float full_circle = 2.0f * static_cast<float>(M_PI);
  return span >= full_circle - (1.5f * scan_cfg.angle_increment);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr cullFloorPoints(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
  const std::array<float, 4> & coeffs,
  float threshold)
{
  const float a = coeffs[0];
  const float b = coeffs[1];
  const float c = coeffs[2];
  const float d = coeffs[3];
  float norm = std::sqrt(a * a + b * b + c * c);
  if (norm <= 0.0f) {
    norm = 1.0f;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr culled(new pcl::PointCloud<pcl::PointXYZ>());
  culled->points.reserve(cloud->points.size());

  for (const auto & pt : cloud->points) {
    const float dist = std::fabs(a * pt.x + b * pt.y + c * pt.z + d) / norm;
    if (dist > threshold) {
      culled->points.emplace_back(pt);
    }
  }

  culled->width = static_cast<uint32_t>(culled->points.size());
  culled->height = 1;
  culled->is_dense = true;
  return culled;
}

}  // namespace

void DualLidarPipeline::setConfig(const DualLidarPipelineConfig & config)
{
  config_ = config;
  voxel_sor_filter_.setConfig(config_.voxel_sor);
  floor_filter_.setConfig(config_.floor);
  region_filter_.setConfig(config_.region);
}

bool DualLidarPipeline::passesPointFilters(
  const Eigen::Vector3f & point,
  RobotSelfFilter & self_filter) const
{
  if (hasStage(stages_, PipelineStage::Region) && !region_filter_.passes(point, stages_)) {
    return false;
  }
  if (hasStage(stages_, PipelineStage::SelfRobot) &&
    self_filter.isWithinSelfFilterGate(point) &&
    self_filter.isSelfFiltered(point))
  {
    return false;
  }
  return true;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr DualLidarPipeline::filterAndCompactXyz(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg,
  const Eigen::Matrix4f & tf_matrix,
  RobotSelfFilter & self_filter) const
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    const Eigen::Vector4f pt(*iter_x, *iter_y, *iter_z, 1.0f);
    const Eigen::Vector4f pt_tf = tf_matrix * pt;
    const Eigen::Vector3f p3(pt_tf.x(), pt_tf.y(), pt_tf.z());

    if (!passesPointFilters(p3, self_filter)) {
      continue;
    }

    cloud->points.emplace_back(p3.x(), p3.y(), p3.z());
  }

  cloud->width = static_cast<uint32_t>(cloud->points.size());
  cloud->height = 1;
  cloud->is_dense = true;
  return cloud;
}

void DualLidarPipeline::projectPointsFused(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
  const ScanProjectionConfig & scan_cfg,
  PipelineOutput & output) const
{
  for (const auto & pt : cloud->points) {
    const float r = std::hypot(pt.x, pt.y);
    const float theta = std::atan2(pt.y, pt.x);
    const int idx = static_cast<int>((theta - scan_cfg.angle_min) / scan_cfg.angle_increment);
    if (idx >= 0 && idx < scan_cfg.num_ranges) {
      const size_t bin = static_cast<size_t>(idx);
      output.ranges[bin] = std::min(output.ranges[bin], r);
      if (output.hit_counts.size() == output.ranges.size()) {
        ++output.hit_counts[bin];
      }
    }
  }
}

void DualLidarPipeline::applySpeckleFilter(
  const ScanProjectionConfig & scan_cfg,
  PipelineOutput & output) const
{
  if (!config_.speckle_filter_enabled ||
    config_.speckle_min_points <= 0 ||
    config_.speckle_neighbor_window <= 0 ||
    config_.speckle_min_neighbors <= 0)
  {
    return;
  }

  if (output.ranges.empty() || output.hit_counts.size() != output.ranges.size()) {
    return;
  }

  const int num_ranges = static_cast<int>(output.ranges.size());
  const bool wrap_scan = isFullCircleScan(scan_cfg);
  const float range_tolerance = std::max(0.0f, config_.speckle_range_tolerance);
  std::vector<float> filtered_ranges = output.ranges;

  for (int i = 0; i < num_ranges; ++i) {
    const size_t bin = static_cast<size_t>(i);
    if (!hasValidHit(output, scan_cfg, bin) ||
      output.hit_counts[bin] >= config_.speckle_min_points)
    {
      continue;
    }

    int similar_neighbors = 0;
    for (int offset = -config_.speckle_neighbor_window;
      offset <= config_.speckle_neighbor_window;
      ++offset)
    {
      if (offset == 0) {
        continue;
      }

      int neighbor = i + offset;
      if (wrap_scan) {
        neighbor %= num_ranges;
        if (neighbor < 0) {
          neighbor += num_ranges;
        }
      } else if (neighbor < 0 || neighbor >= num_ranges) {
        continue;
      }

      const size_t neighbor_bin = static_cast<size_t>(neighbor);
      if (hasValidHit(output, scan_cfg, neighbor_bin) &&
        std::abs(output.ranges[neighbor_bin] - output.ranges[bin]) <= range_tolerance)
      {
        ++similar_neighbors;
        if (similar_neighbors >= config_.speckle_min_neighbors) {
          break;
        }
      }
    }

    if (similar_neighbors < config_.speckle_min_neighbors) {
      filtered_ranges[bin] = scan_cfg.range_max;
      output.hit_counts[bin] = 0;
    }
  }

  output.ranges.swap(filtered_ranges);
}

PipelineOutput DualLidarPipeline::process(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg1,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg2,
  const Eigen::Matrix4f & tf_lidar1,
  const Eigen::Matrix4f & tf_lidar2,
  RobotSelfFilter & self_filter,
  const ScanProjectionConfig & scan_cfg,
  bool pub_pointcloud,
  const std_msgs::msg::Header & output_header,
  rclcpp::Logger logger) const
{
  PipelineOutput output;
  output.ranges.assign(static_cast<size_t>(scan_cfg.num_ranges), scan_cfg.range_max);
  output.hit_counts.assign(static_cast<size_t>(scan_cfg.num_ranges), 0);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2;

#pragma omp parallel sections
  {
#pragma omp section
    {
      cloud_1 = filterAndCompactXyz(msg1, tf_lidar1, self_filter);
    }
#pragma omp section
    {
      cloud_2 = filterAndCompactXyz(msg2, tf_lidar2, self_filter);
    }
  }

  if (hasStage(stages_, PipelineStage::VoxelSor)) {
#pragma omp parallel sections
    {
#pragma omp section
      {
        cloud_1 = voxel_sor_filter_.filter(cloud_1);
      }
#pragma omp section
      {
        cloud_2 = voxel_sor_filter_.filter(cloud_2);
      }
    }
  }

  if (!cloud_1 || !cloud_2) {
    return output;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr merged(new pcl::PointCloud<pcl::PointXYZ>());
  *merged = *cloud_1 + *cloud_2;

  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*merged, *merged, indices);

  pcl::PointCloud<pcl::PointXYZ>::Ptr scan_cloud = merged;

  std::optional<std::array<float, 4>> floor_coeffs;
  if (hasStage(stages_, PipelineStage::FloorRansac)) {
    floor_coeffs = floor_filter_.getFloorCoefficients(merged, logger);
    if (floor_coeffs.has_value()) {
      scan_cloud = cullFloorPoints(
        merged, *floor_coeffs,
        static_cast<float>(config_.floor.plane_fitting_threshold));
    }
  }

  if (pub_pointcloud) {
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*scan_cloud, cloud_msg);
    cloud_msg.header = output_header;
    output.merged_cloud = cloud_msg;
  }

  projectPointsFused(scan_cloud, scan_cfg, output);
  applySpeckleFilter(scan_cfg, output);
  return output;
}

}  // namespace stretch_core
