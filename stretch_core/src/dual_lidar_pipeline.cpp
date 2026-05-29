#include "stretch_core/dual_lidar_pipeline.hpp"

#include <algorithm>
#include <cmath>

#include <omp.h>
#include <rclcpp/logging.hpp>

#include <pcl/filters/filter.h>

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

}  // namespace

void DualLidarPipeline::setConfig(const DualLidarPipelineConfig & config)
{
  config_ = config;
  voxel_sor_filter_.setConfig(config_.voxel_sor);
  floor_filter_.setConfig(config_.floor);
  region_filter_.setConfig(config_.region);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr DualLidarPipeline::extractAndPreFilter(
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

    if (!region_filter_.passes(p3, stages_)) {
      continue;
    }
    if (self_filter.isWithinSelfFilterGate(p3) &&
      self_filter.isSelfFiltered(p3))
    {
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
  const std::optional<std::array<float, 4>> & floor_coeffs,
  const ScanProjectionConfig & scan_cfg,
  bool build_debug_cloud,
  PipelineOutput & output) const
{
  if (build_debug_cloud && !output.debug_cloud) {
    output.debug_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
  }

  const bool has_floor = floor_coeffs.has_value();
  float a = 0.0f;
  float b = 0.0f;
  float c = 0.0f;
  float d = 0.0f;
  float norm = 1.0f;
  const float floor_thresh = static_cast<float>(config_.floor.plane_fitting_threshold);

  if (has_floor) {
    a = (*floor_coeffs)[0];
    b = (*floor_coeffs)[1];
    c = (*floor_coeffs)[2];
    d = (*floor_coeffs)[3];
    norm = std::sqrt(a * a + b * b + c * c);
    if (norm <= 0.0f) {
      norm = 1.0f;
    }
  }

  for (const auto & pt : cloud->points) {
    if (has_floor) {
      const float dist = std::fabs(a * pt.x + b * pt.y + c * pt.z + d) / norm;
      if (dist <= floor_thresh) {
        continue;
      }
    }

    const float r = std::hypot(pt.x, pt.y);
    const float theta = std::atan2(pt.y, pt.x);
    const int idx = static_cast<int>((theta - scan_cfg.angle_min) / scan_cfg.angle_increment);
    if (idx >= 0 && idx < scan_cfg.num_ranges) {
      const size_t bin = static_cast<size_t>(idx);
      output.ranges[bin] = std::min(output.ranges[bin], r);
      if (output.hit_counts.size() == output.ranges.size()) {
        ++output.hit_counts[bin];
      }

      if (build_debug_cloud && output.debug_cloud) {
        output.debug_cloud->points.push_back(pt);
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
  bool build_debug_cloud,
  rclcpp::Logger logger) const
{
  PipelineOutput output;
  output.ranges.assign(static_cast<size_t>(scan_cfg.num_ranges), scan_cfg.range_max);
  output.hit_counts.assign(static_cast<size_t>(scan_cfg.num_ranges), 0);
  if (build_debug_cloud) {
    output.debug_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
  }

  if (!hasStage(stages_, PipelineStage::VoxelSor)) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2;

#pragma omp parallel sections
    {
#pragma omp section
      {
        cloud_1 = extractAndPreFilter(msg1, tf_lidar1, self_filter);
      }
#pragma omp section
      {
        cloud_2 = extractAndPreFilter(msg2, tf_lidar2, self_filter);
      }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr merged(new pcl::PointCloud<pcl::PointXYZ>());
    *merged = *cloud_1 + *cloud_2;

    projectPointsFused(merged, std::nullopt, scan_cfg, build_debug_cloud, output);
    applySpeckleFilter(scan_cfg, output);
    return output;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2;

#pragma omp parallel sections
  {
#pragma omp section
    {
      const auto prefiltered = extractAndPreFilter(msg1, tf_lidar1, self_filter);
      cloud_1 = voxel_sor_filter_.filter(prefiltered);
    }
#pragma omp section
    {
      const auto prefiltered = extractAndPreFilter(msg2, tf_lidar2, self_filter);
      cloud_2 = voxel_sor_filter_.filter(prefiltered);
    }
  }

  if (!cloud_1 || !cloud_2) {
    return output;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr merged(new pcl::PointCloud<pcl::PointXYZ>());
  *merged = *cloud_1 + *cloud_2;

  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*merged, *merged, indices);

  std::optional<std::array<float, 4>> floor_coeffs;
  if (hasStage(stages_, PipelineStage::FloorRansac)) {
    floor_coeffs = floor_filter_.getFloorCoefficients(merged, logger);
  }

  projectPointsFused(merged, floor_coeffs, scan_cfg, build_debug_cloud, output);
  applySpeckleFilter(scan_cfg, output);
  return output;
}

}  // namespace stretch_core
