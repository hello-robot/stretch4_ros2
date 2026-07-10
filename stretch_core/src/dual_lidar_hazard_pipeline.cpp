#include "stretch_core/dual_lidar_hazard_pipeline.hpp"

#include <algorithm>
#include <cmath>

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

struct FloorCullResult
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr scan_cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr floor_holdback;
};

FloorCullResult splitFloorPoints(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
  const std::array<float, 4> & coeffs,
  float threshold)
{
  FloorCullResult result;
  result.scan_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
  result.floor_holdback.reset(new pcl::PointCloud<pcl::PointXYZ>());
  if (!cloud) {
    finalizePointCloud(result.scan_cloud);
    finalizePointCloud(result.floor_holdback);
    return result;
  }

  const float a = coeffs[0];
  const float b = coeffs[1];
  const float c = coeffs[2];
  const float d = coeffs[3];
  float norm = std::sqrt(a * a + b * b + c * c);
  if (norm <= 0.0f) {
    norm = 1.0f;
  }

  result.scan_cloud->points.reserve(cloud->points.size());
  result.floor_holdback->points.reserve(cloud->points.size());

  for (const auto & pt : cloud->points) {
    const float dist = std::fabs(a * pt.x + b * pt.y + c * pt.z + d) / norm;
    if (dist > threshold) {
      result.scan_cloud->points.emplace_back(pt);
    } else {
      result.floor_holdback->points.emplace_back(pt);
    }
  }

  finalizePointCloud(result.scan_cloud);
  finalizePointCloud(result.floor_holdback);
  return result;
}

}  // namespace

void DualLidarHazardPipeline::setConfig(const DualLidarHazardPipelineConfig & config)
{
  config_ = config;
  floor_filter_.setConfig(config_.floor);
  output_pipeline_.setConfig(config_.region, config_.sor, config_.pointcloud);
}

void DualLidarHazardPipeline::setStages(PipelineStages stages)
{
  stages_ = stages;
  output_pipeline_.setStages(stages_);
}

void DualLidarHazardPipeline::projectPointsFused(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
  const ScanProjectionConfig & scan_cfg,
  PipelineOutput & output) const
{
  if (!cloud) {
    return;
  }

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

void DualLidarHazardPipeline::applySpeckleFilter(
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

PipelineOutput DualLidarHazardPipeline::process(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg1,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg2,
  const Eigen::Matrix4f & tf_lidar1,
  const Eigen::Matrix4f & tf_lidar2,
  RobotSelfFilter & self_filter,
  const ScanProjectionConfig & scan_cfg,
  bool pub_laserscan,
  bool pub_pointcloud,
  const std_msgs::msg::Header & output_header,
  rclcpp::Logger logger) const
{
  PipelineOutput output;
  output.ranges.assign(static_cast<size_t>(scan_cfg.num_ranges), scan_cfg.range_max);
  output.hit_counts.assign(static_cast<size_t>(scan_cfg.num_ranges), 0);

  const auto scan_prepared = output_pipeline_.prepareLaserScanCloud(
    msg1, msg2, tf_lidar1, tf_lidar2, self_filter);
  if (!scan_prepared.merged) {
    return output;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr scan_cloud = scan_prepared.merged;
  pcl::PointCloud<pcl::PointXYZ>::Ptr floor_holdback;

  if (hasStage(stages_, PipelineStage::FloorRansac)) {
    const auto floor_coeffs = floor_filter_.getFloorCoefficients(scan_cloud, logger);
    if (floor_coeffs.has_value()) {
      const auto floor_split = splitFloorPoints(
        scan_cloud, *floor_coeffs,
        static_cast<float>(config_.floor.plane_fitting_threshold));
      scan_cloud = floor_split.scan_cloud;
      floor_holdback = floor_split.floor_holdback;
    }
  }

  if (pub_pointcloud) {
    const auto cloud_for_output = output_pipeline_.makePointCloudOutput(
      scan_cloud, scan_prepared.pointcloud_holdback, floor_holdback);
    if (cloud_for_output) {
      sensor_msgs::msg::PointCloud2 cloud_msg;
      pcl::toROSMsg(*cloud_for_output, cloud_msg);
      cloud_msg.header = output_header;
      output.merged_cloud = cloud_msg;
    }
  }

  if (pub_laserscan) {
    projectPointsFused(scan_cloud, scan_cfg, output);
    applySpeckleFilter(scan_cfg, output);
  }
  return output;
}

}  // namespace stretch_core
