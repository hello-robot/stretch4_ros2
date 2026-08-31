#include "stretch_core/fused_lidar_pipeline.hpp"

#include <algorithm>
#include <cstring>
#include <limits>

#include <omp.h>
#include <rclcpp/logging.hpp>

namespace stretch_core
{

namespace
{
constexpr float kNoHitRange = std::numeric_limits<float>::infinity();
}  // namespace

void FusedLidarPipeline::setConfig(const FusedPipelineConfig & config)
{
  config_ = config;
  inv_leaf_ = (config_.voxel_leaf_size > 0.0f) ? (1.0f / config_.voxel_leaf_size) : 0.0f;
  floor_filter_.setConfig(config_.floor);
  sor_filter_.setConfig(config_.sor);

  // SOR applied on scan
  const float scan_leaf = config_.enable_sor ?
    static_cast<float>(config_.sor.leaf_size) : config_.voxel_leaf_size;
  const float half_extent = config_.enable_sor ?
    std::max(static_cast<float>(config_.sor.dist_rob), config_.near_field_radius) :
    config_.near_field_radius;
  for (auto & grid : scan_grid_) {
    grid.configure(half_extent, config_.scan_z_min, config_.scan_z_max, scan_leaf);
  }
}

// Transform and classify. 
void FusedLidarPipeline::classifyCloud(
  const sensor_msgs::msg::PointCloud2 & msg,
  const LinearTransform3f & tf,
  const PointFieldLayout & layout,
  uint8_t source,
  size_t scratch_offset)
{
  const size_t count = pointCount(msg);
  const size_t point_step = layout.point_step;
  const int x_offset = layout.x_offset;
  const int y_offset = layout.y_offset;
  const int z_offset = layout.z_offset;

  const float z_bot = config_.filter_z_bot;
  const float z_top = config_.filter_z_top;
  const float gate_r2 = config_.near_field_radius * config_.near_field_radius;
  const float inv_leaf = inv_leaf_;
  const bool voxelize = inv_leaf > 0.0f;

  const uint8_t * base = msg.data.data();
  ScratchPoint * out = scratch_.data() + scratch_offset;

  #pragma omp parallel for schedule(static)
  for (size_t i = 0; i < count; ++i) {
    const uint8_t * src = base + i * point_step;
    ScratchPoint & p = out[i];
    p.source = source;
    p.source_index = static_cast<uint32_t>(i);

    float x = 0.0F;
    float y = 0.0F;
    float z = 0.0F;
    std::memcpy(&x, src + x_offset, sizeof(float));
    std::memcpy(&y, src + y_offset, sizeof(float));
    std::memcpy(&z, src + z_offset, sizeof(float));

    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      p.cls = PointClass::Dead;
      continue;
    }

    tf.transform(x, y, z, p.x, p.y, p.z);
    p.key = voxelize ? VoxelHashSet::key(p.x, p.y, p.z, inv_leaf) : 0;

    // Saves the height for the pointcloud because its not needed for the laserscan
    if (p.z > z_top || p.z < z_bot) {
      p.cls = PointClass::Fast;
      continue;
    }

    p.r2 = p.x * p.x + p.y * p.y;
    p.cls = (p.r2 < gate_r2) ? PointClass::Gate : PointClass::Open;
  }
}

// Voxelize, self-filter, compaction.
void FusedLidarPipeline::reduceAndCompact(
  const sensor_msgs::msg::PointCloud2 & msg_a,
  const sensor_msgs::msg::PointCloud2 & msg_b,
  const PointFieldLayout & layout,
  RobotSelfFilter & self_filter,
  const FusedScanConfig & scan_cfg,
  bool publish_cloud,
  FusedPipelineOutput & output)
{
  const size_t total = scratch_.size();
  const size_t point_step = layout.point_step;
  const int x_offset = layout.x_offset;
  const int y_offset = layout.y_offset;
  const int z_offset = layout.z_offset;
  const size_t xyz_end = static_cast<size_t>(z_offset) + sizeof(float);
  const bool xyz_contiguous = layout.xyzContiguous();

  const uint8_t * src_base[2] = {msg_a.data.data(), msg_b.data.data()};

  const bool voxelize = inv_leaf_ > 0.0f;
  const bool self_filter_on = config_.enable_self_filter;
  const float scan_z_min = config_.scan_z_min;
  const float scan_z_max = config_.scan_z_max;
  const float scan_r2 = scan_cfg.range_max * scan_cfg.range_max;

  const bool collect_floor = config_.enable_floor_ransac;
  const float floor_lo = config_.floor.floor_detect_z_min;
  const float floor_hi = config_.floor.floor_detect_z_max;
  const int stride = std::max(1, config_.floor_sample_stride);

  voxels_.reset(total);
  scan_candidates_.clear();
  floor_sample_->points.clear();

  const bool thin_scan = scan_grid_[0].enabled();
  const bool sor_on = config_.enable_sor;
  const float sor_box = static_cast<float>(config_.sor.dist_rob);
  if (thin_scan) {
    scan_grid_[0].clear();
    scan_grid_[1].clear();
  }

  uint8_t * dst_base = publish_cloud ? output.cloud.data.data() : nullptr;
  size_t written = 0;
  size_t self_filtered = 0;
  size_t fast_lane = 0;
  int floor_counter = 0;

  for (size_t i = 0; i < total; ++i) {
    const ScratchPoint & p = scratch_[i];
    if (p.cls == PointClass::Dead) {
      continue;
    }

    const bool representative = voxelize ? voxels_.insert(p.key) : true;

    const bool scan_candidate = p.cls != PointClass::Fast &&
      p.z >= scan_z_min && p.z <= scan_z_max && p.r2 < scan_r2;

    bool scan_keep = scan_candidate;
    if (scan_keep && thin_scan) {
      const bool in_thin_region = sor_on ?
        (p.cls == PointClass::Gate ||
        (p.x >= -sor_box && p.x <= sor_box && p.y >= -sor_box && p.y <= sor_box)) :
        (p.cls == PointClass::Gate);
      if (in_thin_region) {
        scan_keep = scan_grid_[p.source & 1].insert(p.x, p.y, p.z);
      }
    }

    if (p.cls == PointClass::Gate) {
      if (!representative && !scan_keep) {
        continue;
      }
      if (self_filter_on) {
        const Eigen::Vector3f point(p.x, p.y, p.z);
        if (self_filter.isSelfFiltered(point)) {
          ++self_filtered;
          continue;
        }
      }
    } else if (p.cls == PointClass::Fast) {
      ++fast_lane;
    }

    if (representative && publish_cloud) {
      const uint8_t * src = src_base[p.source] + static_cast<size_t>(p.source_index) * point_step;
      uint8_t * dst = dst_base + written * point_step;
      // The whole point rides across untouched -- intensity, ring and the absolute float64
      if (xyz_contiguous) {
        const size_t head_bytes = static_cast<size_t>(x_offset);
        if (head_bytes > 0) {
          std::memcpy(dst, src, head_bytes);
        }
        if (xyz_end < point_step) {
          std::memcpy(dst + xyz_end, src + xyz_end, point_step - xyz_end);
        }
      } else {
        std::memcpy(dst, src, point_step);
      }
      std::memcpy(dst + x_offset, &p.x, sizeof(float));
      std::memcpy(dst + y_offset, &p.y, sizeof(float));
      std::memcpy(dst + z_offset, &p.z, sizeof(float));
      ++written;
    }

    if (scan_keep) {
      scan_candidates_.push_back(static_cast<uint32_t>(i));
    }

    // Sample for the floor fit
    if (collect_floor && (p.cls != PointClass::Gate || representative) &&
      p.z >= floor_lo && p.z <= floor_hi)
    {
      if ((floor_counter++ % stride) == 0) {
        floor_sample_->points.emplace_back(p.x, p.y, p.z);
      }
    }
  }

  if (publish_cloud) {
    output.cloud.height = 1;
    output.cloud.width = static_cast<uint32_t>(written);
    output.cloud.row_step = output.cloud.point_step * output.cloud.width;
    output.cloud.data.resize(output.cloud.row_step);
    output.cloud.is_dense = true;
  }

  floor_sample_->width = static_cast<uint32_t>(floor_sample_->points.size());
  floor_sample_->height = 1;
  floor_sample_->is_dense = true;

  output.stats.cloud_points = written;
  scan_obstacle_count_ = scan_candidates_.size();
  output.stats.scan_points = scan_candidates_.size();
  output.stats.self_filtered_points = self_filtered;
  output.stats.fast_lane_points = fast_lane;
}

std::optional<std::array<float, 4>> FusedLidarPipeline::fitFloorPlane(rclcpp::Logger logger)
{
  if (!config_.enable_floor_ransac || floor_sample_->points.empty()) {
    return std::nullopt;
  }
  return floor_filter_.getFloorCoefficients(floor_sample_, logger);
}

// Project into angular bins. Parallel with per-thread range arrays
void FusedLidarPipeline::projectScan(
  const FusedScanConfig & scan_cfg,
  FusedPipelineOutput & output) const
{
  const size_t num_bins = static_cast<size_t>(scan_cfg.num_ranges);
  const float angle_min = scan_cfg.angle_min;
  const float inv_increment = 1.0f / scan_cfg.angle_increment;
  const size_t candidate_count = scan_obstacle_count_;

  std::vector<float> & ranges = output.scan.ranges;
  std::vector<int> & hits = output.scan.hit_counts;

  #pragma omp parallel
  {
    std::vector<float> local_ranges(num_bins, kNoHitRange);
    std::vector<int> local_hits(num_bins, 0);

    #pragma omp for schedule(static) nowait
    for (size_t idx = 0; idx < candidate_count; ++idx) {
      const ScratchPoint & p = scratch_[scan_candidates_[idx]];
      const float theta = std::atan2(p.y, p.x);
      const int bin = static_cast<int>((theta - angle_min) * inv_increment);
      if (bin < 0 || bin >= scan_cfg.num_ranges) {
        continue;
      }
      const float r = std::sqrt(p.r2);
      const size_t b = static_cast<size_t>(bin);
      if (r < local_ranges[b]) {
        local_ranges[b] = r;
      }
      ++local_hits[b];
    }

    #pragma omp critical
    {
      for (size_t b = 0; b < num_bins; ++b) {
        if (local_ranges[b] < ranges[b]) {
          ranges[b] = local_ranges[b];
        }
        hits[b] += local_hits[b];
      }
    }
  }
}


void FusedLidarPipeline::process(
  const sensor_msgs::msg::PointCloud2 & msg_a,
  const sensor_msgs::msg::PointCloud2 & msg_b,
  const LinearTransform3f & tf_a,
  const LinearTransform3f & tf_b,
  const PointFieldLayout & layout,
  RobotSelfFilter & self_filter,
  const FusedScanConfig & scan_cfg,
  const std_msgs::msg::Header & header,
  bool publish_cloud,
  rclcpp::Logger logger,
  FusedPipelineOutput & output)
{
  const size_t count_a = pointCount(msg_a);
  const size_t count_b = pointCount(msg_b);
  const size_t total = count_a + count_b;

  output.stats = FusedPipelineStats{};
  output.stats.input_points = total;
  output.scan.ranges.assign(static_cast<size_t>(scan_cfg.num_ranges), kNoHitRange);
  output.scan.hit_counts.assign(static_cast<size_t>(scan_cfg.num_ranges), 0);

  if (!floor_sample_) {
    floor_sample_.reset(new pcl::PointCloud<pcl::PointXYZ>());
  }

  scratch_.resize(total);

  if (publish_cloud) {
    output.cloud.header = header;
    output.cloud.fields = msg_a.fields;
    output.cloud.point_step = msg_a.point_step;
    output.cloud.is_bigendian = msg_a.is_bigendian;
    output.cloud.height = 1;
    // Sized for the worst case, then shrunk to the survivor count in reduceAndCompact.
    output.cloud.data.resize(total * static_cast<size_t>(msg_a.point_step));
  }

  classifyCloud(msg_a, tf_a, layout, 0, 0);
  classifyCloud(msg_b, tf_b, layout, 1, count_a);

  reduceAndCompact(msg_a, msg_b, layout, self_filter, scan_cfg, publish_cloud, output);

  // The plane has to exist before projection, because the scan applies it per point.
  const auto floor_coeffs = fitFloorPlane(logger);
  if (floor_coeffs.has_value()) {
    const float a = (*floor_coeffs)[0];
    const float b = (*floor_coeffs)[1];
    const float c = (*floor_coeffs)[2];
    const float d = (*floor_coeffs)[3];
    float norm = std::sqrt(a * a + b * b + c * c);
    if (norm <= 0.0f) {
      norm = 1.0f;
    }
    const float inv_norm = 1.0f / norm;
    const float threshold = static_cast<float>(config_.floor.plane_fitting_threshold);

    // Drop floor returns from the scan only. The floor points stay addressable in the tail 
    // so SOR can still see them as neighbours, while only the obstacle prefix is ever projected.
    const auto obstacle_end = std::partition(
      scan_candidates_.begin(), scan_candidates_.end(),
      [&](uint32_t idx) {
        const ScratchPoint & p = scratch_[idx];
        const float dist = std::fabs(a * p.x + b * p.y + c * p.z + d) * inv_norm;
        return dist > threshold;
      });
    scan_obstacle_count_ =
      static_cast<size_t>(std::distance(scan_candidates_.begin(), obstacle_end));
    output.stats.scan_points = scan_obstacle_count_;
  }

  if (config_.enable_sor) {
    // Statistical outlier removal
    output.scan.ranges.assign(static_cast<size_t>(scan_cfg.num_ranges), kNoHitRange);
    output.scan.hit_counts.assign(static_cast<size_t>(scan_cfg.num_ranges), 0);
    size_t kept = 0;

    const auto project_point = [&output, &scan_cfg](float x, float y) {
        const float r = std::hypot(x, y);
        const float theta = std::atan2(y, x);
        const int bin =
          static_cast<int>((theta - scan_cfg.angle_min) / scan_cfg.angle_increment);
        if (bin < 0 || bin >= scan_cfg.num_ranges) {
          return;
        }
        const size_t b = static_cast<size_t>(bin);
        output.scan.ranges[b] = std::min(output.scan.ranges[b], r);
        ++output.scan.hit_counts[b];
      };

    const float box = static_cast<float>(config_.sor.dist_rob);
    for (uint8_t s = 0; s < 2; ++s) {
      if (!sor_scan_[s]) {
        sor_scan_[s].reset(new pcl::PointCloud<pcl::PointXYZ>());
      }
      sor_scan_[s]->points.clear();
      sor_floor_[s].clear();
    }

    const size_t candidate_count = scan_candidates_.size();
    for (size_t i = 0; i < candidate_count; ++i) {
      const ScratchPoint & p = scratch_[scan_candidates_[i]];
      const bool is_floor = i >= scan_obstacle_count_;
      if (p.x >= -box && p.x <= box && p.y >= -box && p.y <= box) {
        sor_scan_[p.source & 1]->points.emplace_back(p.x, p.y, p.z);
        sor_floor_[p.source & 1].push_back(is_floor ? uint8_t{1} : uint8_t{0});
      } else if (!is_floor) {
        // Outside the box there is no statistic to apply, so obstacles pass through.
        project_point(p.x, p.y);
        ++kept;
      }
    }

    for (uint8_t s = 0; s < 2; ++s) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud = sor_scan_[s];
      cloud->width = static_cast<uint32_t>(cloud->points.size());
      cloud->height = 1;
      cloud->is_dense = true;
      const std::vector<uint8_t> & floor_flags = sor_floor_[s];

      pcl::Indices inliers;
      const bool statistic_ran = sor_filter_.statisticalInlierIndices(cloud, inliers);
      if (!statistic_ran) {
        // Box empty or too small to support the statistic: keep every obstacle, as before.
        for (size_t i = 0; i < cloud->points.size(); ++i) {
          if (floor_flags[i]) {
            continue;
          }
          const auto & pt = cloud->points[i];
          project_point(pt.x, pt.y);
          ++kept;
        }
        continue;
      }

      for (const auto index : inliers) {
        const size_t i = static_cast<size_t>(index);
        if (floor_flags[i]) {
          continue;
        }
        const auto & pt = cloud->points[i];
        project_point(pt.x, pt.y);
        ++kept;
      }
    }

    output.stats.scan_points = kept;
    applySpeckleFilter(
      config_.speckle, scan_cfg.angle_min, scan_cfg.angle_max, scan_cfg.angle_increment,
      scan_cfg.range_max, output.scan);
    return;
  }

  projectScan(scan_cfg, output);

  applySpeckleFilter(
    config_.speckle, scan_cfg.angle_min, scan_cfg.angle_max, scan_cfg.angle_increment,
    scan_cfg.range_max, output.scan);
}

}  // namespace stretch_core
