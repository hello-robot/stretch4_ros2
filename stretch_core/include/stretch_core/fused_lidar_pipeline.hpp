#pragma once

// One pass over the two lidar clouds producing BOTH the merged point cloud and the
// LaserScan.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <optional>
#include <vector>

#include <Eigen/Dense>
#include <rclcpp/logger.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

#include "stretch_core/floor_plane_filter.hpp"
#include "stretch_core/point_cloud_layout.hpp"
#include "stretch_core/robot_self_filter.hpp"
#include "stretch_core/scan_speckle_filter.hpp"
#include "stretch_core/sor_filter.hpp"
#include "stretch_core/voxel_hash_set.hpp"

namespace stretch_core
{

struct FusedScanConfig
{
  float angle_min{-static_cast<float>(M_PI)};
  float angle_max{static_cast<float>(M_PI)};
  float angle_increment{0.1f * static_cast<float>(M_PI) / 180.0f};
  float range_max{30.0f};
  int num_ranges{3600};
};


struct FusedPipelineConfig
{
  float voxel_leaf_size{0.05f};

  // Radius inside which the self-filter geometry test can fire. 
  float near_field_radius{1.5f};

  // The height band outside which NO check can fire: not the scan band, but the
  // self-filter gate span, which strictly contains it. The robot's mast top (1.554) and
  // head (1.571) sit above the scan's z_max (1.472) and below the gate top (1.572), so a
  // fast lane keyed on z_max would leave the robot's own head in the published cloud.
  float filter_z_bot{-0.078f};
  float filter_z_top{1.572f};

  // Scan band. scan_z_min follows enable_floor_ransac: with RANSAC on it drops to
  // floor_detect_z_min so the plane has floor points to fit against.
  float scan_z_min{0.107f};
  float scan_z_max{1.472f};

  bool enable_self_filter{true};
  bool enable_floor_ransac{true};
  int floor_sample_stride{16};
  bool enable_sor{false};

  FloorPlaneFilterConfig floor;
  SorFilterConfig sor;
  SpeckleFilterConfig speckle;
};

struct FusedPipelineStats
{
  size_t input_points{0};
  size_t cloud_points{0};
  size_t scan_points{0};
  size_t self_filtered_points{0};
  size_t fast_lane_points{0};
};

struct FusedPipelineOutput
{
  sensor_msgs::msg::PointCloud2 cloud;
  ScanBins scan;
  FusedPipelineStats stats;
};

class FusedLidarPipeline
{
public:
  void setConfig(const FusedPipelineConfig & config);
  const FusedPipelineConfig & config() const {return config_;}

  // msg_a/msg_b must already have been checked for matching fields and point_step.
  void process(
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
    FusedPipelineOutput & output);

private:
  // How a point was classified by the first pass. 
  enum class PointClass : uint8_t
  {
    Dead = 0,   // non-finite input
    Fast = 1,   // outside the gate z span: cloud only, no radius, no gate test, no scan
    Gate = 2,   // inside the gate: voxel-thin first, then self-filter
    Open = 3,   // in the gate z span but beyond its radius: no self-filter possible
  };

  struct ScratchPoint
  {
    float x{0.0f};
    float y{0.0f};
    float z{0.0f};
    float r2{0.0f};
    uint64_t key{0};
    uint32_t source_index{0};
    uint8_t source{0};
    PointClass cls{PointClass::Dead};
  };

  // Pass 1, parallel: transform, classify, compute the voxel key. No shared state.
  void classifyCloud(
    const sensor_msgs::msg::PointCloud2 & msg,
    const LinearTransform3f & tf,
    const PointFieldLayout & layout,
    uint8_t source,
    size_t scratch_offset);

  // Pass 2, serial: voxelize, self-filter, cloud compaction, scan candidate list.
  void reduceAndCompact(
    const sensor_msgs::msg::PointCloud2 & msg_a,
    const sensor_msgs::msg::PointCloud2 & msg_b,
    const PointFieldLayout & layout,
    RobotSelfFilter & self_filter,
    const FusedScanConfig & scan_cfg,
    bool publish_cloud,
    FusedPipelineOutput & output);

  // Pass 3, parallel: project the scan candidates into angular bins.
  void projectScan(const FusedScanConfig & scan_cfg, FusedPipelineOutput & output) const;

  class ScanThinGrid
  {
public:
    void configure(float half_extent_xy, float z_min, float z_max, float leaf)
    {
      if (leaf <= 0.0f || half_extent_xy <= 0.0f || z_max <= z_min) {
        nx_ = ny_ = nz_ = 0;
        words_.clear();
        return;
      }
      inv_leaf_ = 1.0f / leaf;
      x0_ = -half_extent_xy;
      y0_ = -half_extent_xy;
      z0_ = z_min;
      nx_ = static_cast<int>(std::ceil(2.0f * half_extent_xy * inv_leaf_)) + 1;
      ny_ = nx_;
      nz_ = static_cast<int>(std::ceil((z_max - z_min) * inv_leaf_)) + 1;
      const size_t cells = static_cast<size_t>(nx_) * static_cast<size_t>(ny_) *
        static_cast<size_t>(nz_);
      words_.assign((cells + 63) / 64, 0);
    }

    bool enabled() const {return nx_ > 0;}

    void clear() {std::fill(words_.begin(), words_.end(), 0);}

    bool insert(float x, float y, float z)
    {
      const int ix = clampIndex((x - x0_) * inv_leaf_, nx_);
      const int iy = clampIndex((y - y0_) * inv_leaf_, ny_);
      const int iz = clampIndex((z - z0_) * inv_leaf_, nz_);
      const size_t cell =
        (static_cast<size_t>(ix) * static_cast<size_t>(ny_) + static_cast<size_t>(iy)) *
        static_cast<size_t>(nz_) + static_cast<size_t>(iz);
      uint64_t & word = words_[cell >> 6];
      const uint64_t mask = 1ULL << (cell & 63);
      if (word & mask) {
        return false;
      }
      word |= mask;
      return true;
    }

private:
    static int clampIndex(float v, int n)
    {
      const int i = static_cast<int>(std::floor(v));
      return std::min(std::max(i, 0), n - 1);
    }

    float inv_leaf_{0.0f};
    float x0_{0.0f};
    float y0_{0.0f};
    float z0_{0.0f};
    int nx_{0};
    int ny_{0};
    int nz_{0};
    std::vector<uint64_t> words_;
  };


  std::optional<std::array<float, 4>> fitFloorPlane(rclcpp::Logger logger);

  FusedPipelineConfig config_;
  float inv_leaf_{20.0f};

  std::vector<ScratchPoint> scratch_;
  std::vector<uint32_t> scan_candidates_;
  // [scan_obstacle_count_, size()) are floor returns, retained so SOR can still count them
  // as neighbours. Without them a small object on the floor looks like an isolated island.
  size_t scan_obstacle_count_{0};
  VoxelHashSet voxels_;
  // Per-lidar thinning grids for the scan path.
  ScanThinGrid scan_grid_[2];
  pcl::PointCloud<pcl::PointXYZ>::Ptr floor_sample_;
  // Per-lidar scan clouds for the SOR path, reused across frames.
  pcl::PointCloud<pcl::PointXYZ>::Ptr sor_scan_[2];
  // Floor flags parallel to sor_scan_, so a floor point can support the statistic without
  // being projected into the scan.
  std::vector<uint8_t> sor_floor_[2];
  FloorPlaneFilter floor_filter_;
  SorFilter sor_filter_;
};

}  // namespace stretch_core
