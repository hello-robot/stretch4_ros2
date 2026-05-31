#pragma once

#include <array>
#include <optional>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/logger.hpp>

namespace stretch_core
{

struct FloorPlaneFilterConfig
{
  double plane_fitting_threshold{0.1};
  double angle_deg{10.0};
  float floor_detect_z_min{-0.4f};
  float floor_detect_z_max{0.1f};
};

class FloorPlaneFilter
{
public:
  void setConfig(const FloorPlaneFilterConfig & config) {config_ = config;}

  std::optional<std::array<float, 4>> getFloorCoefficients(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_in,
    rclcpp::Logger logger) const;

private:
  FloorPlaneFilterConfig config_;
};

}  // namespace stretch_core
