#include "stretch_core/floor_plane_filter.hpp"

#include <cmath>

#include <rclcpp/logging.hpp>

#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>
#include <pcl/segmentation/sac_segmentation.h>

namespace stretch_core
{

std::optional<std::array<float, 4>> FloorPlaneFilter::getFloorCoefficients(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_in,
  rclcpp::Logger logger) const
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr roi_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  for (const auto & pt : cloud_in->points) {
    if (pt.z >= config_.floor_detect_z_min && pt.z <= config_.floor_detect_z_max) {
      roi_cloud->points.push_back(pt);
    }
  }

  if (roi_cloud->empty()) {
    RCLCPP_WARN(logger, "No points in ROI for floor detection; skipping floor culling.");
    return std::nullopt;
  }

  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setAxis(Eigen::Vector3f(0.0f, 0.0f, 1.0f));
  seg.setEpsAngle(static_cast<float>(config_.angle_deg * M_PI / 180.0));
  seg.setMethodType(pcl::SAC_MSAC);
  seg.setDistanceThreshold(config_.plane_fitting_threshold);
  seg.setInputCloud(roi_cloud);
  seg.segment(*inliers, *coefficients);

  if (coefficients->values.size() < 4) {
    RCLCPP_WARN(
      logger,
      "Plane detection failed (%zu coefficients); skipping floor culling.",
      coefficients->values.size());
    return std::nullopt;
  }

  return std::array<float, 4>{
    coefficients->values[0],
    coefficients->values[1],
    coefficients->values[2],
    coefficients->values[3],
  };
}

}  // namespace stretch_core
