#pragma once

#include <cmath>

#include <Eigen/Dense>

#include "stretch_core/pipeline_stages.hpp"

namespace stretch_core
{

struct RegionFilterConfig
{
  float z_min{0.135f};
  float z_max{1.5f};
  float range_max{30.0f};
};

class RegionFilter
{
public:
  void setConfig(const RegionFilterConfig & config) {config_ = config;}

  bool passes(const Eigen::Vector3f & point, PipelineStages stages) const
  {
    if (!hasStage(stages, PipelineStage::Region)) {
      return true;
    }
    const float z = point.z();
    if (!hasStage(stages, PipelineStage::FloorRansac) && z < config_.z_min) {
      return false;
    }
    if (z > config_.z_max) {
      return false;
    }
    const float r = std::hypot(point.x(), point.y());
    if (r > config_.range_max) {
      return false;
    }
    return true;
  }

private:
  RegionFilterConfig config_;
};

}  // namespace stretch_core
