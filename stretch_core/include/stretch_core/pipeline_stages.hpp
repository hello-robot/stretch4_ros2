#pragma once

#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace stretch_core
{

enum class PipelineStage : uint8_t
{
  SelfRobot = 1 << 0,
  Region = 1 << 1,
  VoxelSor = 1 << 2,
  FloorRansac = 1 << 3,
};

using PipelineStages = uint8_t;

inline PipelineStages operator|(PipelineStage a, PipelineStage b)
{
  return static_cast<PipelineStages>(a) | static_cast<PipelineStages>(b);
}

inline PipelineStages operator|(PipelineStages a, PipelineStage b)
{
  return a | static_cast<PipelineStages>(b);
}

inline bool hasStage(PipelineStages stages, PipelineStage stage)
{
  return (stages & static_cast<PipelineStages>(stage)) != 0;
}

inline PipelineStages stagesFromFilterType(const std::string & filter_type)
{
  if (filter_type == "region") {
    return PipelineStage::SelfRobot | PipelineStage::Region;
  }
  if (filter_type == "sor") {
    return PipelineStage::SelfRobot | PipelineStage::Region | PipelineStage::VoxelSor;
  }
  if (filter_type == "sor_ransac") {
    return PipelineStage::SelfRobot | PipelineStage::Region | PipelineStage::VoxelSor |
           PipelineStage::FloorRansac;
  }
  throw std::invalid_argument("Unknown filter_type: " + filter_type);
}

inline std::vector<std::string> stageNames(PipelineStages stages)
{
  std::vector<std::string> names;
  if (hasStage(stages, PipelineStage::SelfRobot)) {
    names.emplace_back("SelfRobot");
  }
  if (hasStage(stages, PipelineStage::Region)) {
    names.emplace_back("Region");
  }
  if (hasStage(stages, PipelineStage::VoxelSor)) {
    names.emplace_back("VoxelSor");
  }
  if (hasStage(stages, PipelineStage::FloorRansac)) {
    names.emplace_back("FloorRansac");
  }
  return names;
}

}  // namespace stretch_core
