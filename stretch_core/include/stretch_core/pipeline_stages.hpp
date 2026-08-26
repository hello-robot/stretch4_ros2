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
  Sor = 1 << 2,
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

inline PipelineStages stagesFromEnables(
  bool self_robot,
  bool region,
  bool sor,
  bool floor_ransac)
{
  PipelineStages stages = 0;
  if (self_robot) {
    stages = stages | PipelineStage::SelfRobot;
  }
  if (region) {
    stages = stages | PipelineStage::Region;
  }
  if (sor) {
    stages = stages | PipelineStage::Sor;
  }
  if (floor_ransac) {
    stages = stages | PipelineStage::FloorRansac;
  }
  return stages;
}

inline PipelineStages stagesFromFilterType(const std::string & filter_type)
{
  if (filter_type == "region") {
    return PipelineStage::SelfRobot | PipelineStage::Region;
  }
  if (filter_type == "sor") {
    return PipelineStage::SelfRobot | PipelineStage::Region | PipelineStage::Sor;
  }
  if (filter_type == "sor_ransac") {
    return PipelineStage::SelfRobot | PipelineStage::Region | PipelineStage::Sor |
           PipelineStage::FloorRansac;
  }
  if (filter_type == "self") {
    return static_cast<PipelineStages>(PipelineStage::SelfRobot);
  }
  if (filter_type == "none") {
    return 0;
  }
  if (filter_type == "custom") {
    return 0;
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
  if (hasStage(stages, PipelineStage::Sor)) {
    names.emplace_back("SOR");
  }
  if (hasStage(stages, PipelineStage::FloorRansac)) {
    names.emplace_back("FloorRansac");
  }
  return names;
}

}  // namespace stretch_core
