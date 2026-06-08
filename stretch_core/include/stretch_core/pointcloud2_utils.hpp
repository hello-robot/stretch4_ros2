#pragma once

#include <cstddef>
#include <string>

#include <builtin_interfaces/msg/time.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

namespace stretch_core
{

int fieldOffset(const sensor_msgs::msg::PointCloud2 & cloud, const std::string & name);

bool hasField(const sensor_msgs::msg::PointCloud2 & cloud, const std::string & name);

bool fieldsMatch(
  const std::vector<sensor_msgs::msg::PointField> & a,
  const std::vector<sensor_msgs::msg::PointField> & b);

size_t pointCount(const sensor_msgs::msg::PointCloud2 & cloud);

builtin_interfaces::msg::Time olderStamp(
  const builtin_interfaces::msg::Time & a,
  const builtin_interfaces::msg::Time & b);

sensor_msgs::msg::PointCloud2 makeCompactCloudTemplate(
  const sensor_msgs::msg::PointCloud2 & reference);

sensor_msgs::msg::PointCloud2 mergePointClouds(
  const sensor_msgs::msg::PointCloud2 & left,
  const sensor_msgs::msg::PointCloud2 & right,
  const std_msgs::msg::Header & header);

void readPointXyz(
  const sensor_msgs::msg::PointCloud2 & cloud,
  size_t index,
  float & x,
  float & y,
  float & z);

void writePointXyz(
  sensor_msgs::msg::PointCloud2 & cloud,
  size_t index,
  float x,
  float y,
  float z);

void copyPoint(
  const sensor_msgs::msg::PointCloud2 & src,
  size_t src_index,
  sensor_msgs::msg::PointCloud2 & dst,
  size_t dst_index);

}  // namespace stretch_core
