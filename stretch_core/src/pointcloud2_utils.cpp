#include "stretch_core/pointcloud2_utils.hpp"

#include <algorithm>
#include <cstring>

namespace stretch_core
{

int fieldOffset(const sensor_msgs::msg::PointCloud2 & cloud, const std::string & name)
{
  const auto it = std::find_if(
    cloud.fields.begin(), cloud.fields.end(),
    [&name](const sensor_msgs::msg::PointField & field) {
      return field.name == name;
    });
  return (it != cloud.fields.end()) ? static_cast<int>(it->offset) : -1;
}

bool hasField(const sensor_msgs::msg::PointCloud2 & cloud, const std::string & name)
{
  return fieldOffset(cloud, name) >= 0;
}

bool fieldsMatch(
  const std::vector<sensor_msgs::msg::PointField> & a,
  const std::vector<sensor_msgs::msg::PointField> & b)
{
  if (a.size() != b.size()) {
    return false;
  }
  for (size_t i = 0; i < a.size(); ++i) {
    if (a[i].name != b[i].name || a[i].offset != b[i].offset ||
      a[i].datatype != b[i].datatype || a[i].count != b[i].count)
    {
      return false;
    }
  }
  return true;
}

size_t pointCount(const sensor_msgs::msg::PointCloud2 & cloud)
{
  return static_cast<size_t>(cloud.width) * static_cast<size_t>(cloud.height);
}

builtin_interfaces::msg::Time olderStamp(
  const builtin_interfaces::msg::Time & a,
  const builtin_interfaces::msg::Time & b)
{
  if (a.sec != b.sec) {
    return (a.sec < b.sec) ? a : b;
  }
  return (a.nanosec < b.nanosec) ? a : b;
}

sensor_msgs::msg::PointCloud2 makeCompactCloudTemplate(
  const sensor_msgs::msg::PointCloud2 & reference)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.fields = reference.fields;
  cloud.point_step = reference.point_step;
  cloud.height = 1;
  cloud.width = 0;
  cloud.row_step = 0;
  cloud.is_bigendian = reference.is_bigendian;
  cloud.is_dense = reference.is_dense;
  return cloud;
}

sensor_msgs::msg::PointCloud2 mergePointClouds(
  const sensor_msgs::msg::PointCloud2 & left,
  const sensor_msgs::msg::PointCloud2 & right,
  const std_msgs::msg::Header & header)
{
  sensor_msgs::msg::PointCloud2 merged;
  merged.header = header;
  merged.fields = left.fields;
  merged.point_step = left.point_step;
  merged.height = 1;
  merged.width = static_cast<uint32_t>(pointCount(left) + pointCount(right));
  merged.row_step = merged.point_step * merged.width;
  merged.is_bigendian = left.is_bigendian;
  merged.is_dense = left.is_dense && right.is_dense;
  merged.data.resize(merged.row_step);

  const size_t left_count = pointCount(left);
  for (size_t i = 0; i < left_count; ++i) {
    copyPoint(left, i, merged, i);
  }
  for (size_t i = 0; i < pointCount(right); ++i) {
    copyPoint(right, i, merged, left_count + i);
  }

  return merged;
}

void readPointXyz(
  const sensor_msgs::msg::PointCloud2 & cloud,
  size_t index,
  float & x,
  float & y,
  float & z)
{
  const auto * src = cloud.data.data() + index * cloud.point_step;
  const int x_offset = fieldOffset(cloud, "x");
  const int y_offset = fieldOffset(cloud, "y");
  const int z_offset = fieldOffset(cloud, "z");
  std::memcpy(&x, src + x_offset, sizeof(float));
  std::memcpy(&y, src + y_offset, sizeof(float));
  std::memcpy(&z, src + z_offset, sizeof(float));
}

void writePointXyz(
  sensor_msgs::msg::PointCloud2 & cloud,
  size_t index,
  float x,
  float y,
  float z)
{
  auto * dst = cloud.data.data() + index * cloud.point_step;
  const int x_offset = fieldOffset(cloud, "x");
  const int y_offset = fieldOffset(cloud, "y");
  const int z_offset = fieldOffset(cloud, "z");
  std::memcpy(dst + x_offset, &x, sizeof(float));
  std::memcpy(dst + y_offset, &y, sizeof(float));
  std::memcpy(dst + z_offset, &z, sizeof(float));
}

void copyPoint(
  const sensor_msgs::msg::PointCloud2 & src,
  size_t src_index,
  sensor_msgs::msg::PointCloud2 & dst,
  size_t dst_index)
{
  const auto * src_ptr = src.data.data() + src_index * src.point_step;
  auto * dst_ptr = dst.data.data() + dst_index * dst.point_step;
  std::memcpy(dst_ptr, src_ptr, src.point_step);
}

}  // namespace stretch_core
