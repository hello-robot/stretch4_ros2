#pragma once

// Raw sensor_msgs::PointCloud2 byte-layout helpers.
//
// The dual-lidar path deliberately does NOT go through pcl::fromROSMsg. Converting to
// pcl::PointXYZ drops per-point `ring` and `timestamp` fields needed to deskew.

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <builtin_interfaces/msg/time.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace stretch_core
{

inline int fieldOffset(const sensor_msgs::msg::PointCloud2 & cloud, const std::string & name)
{
  const auto it = std::find_if(
    cloud.fields.begin(), cloud.fields.end(),
    [&name](const sensor_msgs::msg::PointField & field) {
      return field.name == name;
    });
  return (it != cloud.fields.end()) ? static_cast<int>(it->offset) : -1;
}

inline bool hasField(const sensor_msgs::msg::PointCloud2 & cloud, const std::string & name)
{
  return fieldOffset(cloud, name) >= 0;
}

inline std::string fieldNames(const sensor_msgs::msg::PointCloud2 & cloud)
{
  std::string names;
  for (size_t i = 0; i < cloud.fields.size(); ++i) {
    if (i > 0) {
      names += ", ";
    }
    names += cloud.fields[i].name;
  }
  return names;
}

inline bool fieldsMatch(
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

inline size_t pointCount(const sensor_msgs::msg::PointCloud2 & cloud)
{
  return static_cast<size_t>(cloud.width) * static_cast<size_t>(cloud.height);
}

// The merged cloud is stamped with the OLDER of the two input stamps: it is the earliest
// instant any point in it was observed, so downstream deskewing never extrapolates forward.
inline builtin_interfaces::msg::Time olderStamp(
  const builtin_interfaces::msg::Time & a,
  const builtin_interfaces::msg::Time & b)
{
  if (a.sec != b.sec) {
    return (a.sec < b.sec) ? a : b;
  }
  return (a.nanosec < b.nanosec) ? a : b;
}

struct PointFieldLayout
{
  int x_offset{-1};
  int y_offset{-1};
  int z_offset{-1};
  uint32_t point_step{0};
  bool valid{false};

  bool xyzContiguous() const
  {
    return valid &&
           y_offset == x_offset + static_cast<int>(sizeof(float)) &&
           z_offset == y_offset + static_cast<int>(sizeof(float));
  }

  static PointFieldLayout fromCloud(const sensor_msgs::msg::PointCloud2 & cloud)
  {
    PointFieldLayout layout;
    layout.x_offset = fieldOffset(cloud, "x");
    layout.y_offset = fieldOffset(cloud, "y");
    layout.z_offset = fieldOffset(cloud, "z");
    layout.point_step = cloud.point_step;
    layout.valid = layout.x_offset >= 0 && layout.y_offset >= 0 && layout.z_offset >= 0;
    return layout;
  }
};

// A 3x4 rigid transform flattened into scalars. Eigen's expression templates are fine, but
// in the innermost per-point loop this form keeps everything in registers.
struct LinearTransform3f
{
  float r00{1.0F};
  float r01{0.0F};
  float r02{0.0F};
  float r10{0.0F};
  float r11{1.0F};
  float r12{0.0F};
  float r20{0.0F};
  float r21{0.0F};
  float r22{1.0F};
  float tx{0.0F};
  float ty{0.0F};
  float tz{0.0F};

  static LinearTransform3f fromAffine(const Eigen::Affine3f & transform)
  {
    const auto & matrix = transform.matrix();
    LinearTransform3f linear;
    linear.r00 = matrix(0, 0);
    linear.r01 = matrix(0, 1);
    linear.r02 = matrix(0, 2);
    linear.r10 = matrix(1, 0);
    linear.r11 = matrix(1, 1);
    linear.r12 = matrix(1, 2);
    linear.r20 = matrix(2, 0);
    linear.r21 = matrix(2, 1);
    linear.r22 = matrix(2, 2);
    linear.tx = matrix(0, 3);
    linear.ty = matrix(1, 3);
    linear.tz = matrix(2, 3);
    return linear;
  }

  void transform(float x, float y, float z, float & out_x, float & out_y, float & out_z) const
  {
    out_x = r00 * x + r01 * y + r02 * z + tx;
    out_y = r10 * x + r11 * y + r12 * z + ty;
    out_z = r20 * x + r21 * y + r22 * z + tz;
  }
};

}  // namespace stretch_core
