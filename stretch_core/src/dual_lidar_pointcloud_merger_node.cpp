#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <omp.h>

namespace
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

std::string fieldNames(const sensor_msgs::msg::PointCloud2 & cloud)
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

sensor_msgs::msg::PointCloud2 voxelDownsample(
  const sensor_msgs::msg::PointCloud2 & input,
  const PointFieldLayout & layout,
  const float leaf_size)
{
  sensor_msgs::msg::PointCloud2 output = input;
  if (leaf_size <= 0.0F || !layout.valid) {
    return output;
  }

  const size_t count = pointCount(input);
  const size_t point_step = layout.point_step;
  const float inv_leaf = 1.0F / leaf_size;

  struct VoxelKey
  {
    int32_t x{0};
    int32_t y{0};
    int32_t z{0};
  };

  struct VoxelKeyHash
  {
    size_t operator()(const VoxelKey & key) const noexcept
    {
      return (static_cast<size_t>(key.x) * 73856093U) ^
             (static_cast<size_t>(key.y) * 19349663U) ^
             (static_cast<size_t>(key.z) * 83492791U);
    }
  };

  struct VoxelKeyEqual
  {
    bool operator()(const VoxelKey & a, const VoxelKey & b) const noexcept
    {
      return a.x == b.x && a.y == b.y && a.z == b.z;
    }
  };

  std::unordered_map<VoxelKey, size_t, VoxelKeyHash, VoxelKeyEqual> occupied;
  occupied.reserve(count / 8U);

  std::vector<size_t> kept;
  kept.reserve(count / 8U);

  for (size_t i = 0; i < count; ++i) {
    const auto * src = input.data.data() + i * point_step;
    float x = 0.0F;
    float y = 0.0F;
    float z = 0.0F;
    std::memcpy(&x, src + layout.x_offset, sizeof(float));
    std::memcpy(&y, src + layout.y_offset, sizeof(float));
    std::memcpy(&z, src + layout.z_offset, sizeof(float));

    const VoxelKey key{
      static_cast<int32_t>(std::floor(x * inv_leaf)),
      static_cast<int32_t>(std::floor(y * inv_leaf)),
      static_cast<int32_t>(std::floor(z * inv_leaf)),
    };

    if (occupied.emplace(key, i).second) {
      kept.push_back(i);
    }
  }

  output.height = 1;
  output.width = static_cast<uint32_t>(kept.size());
  output.row_step = output.point_step * output.width;
  output.data.resize(output.row_step);

  for (size_t out_index = 0; out_index < kept.size(); ++out_index) {
    std::memcpy(
      output.data.data() + out_index * point_step,
      input.data.data() + kept[out_index] * point_step,
      point_step);
  }

  return output;
}

bool appendTransformedCloud(
  const sensor_msgs::msg::PointCloud2 & input,
  const PointFieldLayout & layout,
  const LinearTransform3f & transform,
  sensor_msgs::msg::PointCloud2 & output,
  const size_t output_point_offset,
  const double filter_radius)
{
  if (!layout.valid || layout.point_step != input.point_step) {
    return false;
  }

  const size_t count = pointCount(input);
  const size_t point_step = layout.point_step;
  const int x_offset = layout.x_offset;
  const int y_offset = layout.y_offset;
  const int z_offset = layout.z_offset;
  const bool xyz_contiguous = layout.xyzContiguous();
  const size_t xyz_end = static_cast<size_t>(z_offset) + sizeof(float);
  const float radius_sq = static_cast<float>(filter_radius * filter_radius);

  #pragma omp parallel for
  for (size_t i = 0; i < count; ++i) {
    const auto * src = input.data.data() + i * point_step;
    auto * dst = output.data.data() + (output_point_offset + i) * point_step;

    float x = 0.0F;
    float y = 0.0F;
    float z = 0.0F;
    std::memcpy(&x, src + x_offset, sizeof(float));
    std::memcpy(&y, src + y_offset, sizeof(float));
    std::memcpy(&z, src + z_offset, sizeof(float));

    float tx = 0.0F;
    float ty = 0.0F;
    float tz = 0.0F;
    transform.transform(x, y, z, tx, ty, tz);

    if (radius_sq > 0.0f && ((tx * tx) + (ty * ty) < radius_sq)) {
      float nan_val = std::numeric_limits<float>::quiet_NaN();
      tx = nan_val;
      ty = nan_val;
      tz = nan_val;
    }

    if (xyz_contiguous) {
      const size_t head_bytes = static_cast<size_t>(x_offset);
      if (head_bytes > 0) {
        std::memcpy(dst, src, head_bytes);
      }
      std::memcpy(dst + x_offset, &tx, sizeof(float));
      std::memcpy(dst + y_offset, &ty, sizeof(float));
      std::memcpy(dst + z_offset, &tz, sizeof(float));
      if (xyz_end < point_step) {
        std::memcpy(dst + xyz_end, src + xyz_end, point_step - xyz_end);
      }
    } else {
      std::memcpy(dst, src, point_step);
      std::memcpy(dst + x_offset, &tx, sizeof(float));
      std::memcpy(dst + y_offset, &ty, sizeof(float));
      std::memcpy(dst + z_offset, &tz, sizeof(float));
    }
  }

  return true;
}

sensor_msgs::msg::PointCloud2 makeMergedCloudSkeleton(
  const sensor_msgs::msg::PointCloud2 & left,
  const sensor_msgs::msg::PointCloud2 & right,
  const std_msgs::msg::Header & header,
  const double filter_radius)
{
  sensor_msgs::msg::PointCloud2 merged;
  merged.header = header;
  merged.fields = left.fields;
  merged.point_step = left.point_step;
  merged.height = 1;
  merged.width = pointCount(left) + pointCount(right);
  merged.row_step = merged.point_step * merged.width;
  merged.is_bigendian = left.is_bigendian;
  //If filtering is active, it's false (to support Nan). Otherwise, match the input clouds.
  merged.is_dense = (filter_radius > 0.0) ? false : (left.is_dense && right.is_dense);
  merged.data.resize(merged.row_step);
  return merged;
}

}  // namespace

class DualLidarPointcloudMergerNode : public rclcpp::Node
{
public:
  DualLidarPointcloudMergerNode()
  : Node("dual_lidar_pointcloud_merger"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    declare_parameter<std::string>("left_topic", "/lidar_points_left");
    declare_parameter<std::string>("right_topic", "/lidar_points_right");
    declare_parameter<std::string>("output_topic", "/lidar_points");
    declare_parameter<std::string>("target_frame", "base_link");
    declare_parameter<std::string>("ring_field", "ring");
    declare_parameter<std::string>("timestamp_field", "timestamp");
    declare_parameter<double>("sync_slop_sec", 0.05);
    declare_parameter<double>("merger_voxel_leaf_size", 0.0);
    declare_parameter<double>("cylinder_filter_radius", 0.3);

    const auto left_topic = get_parameter("left_topic").as_string();
    const auto right_topic = get_parameter("right_topic").as_string();
    const auto output_topic = get_parameter("output_topic").as_string();
    target_frame_ = get_parameter("target_frame").as_string();
    ring_field_ = get_parameter("ring_field").as_string();
    timestamp_field_ = get_parameter("timestamp_field").as_string();
    const double sync_slop = get_parameter("sync_slop_sec").as_double();
    merger_voxel_leaf_size_ = get_parameter("merger_voxel_leaf_size").as_double();
    cylinder_filter_radius_ = get_parameter("cylinder_filter_radius").as_double();

    rclcpp::SensorDataQoS qos;
    // qos.keep_last(1);

    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, qos);

    left_sub_.subscribe(this, left_topic, qos.get_rmw_qos_profile());
    right_sub_.subscribe(this, right_topic, qos.get_rmw_qos_profile());

    // Increased queue size slightly to handle 10Hz buffer safety
    sync_ = std::make_shared<Sync>(
      SyncPolicy(20), left_sub_, right_sub_);
    sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(sync_slop));
    sync_->registerCallback(
      std::bind(
        &DualLidarPointcloudMergerNode::cloudCallback, this,
        std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(
      get_logger(),
      "Merging %s + %s -> %s in %s (preserving '%s' and '%s' per-point fields)",
      left_topic.c_str(), right_topic.c_str(), output_topic.c_str(),
      target_frame_.c_str(), ring_field_.c_str(), timestamp_field_.c_str());
    if (merger_voxel_leaf_size_ > 0.0) {
      RCLCPP_INFO(
        get_logger(),
        "Voxel downsample enabled: merger_voxel_leaf_size=%.3f m in lidar frame (before transform)",
        merger_voxel_leaf_size_);
    }
  }

private:
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::PointCloud2,
    sensor_msgs::msg::PointCloud2>;
  using Sync = message_filters::Synchronizer<SyncPolicy>;

  bool hasRequiredFields(const sensor_msgs::msg::PointCloud2 & cloud) const
  {
    const bool has_ring = hasField(cloud, ring_field_);
    const bool has_timestamp = hasField(cloud, timestamp_field_);
    if (!has_ring || !has_timestamp) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Cloud from %s missing required fields. Fields: [%s]",
        cloud.header.frame_id.c_str(), fieldNames(cloud).c_str());
      return false;
    }
    return true;
  }

  bool cacheTransform(
    const std::string & source_frame,
    LinearTransform3f & linear_out,
    bool & is_ready)
  {
    if (is_ready) {
      return true;
    }

    try {
      // Use TimePointZero to grab the latest available static transform instantly without blocking
      const auto tf_stamped = tf_buffer_.lookupTransform(
        target_frame_, source_frame, tf2::TimePointZero);
      const Eigen::Affine3f affine =
        tf2::transformToEigen(tf_stamped.transform).cast<float>();
      linear_out = LinearTransform3f::fromAffine(affine);
      is_ready = true;
      RCLCPP_INFO(get_logger(), "Successfully cached static transform for %s", source_frame.c_str());
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for static transform from %s to %s...",
        source_frame.c_str(), target_frame_.c_str());
      return false;
    }
  }

  void cloudCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & left,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & right)
  {
    if (!hasRequiredFields(*left) || !hasRequiredFields(*right)) {
      return;
    }

    if (!field_layout_cached_) {
      field_layout_ = PointFieldLayout::fromCloud(*left);
      if (!field_layout_.valid) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Could not cache x/y/z field offsets from incoming point cloud");
        return;
      }
      field_layout_cached_ = true;
      RCLCPP_INFO(get_logger(), "Left cloud fields: [%s]", fieldNames(*left).c_str());
      RCLCPP_INFO(get_logger(), "Right cloud fields: [%s]", fieldNames(*right).c_str());
      RCLCPP_INFO(
        get_logger(),
        "Cached point layout: point_step=%u x=%d y=%d z=%d contiguous_xyz=%s",
        field_layout_.point_step,
        field_layout_.x_offset,
        field_layout_.y_offset,
        field_layout_.z_offset,
        field_layout_.xyzContiguous() ? "true" : "false");
    }

    if (left->point_step != right->point_step || !fieldsMatch(left->fields, right->fields)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Left/right point clouds have mismatched fields or point_step; skipping merge");
      return;
    }

    // Try to get transforms from cache (or populate cache if first run)
    if (!cacheTransform(left->header.frame_id, left_tf_linear_, left_tf_ready_) ||
        !cacheTransform(right->header.frame_id, right_tf_linear_, right_tf_ready_))
    {
      return; // Skip this frame until static transforms are published and cached
    }

    const sensor_msgs::msg::PointCloud2 * left_in = left.get();
    const sensor_msgs::msg::PointCloud2 * right_in = right.get();
    sensor_msgs::msg::PointCloud2 left_voxel;
    sensor_msgs::msg::PointCloud2 right_voxel;
    if (merger_voxel_leaf_size_ > 0.0) {
      left_voxel = voxelDownsample(
        *left, field_layout_, static_cast<float>(merger_voxel_leaf_size_));
      right_voxel = voxelDownsample(
        *right, field_layout_, static_cast<float>(merger_voxel_leaf_size_));
      left_in = &left_voxel;
      right_in = &right_voxel;

      if (!logged_voxel_stats_) {
        logged_voxel_stats_ = true;
        RCLCPP_INFO(
          get_logger(),
          "Voxel downsample: left %zu -> %u, right %zu -> %u points (leaf=%.3f m)",
          pointCount(*left),
          left_voxel.width,
          pointCount(*right),
          right_voxel.width,
          merger_voxel_leaf_size_);
      }
    }

    std_msgs::msg::Header header;
    header.frame_id = target_frame_;
    header.stamp = olderStamp(left->header.stamp, right->header.stamp);

    auto merged = makeMergedCloudSkeleton(*left_in, *right_in, header, cylinder_filter_radius_);

    if (!appendTransformedCloud(*left_in, field_layout_, left_tf_linear_, merged, 0, cylinder_filter_radius_) ||
        !appendTransformedCloud(
        *right_in, field_layout_, right_tf_linear_, merged, pointCount(*left_in), cylinder_filter_radius_))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Could not transform merged point cloud because x/y/z fields are missing");
      return;
    }

    if (!hasRequiredFields(merged)) {
      return;
    }

    pub_->publish(merged);
  }

  bool field_layout_cached_{false};
  bool logged_voxel_stats_{false};
  double merger_voxel_leaf_size_{0.0};
  PointFieldLayout field_layout_;
  std::string target_frame_;
  std::string ring_field_;
  std::string timestamp_field_;
  double cylinder_filter_radius_{0.0};

  bool left_tf_ready_{false};
  bool right_tf_ready_{false};
  LinearTransform3f left_tf_linear_;
  LinearTransform3f right_tf_linear_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> left_sub_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> right_sub_;
  std::shared_ptr<Sync> sync_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DualLidarPointcloudMergerNode>());
  rclcpp::shutdown();
  return 0;
}