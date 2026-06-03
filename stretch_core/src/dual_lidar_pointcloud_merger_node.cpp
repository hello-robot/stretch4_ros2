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
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
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

bool appendTransformedCloud(
  const sensor_msgs::msg::PointCloud2 & input,
  const Eigen::Affine3f & transform,
  sensor_msgs::msg::PointCloud2 & output,
  const size_t output_point_offset)
{
  const int x_offset = fieldOffset(input, "x");
  const int y_offset = fieldOffset(input, "y");
  const int z_offset = fieldOffset(input, "z");
  if (x_offset < 0 || y_offset < 0 || z_offset < 0) {
    return false;
  }

  const size_t count = pointCount(input);

  // Multi-thread the point transformations
  #pragma omp parallel for
  for (size_t i = 0; i < count; ++i) {
    const auto * src = input.data.data() + i * input.point_step;
    auto * dst = output.data.data() + (output_point_offset + i) * output.point_step;
    
    // Copy all fields (intensity, ring, timestamp, etc.)
    std::memcpy(dst, src, input.point_step);

    // Extract XYZ
    float x = 0.0F;
    float y = 0.0F;
    float z = 0.0F;
    std::memcpy(&x, src + x_offset, sizeof(float));
    std::memcpy(&y, src + y_offset, sizeof(float));
    std::memcpy(&z, src + z_offset, sizeof(float));

    // Apply transform
    const Eigen::Vector3f transformed = transform * Eigen::Vector3f(x, y, z);
    const float tx = transformed.x();
    const float ty = transformed.y();
    const float tz = transformed.z();
    
    // Overwrite XYZ with transformed coordinates
    std::memcpy(dst + x_offset, &tx, sizeof(float));
    std::memcpy(dst + y_offset, &ty, sizeof(float));
    std::memcpy(dst + z_offset, &tz, sizeof(float));
  }

  return true;
}

sensor_msgs::msg::PointCloud2 makeMergedCloudSkeleton(
  const sensor_msgs::msg::PointCloud2 & left,
  const sensor_msgs::msg::PointCloud2 & right,
  const std_msgs::msg::Header & header)
{
  sensor_msgs::msg::PointCloud2 merged;
  merged.header = header;
  merged.fields = left.fields;
  merged.point_step = left.point_step;
  merged.height = 1;
  merged.width = pointCount(left) + pointCount(right);
  merged.row_step = merged.point_step * merged.width;
  merged.is_bigendian = left.is_bigendian;
  merged.is_dense = left.is_dense && right.is_dense;
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

    const auto left_topic = get_parameter("left_topic").as_string();
    const auto right_topic = get_parameter("right_topic").as_string();
    const auto output_topic = get_parameter("output_topic").as_string();
    target_frame_ = get_parameter("target_frame").as_string();
    ring_field_ = get_parameter("ring_field").as_string();
    timestamp_field_ = get_parameter("timestamp_field").as_string();
    const double sync_slop = get_parameter("sync_slop_sec").as_double();

    rclcpp::SensorDataQoS qos;

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

  bool getCachedTransform(const std::string& source_frame, Eigen::Affine3f& transform, bool& is_ready)
  {
    if (is_ready) {
      return true;
    }

    try {
      // Use TimePointZero to grab the latest available static transform instantly without blocking
      const auto tf_stamped = tf_buffer_.lookupTransform(
        target_frame_, source_frame, tf2::TimePointZero);
      transform = tf2::transformToEigen(tf_stamped.transform).cast<float>();
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
    if (!logged_fields_) {
      logged_fields_ = true;
      RCLCPP_INFO(get_logger(), "Left cloud fields: [%s]", fieldNames(*left).c_str());
      RCLCPP_INFO(get_logger(), "Right cloud fields: [%s]", fieldNames(*right).c_str());
    }

    if (!hasRequiredFields(*left) || !hasRequiredFields(*right)) {
      return;
    }

    if (left->point_step != right->point_step || !fieldsMatch(left->fields, right->fields)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Left/right point clouds have mismatched fields or point_step; skipping merge");
      return;
    }

    // Try to get transforms from cache (or populate cache if first run)
    if (!getCachedTransform(left->header.frame_id, left_transform_, left_tf_ready_) ||
        !getCachedTransform(right->header.frame_id, right_transform_, right_tf_ready_)) 
    {
      return; // Skip this frame until static transforms are published and cached
    }

    std_msgs::msg::Header header;
    header.frame_id = target_frame_;
    header.stamp = olderStamp(left->header.stamp, right->header.stamp);

    auto merged = makeMergedCloudSkeleton(*left, *right, header);
    
    if (!appendTransformedCloud(*left, left_transform_, merged, 0) ||
        !appendTransformedCloud(*right, right_transform_, merged, pointCount(*left)))
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

  bool logged_fields_{false};
  std::string target_frame_;
  std::string ring_field_;
  std::string timestamp_field_;

  // TF Caching variables
  bool left_tf_ready_{false};
  bool right_tf_ready_{false};
  Eigen::Affine3f left_transform_;
  Eigen::Affine3f right_transform_;

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