#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <algorithm>
#include <cstring>
#include <memory>
#include <string>

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

sensor_msgs::msg::PointCloud2 mergeClouds(
  const sensor_msgs::msg::PointCloud2 & left,
  const sensor_msgs::msg::PointCloud2 & right,
  const std_msgs::msg::Header & header)
{
  sensor_msgs::msg::PointCloud2 merged;
  merged.header = header;
  merged.fields = left.fields;
  merged.point_step = left.point_step;
  merged.height = 1;
  merged.width = left.width + right.width;
  merged.row_step = merged.point_step * merged.width;
  merged.is_bigendian = left.is_bigendian;
  merged.is_dense = left.is_dense && right.is_dense;
  merged.data.resize(merged.row_step);
  if (!left.data.empty()) {
    std::memcpy(merged.data.data(), left.data.data(), left.data.size());
  }
  if (!right.data.empty()) {
    std::memcpy(
      merged.data.data() + left.data.size(), right.data.data(), right.data.size());
  }
  return merged;
}

builtin_interfaces::msg::Time newerStamp(
  const builtin_interfaces::msg::Time & a,
  const builtin_interfaces::msg::Time & b)
{
  if (a.sec != b.sec) {
    return (a.sec > b.sec) ? a : b;
  }
  return (a.nanosec > b.nanosec) ? a : b;
}

bool transformCloudXyzOnly(
  const sensor_msgs::msg::PointCloud2 & input,
  const Eigen::Affine3f & transform,
  const std::string & target_frame,
  sensor_msgs::msg::PointCloud2 & output)
{
  output = input;
  output.header.frame_id = target_frame;

  sensor_msgs::PointCloud2ConstIterator<float> x_in(input, "x");
  sensor_msgs::PointCloud2ConstIterator<float> y_in(input, "y");
  sensor_msgs::PointCloud2ConstIterator<float> z_in(input, "z");

  sensor_msgs::PointCloud2Iterator<float> x_out(output, "x");
  sensor_msgs::PointCloud2Iterator<float> y_out(output, "y");
  sensor_msgs::PointCloud2Iterator<float> z_out(output, "z");

  for (; x_in != x_in.end(); ++x_in, ++y_in, ++z_in, ++x_out, ++y_out, ++z_out) {
    const Eigen::Vector3f transformed =
      transform * Eigen::Vector3f(*x_in, *y_in, *z_in);
    *x_out = transformed.x();
    *y_out = transformed.y();
    *z_out = transformed.z();
  }
  return true;
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

    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.reliable();

    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, qos);

    left_sub_.subscribe(this, left_topic, qos.get_rmw_qos_profile());
    right_sub_.subscribe(this, right_topic, qos.get_rmw_qos_profile());

    sync_ = std::make_shared<Sync>(
      SyncPolicy(10), left_sub_, right_sub_);
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
        "Cloud from %s missing required fields (has %s=%d, %s=%d). Fields: [%s]",
        cloud.header.frame_id.c_str(),
        ring_field_.c_str(), has_ring,
        timestamp_field_.c_str(), has_timestamp,
        fieldNames(cloud).c_str());
      return false;
    }
    return true;
  }

  bool transformCloud(
    const sensor_msgs::msg::PointCloud2 & input,
    sensor_msgs::msg::PointCloud2 & output)
  {
    try {
      const auto tf_stamped = tf_buffer_.lookupTransform(
        target_frame_, input.header.frame_id, input.header.stamp,
        rclcpp::Duration::from_seconds(0.05));
      const Eigen::Affine3f transform =
        tf2::transformToEigen(tf_stamped.transform).cast<float>();
      return transformCloudXyzOnly(input, transform, target_frame_, output);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_DEBUG(
        get_logger(),
        "Could not transform cloud from %s to %s: %s",
        input.header.frame_id.c_str(), target_frame_.c_str(), ex.what());
      return false;
    }
  }

  void cloudCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & left,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & right)
  {
    if (!logged_fields_) {
      logged_fields_ = true;
      RCLCPP_INFO(
        get_logger(), "Left cloud fields: [%s]", fieldNames(*left).c_str());
      RCLCPP_INFO(
        get_logger(), "Right cloud fields: [%s]", fieldNames(*right).c_str());
    }

    if (!hasRequiredFields(*left) || !hasRequiredFields(*right)) {
      return;
    }

    if (left->point_step != right->point_step ||
      !fieldsMatch(left->fields, right->fields))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Left/right point clouds have mismatched fields or point_step; skipping merge");
      return;
    }

    sensor_msgs::msg::PointCloud2 left_tf;
    sensor_msgs::msg::PointCloud2 right_tf;
    if (!transformCloud(*left, left_tf) || !transformCloud(*right, right_tf)) {
      return;
    }

    std_msgs::msg::Header header;
    header.frame_id = target_frame_;
    header.stamp = newerStamp(left_tf.header.stamp, right_tf.header.stamp);

    const auto merged = mergeClouds(left_tf, right_tf, header);
    if (!hasRequiredFields(merged)) {
      return;
    }

    pub_->publish(merged);
  }

  bool logged_fields_{false};
  std::string target_frame_;
  std::string ring_field_;
  std::string timestamp_field_;
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
