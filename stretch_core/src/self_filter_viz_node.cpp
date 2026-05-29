#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include "stretch_core/robot_self_filter.hpp"
#include "stretch_core/robot_self_filter_params.hpp"

class SelfFilterVizNode : public rclcpp::Node
{
public:
  SelfFilterVizNode()
  : Node("self_filter_viz_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    this->declare_parameter<std::string>("frame_id", "base_footprint");
    stretch_core::declareRobotSelfFilterParameters(*this);
    this->declare_parameter("pub_self_filter_markers", true);
    this->declare_parameter<std::string>("self_filter_markers_topic", "/self_filter_markers");
    this->declare_parameter("update_rate_hz", 10.0);

    loadConfig();
    self_filter_.setConfig(config_);

    target_frame_ = this->get_parameter("frame_id").as_string();
    pub_markers_ = this->get_parameter("pub_self_filter_markers").as_bool();
    markers_topic_ = this->get_parameter("self_filter_markers_topic").as_string();

    if (pub_markers_) {
      pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        markers_topic_, rclcpp::QoS(1).reliable());
    }

    const double update_rate_hz = this->get_parameter("update_rate_hz").as_double();
    const auto period = std::chrono::duration<double>(1.0 / std::max(update_rate_hz, 1.0));
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&SelfFilterVizNode::timerCallback, this));

    RCLCPP_INFO(
      this->get_logger(),
      "Self-filter viz node running in frame '%s', markers=%s",
      target_frame_.c_str(),
      pub_markers_ ? "on" : "off");
  }

private:
  void loadConfig()
  {
    config_ = stretch_core::loadRobotSelfFilterConfig(*this);
  }

  void updateSelfFilterTransforms(const tf2::TimePoint & tf_time, bool force_markers)
  {
    if (config_.filter_arm || force_markers) {
      self_filter_.updateArmSegment(
        tf_buffer_, target_frame_, tf_time, this->get_logger(), *this->get_clock(),
        force_markers);
    }
    if (config_.filter_arm_shoulder || force_markers) {
      self_filter_.updateArmShoulderBox(
        tf_buffer_, target_frame_, tf_time, this->get_logger(), *this->get_clock(),
        force_markers);
    }
    if (config_.filter_wrist || force_markers) {
      self_filter_.updateWristChain(
        tf_buffer_, target_frame_, tf_time, this->get_logger(), *this->get_clock(),
        force_markers);
    }
    if (config_.filter_attachment || force_markers) {
      self_filter_.updateAttachmentBox(
        tf_buffer_, target_frame_, tf_time, this->get_logger(), *this->get_clock(),
        force_markers);
    }
  }

  void timerCallback()
  {
    const tf2::TimePoint tf_time = tf2::TimePointZero;
    const bool force_markers = pub_markers_;
    updateSelfFilterTransforms(tf_time, force_markers);

    if (!pub_markers_ || !pub_) {
      return;
    }

    visualization_msgs::msg::MarkerArray marker_array;
    self_filter_.appendSelfFilterMarkers(marker_array, target_frame_, this->now(), true);
    pub_->publish(marker_array);
  }

  stretch_core::RobotSelfFilter self_filter_;
  stretch_core::RobotSelfFilterConfig config_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string target_frame_;
  std::string markers_topic_;
  bool pub_markers_{true};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SelfFilterVizNode>());
  rclcpp::shutdown();
  return 0;
}
