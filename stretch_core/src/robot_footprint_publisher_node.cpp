#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

#include "stretch_core/robot_self_filter.hpp"
#include "stretch_core/robot_self_filter_params.hpp"

namespace
{

std::vector<Eigen::Vector2f> parseBasePolygon(const std::vector<double> & flat)
{
  std::vector<Eigen::Vector2f> polygon;
  if (flat.size() % 2 != 0) {
    return polygon;
  }
  polygon.reserve(flat.size() / 2);
  for (size_t i = 0; i + 1 < flat.size(); i += 2) {
    polygon.emplace_back(static_cast<float>(flat[i]), static_cast<float>(flat[i + 1]));
  }
  return polygon;
}

bool isArmRelatedJoint(const std::string & name)
{
  if (name == "lift_joint") {
    return true;
  }
  if (name.rfind("arm_l", 0) == 0 && name.size() > 5 && name.find("_joint") != std::string::npos) {
    return true;
  }
  if (name.find("wrist") != std::string::npos) {
    return true;
  }
  if (name.find("gripper") != std::string::npos) {
    return true;
  }
  return false;
}

bool isPrismaticJoint(const std::string & name)
{
  if (name == "lift_joint") {
    return true;
  }
  if (name.rfind("arm_l", 0) == 0 && name.find("_joint") != std::string::npos) {
    return true;
  }
  return false;
}

double jointChangeThreshold(
  const std::string & name,
  double threshold_m,
  double threshold_rad)
{
  return isPrismaticJoint(name) ? threshold_m : threshold_rad;
}

float maxVertexDisplacement(
  const std::vector<Eigen::Vector2f> & a,
  const std::vector<Eigen::Vector2f> & b)
{
  if (a.empty() || b.empty()) {
    return std::numeric_limits<float>::infinity();
  }
  if (a.size() != b.size()) {
    return std::numeric_limits<float>::infinity();
  }

  float max_disp = 0.0f;
  for (size_t i = 0; i < a.size(); ++i) {
    max_disp = std::max(max_disp, (a[i] - b[i]).norm());
  }
  return max_disp;
}

geometry_msgs::msg::PolygonStamped toPolygonMsg(
  const std::vector<Eigen::Vector2f> & hull,
  const std::string & frame_id,
  const rclcpp::Time & stamp)
{
  geometry_msgs::msg::PolygonStamped msg;
  msg.header.frame_id = frame_id;
  msg.header.stamp = stamp;
  msg.polygon.points.reserve(hull.size());
  for (const auto & p : hull) {
    geometry_msgs::msg::Point32 pt;
    pt.x = p.x();
    pt.y = p.y();
    pt.z = 0.0f;
    msg.polygon.points.push_back(pt);
  }
  return msg;
}

}  // namespace

class RobotFootprintPublisherNode : public rclcpp::Node
{
public:
  RobotFootprintPublisherNode()
  : Node("robot_footprint_publisher"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    declareFootprintParameters();
    stretch_core::declareRobotSelfFilterParameters(*this);
    loadParameters();
    if (base_polygon_.size() < 3) {
      RCLCPP_ERROR(get_logger(), "base_footprint_polygon must have at least 3 vertices (6 values).");
    }

    footprint_pub_ = create_publisher<geometry_msgs::msg::PolygonStamped>(
      footprint_topic_, rclcpp::QoS(1).transient_local());
    if (publish_costmap_topics_) {
      local_footprint_pub_ = create_publisher<geometry_msgs::msg::PolygonStamped>(
        local_costmap_footprint_topic_, rclcpp::QoS(1).transient_local());
      global_footprint_pub_ = create_publisher<geometry_msgs::msg::PolygonStamped>(
        global_costmap_footprint_topic_, rclcpp::QoS(1).transient_local());
    }

    joint_states_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_states_topic_, rclcpp::QoS(10),
      std::bind(&RobotFootprintPublisherNode::jointStatesCallback, this, std::placeholders::_1));

    startup_timer_ = create_wall_timer(
      std::chrono::milliseconds(200),
      std::bind(&RobotFootprintPublisherNode::startupTimerCallback, this));
  }

private:
  void declareFootprintParameters()
  {
    declare_parameter<std::string>("frame_id", "base_footprint");
    declare_parameter<std::string>("footprint_topic", "/footprint");
    declare_parameter("publish_costmap_footprint_topics", true);
    declare_parameter<std::string>(
      "local_costmap_footprint_topic", "/local_costmap/local_costmap/footprint");
    declare_parameter<std::string>(
      "global_costmap_footprint_topic", "/global_costmap/global_costmap/footprint");
    declare_parameter<std::string>("joint_states_topic", "/joint_states");
    declare_parameter("joint_change_threshold_m", 0.01);
    declare_parameter("joint_change_threshold_rad", 0.01);
    declare_parameter("footprint_change_epsilon_m", 0.01);
    declare_parameter("base_footprint_polygon", std::vector<double>{});
  }

  void loadParameters()
  {
    frame_id_ = get_parameter("frame_id").as_string();
    footprint_topic_ = get_parameter("footprint_topic").as_string();
    publish_costmap_topics_ = get_parameter("publish_costmap_footprint_topics").as_bool();
    local_costmap_footprint_topic_ = get_parameter("local_costmap_footprint_topic").as_string();
    global_costmap_footprint_topic_ = get_parameter("global_costmap_footprint_topic").as_string();
    joint_states_topic_ = get_parameter("joint_states_topic").as_string();
    joint_change_threshold_m_ = get_parameter("joint_change_threshold_m").as_double();
    joint_change_threshold_rad_ = get_parameter("joint_change_threshold_rad").as_double();
    footprint_change_epsilon_m_ = get_parameter("footprint_change_epsilon_m").as_double();
    base_polygon_ = parseBasePolygon(get_parameter("base_footprint_polygon").as_double_array());

    self_filter_config_ = stretch_core::loadRobotSelfFilterConfig(*this);
    self_filter_.setConfig(self_filter_config_);
  }

  bool jointsMovedEnough(const sensor_msgs::msg::JointState & msg)
  {
    if (last_joint_positions_.empty()) {
      return true;
    }

    for (size_t i = 0; i < msg.name.size(); ++i) {
      if (!isArmRelatedJoint(msg.name[i])) {
        continue;
      }
      const auto it = last_joint_positions_.find(msg.name[i]);
      if (it == last_joint_positions_.end()) {
        return true;
      }
      if (i >= msg.position.size()) {
        continue;
      }
      const double threshold = jointChangeThreshold(
        msg.name[i], joint_change_threshold_m_, joint_change_threshold_rad_);
      if (std::abs(msg.position[i] - it->second) > threshold) {
        return true;
      }
    }
    return false;
  }

  void storeJointPositions(const sensor_msgs::msg::JointState & msg)
  {
    for (size_t i = 0; i < msg.name.size(); ++i) {
      if (!isArmRelatedJoint(msg.name[i]) || i >= msg.position.size()) {
        continue;
      }
      last_joint_positions_[msg.name[i]] = msg.position[i];
    }
  }

  bool updateFootprintIfChanged(bool force_publish)
  {
    if (base_polygon_.size() < 3) {
      return false;
    }

    const auto time = tf2::TimePointZero;
    self_filter_.updateArmSegment(tf_buffer_, frame_id_, time, get_logger(), *get_clock(), true);
    self_filter_.updateWristChain(tf_buffer_, frame_id_, time, get_logger(), *get_clock(), true);
    if (self_filter_config_.filter_attachment) {
      self_filter_.updateAttachmentBox(
        tf_buffer_, frame_id_, time, get_logger(), *get_clock(), true);
    }

    const auto hull = self_filter_.computeFootprintPolygon2d(base_polygon_);
    if (hull.size() < 3) {
      return false;
    }

    const float displacement = maxVertexDisplacement(hull, last_published_hull_);
    if (!force_publish && displacement <= static_cast<float>(footprint_change_epsilon_m_)) {
      return false;
    }

    const auto stamp = now();
    const auto msg = toPolygonMsg(hull, frame_id_, stamp);
    footprint_pub_->publish(msg);
    if (local_footprint_pub_) {
      local_footprint_pub_->publish(msg);
    }
    if (global_footprint_pub_) {
      global_footprint_pub_->publish(msg);
    }

    last_published_hull_ = hull;
    has_published_ = true;
    return true;
  }

  void jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (!jointsMovedEnough(*msg)) {
      return;
    }
    storeJointPositions(*msg);
    updateFootprintIfChanged(false);
  }

  void startupTimerCallback()
  {
    if (has_published_) {
      startup_timer_->cancel();
      return;
    }
    if (updateFootprintIfChanged(true)) {
      startup_timer_->cancel();
      RCLCPP_INFO(get_logger(), "Published initial footprint on %s", footprint_topic_.c_str());
    }
  }

  std::string frame_id_;
  std::string footprint_topic_;
  std::string local_costmap_footprint_topic_;
  std::string global_costmap_footprint_topic_;
  std::string joint_states_topic_;
  double joint_change_threshold_m_{0.01};
  double joint_change_threshold_rad_{0.01};
  double footprint_change_epsilon_m_{0.01};
  bool publish_costmap_topics_{true};
  bool has_published_{false};

  std::vector<Eigen::Vector2f> base_polygon_;
  std::vector<Eigen::Vector2f> last_published_hull_;
  std::unordered_map<std::string, double> last_joint_positions_;

  stretch_core::RobotSelfFilterConfig self_filter_config_;
  stretch_core::RobotSelfFilter self_filter_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr footprint_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr local_footprint_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr global_footprint_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  rclcpp::TimerBase::SharedPtr startup_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotFootprintPublisherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
