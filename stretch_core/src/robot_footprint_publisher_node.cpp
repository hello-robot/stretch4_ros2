#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <chrono>
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

geometry_msgs::msg::Polygon toPolygonMsg(
  const std::vector<Eigen::Vector2f> & hull)
{
  geometry_msgs::msg::Polygon msg;
  msg.points.reserve(hull.size());
  for (const auto & p : hull) {
    geometry_msgs::msg::Point32 pt;
    pt.x = p.x();
    pt.y = p.y();
    pt.z = 0.0f;
    msg.points.push_back(pt);
  }
  return msg;
}

geometry_msgs::msg::PolygonStamped toPolygonStampedMsg(
  const std::vector<Eigen::Vector2f> & hull,
  const std::string & frame_id,
  const builtin_interfaces::msg::Time & stamp)
{
  geometry_msgs::msg::PolygonStamped msg;
  msg.header.frame_id = frame_id;
  msg.header.stamp = stamp;
  msg.polygon = toPolygonMsg(hull);
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

    rclcpp::QoS costmap_footprint_qos(rclcpp::KeepLast(1));
    costmap_footprint_qos.transient_local();
    costmap_footprint_qos.reliable();

    rclcpp::QoS qos_profile(1);
    qos_profile.reliable();
    qos_profile.durability_volatile();

    footprint_pub_ = create_publisher<geometry_msgs::msg::Polygon>(
        footprint_topic_, qos_profile);
    if (publish_footprint_stamped_) {
      footprint_stamped_pub_ = create_publisher<geometry_msgs::msg::PolygonStamped>(
          footprint_stamped_topic_, costmap_footprint_qos);
      RCLCPP_INFO(
        get_logger(),
        "Publishing PolygonStamped footprint on %s",
        footprint_stamped_topic_.c_str());
        
      stamped_subscriber_watch_timer_ = create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&RobotFootprintPublisherNode::republishStampedOnNewSubscriber, this));
    }

    if (publish_costmap_topics_) {
        local_footprint_pub_ = create_publisher<geometry_msgs::msg::Polygon>(
            local_costmap_footprint_topic_, qos_profile);
        global_footprint_pub_ = create_publisher<geometry_msgs::msg::Polygon>(
            global_costmap_footprint_topic_, qos_profile);

        local_published_footprint_sub_ =
          create_subscription<geometry_msgs::msg::PolygonStamped>(
            local_costmap_published_footprint_topic_, costmap_footprint_qos,
            std::bind(
              &RobotFootprintPublisherNode::localPublishedFootprintCallback, this,
              std::placeholders::_1));
        global_published_footprint_sub_ =
          create_subscription<geometry_msgs::msg::PolygonStamped>(
            global_costmap_published_footprint_topic_, costmap_footprint_qos,
            std::bind(
              &RobotFootprintPublisherNode::globalPublishedFootprintCallback, this,
              std::placeholders::_1));
    }

    joint_states_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_states_topic_, rclcpp::QoS(10),
      std::bind(&RobotFootprintPublisherNode::jointStatesCallback, this, std::placeholders::_1));

    joystick_control_service_ = create_service<std_srvs::srv::SetBool>(
      "joystick_control",
      std::bind(
        &RobotFootprintPublisherNode::joystickControlCallback, this,
        std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(
      get_logger(),
      "Joystick control footprint service ready on ~/joystick_control "
      "(data=true: base-only; data=false: joint-state / arm footprint).");

    // Optional startup mode: base-only footprint (no arm)
    if (joystick_control_) {
      publishBaseFootprintOnly();
      RCLCPP_INFO(
        get_logger(),
        "Started with joystick_control=true; publishing base footprint only.");
    }
  }

private:
  void declareFootprintParameters()
  {
    declare_parameter<std::string>("frame_id", "base_footprint");
    declare_parameter<std::string>("footprint_topic", "/footprint");
    declare_parameter<std::string>("footprint_stamped_topic", "/footprint_stamped");
    declare_parameter("publish_footprint_stamped", false);
    declare_parameter("publish_costmap_footprint_topics", true);
    declare_parameter<std::string>(
      "local_costmap_footprint_topic", "/local_costmap/local_costmap/footprint");
    declare_parameter<std::string>(
      "global_costmap_footprint_topic", "/global_costmap/global_costmap/footprint");
    declare_parameter<std::string>(
      "local_costmap_published_footprint_topic", "/local_costmap/published_footprint");
    declare_parameter<std::string>(
      "global_costmap_published_footprint_topic", "/global_costmap/published_footprint");
    declare_parameter<std::string>("joint_states_topic", "/joint_states");
    declare_parameter("joint_change_threshold_m", 0.01);
    declare_parameter("joint_change_threshold_rad", 0.01);
    declare_parameter("footprint_change_epsilon_m", 0.01);
    declare_parameter("base_footprint_polygon", std::vector<double>{});
    declare_parameter("base_only_footprint_polygon", std::vector<double>{});
    // When true at startup (or via ~/joystick_control), publish base-only polygon (no arm).
    declare_parameter("joystick_control", false);
  }

  void loadParameters()
  {
    frame_id_ = get_parameter("frame_id").as_string();
    footprint_topic_ = get_parameter("footprint_topic").as_string();
    footprint_stamped_topic_ = get_parameter("footprint_stamped_topic").as_string();
    publish_footprint_stamped_ = get_parameter("publish_footprint_stamped").as_bool();
    publish_costmap_topics_ = get_parameter("publish_costmap_footprint_topics").as_bool();
    local_costmap_footprint_topic_ = get_parameter("local_costmap_footprint_topic").as_string();
    global_costmap_footprint_topic_ = get_parameter("global_costmap_footprint_topic").as_string();
    local_costmap_published_footprint_topic_ =
      get_parameter("local_costmap_published_footprint_topic").as_string();
    global_costmap_published_footprint_topic_ =
      get_parameter("global_costmap_published_footprint_topic").as_string();
    joint_states_topic_ = get_parameter("joint_states_topic").as_string();
    joint_change_threshold_m_ = get_parameter("joint_change_threshold_m").as_double();
    joint_change_threshold_rad_ = get_parameter("joint_change_threshold_rad").as_double();
    footprint_change_epsilon_m_ = get_parameter("footprint_change_epsilon_m").as_double();
    base_polygon_ = parseBasePolygon(get_parameter("base_footprint_polygon").as_double_array());
    base_only_polygon_ = parseBasePolygon(
      get_parameter("base_only_footprint_polygon").as_double_array());
    // Fall back to the normal base polygon if base-only is unset.
    if (base_only_polygon_.size() < 3) {
      base_only_polygon_ = base_polygon_;
    }
    joystick_control_ = get_parameter("joystick_control").as_bool();

    self_filter_config_ = stretch_core::loadRobotSelfFilterConfig(*this);
    self_filter_.setConfig(self_filter_config_);
  }

  void publishFootprint(const std::vector<Eigen::Vector2f> & hull)
  {
    const auto msg = toPolygonMsg(hull);
    footprint_pub_->publish(msg);
    if (footprint_stamped_pub_) {
      footprint_stamped_pub_->publish(
        toPolygonStampedMsg(hull, frame_id_, now()));
    }
    if (local_footprint_pub_) {
      local_footprint_pub_->publish(msg);
    }
    if (global_footprint_pub_) {
      global_footprint_pub_->publish(msg);
    }
  }

  void republishStampedOnNewSubscriber()
  {
    const std::size_t count = footprint_stamped_pub_->get_subscription_count();
    if (count > last_stamped_subscriber_count_ && !last_published_hull_.empty()) {
      footprint_stamped_pub_->publish(
        toPolygonStampedMsg(last_published_hull_, frame_id_, now()));
      RCLCPP_INFO(
        get_logger(),
        "New subscriber on %s; republished footprint.",
        footprint_stamped_topic_.c_str());
    }
    last_stamped_subscriber_count_ = count;
  }

  void publishLocalCostmapFootprintOnce()
  {
    if (published_local_costmap_footprint_ || !local_footprint_pub_ || last_published_hull_.empty()) {
      return;
    }

    local_footprint_pub_->publish(toPolygonMsg(last_published_hull_));
    published_local_costmap_footprint_ = true;
    local_published_footprint_sub_.reset();
    RCLCPP_INFO(
      get_logger(),
      "Local costmap is up; published footprint on %s",
      local_costmap_footprint_topic_.c_str());
  }

  void publishGlobalCostmapFootprintOnce()
  {
    if (published_global_costmap_footprint_ || !global_footprint_pub_ || last_published_hull_.empty()) {
      return;
    }

    global_footprint_pub_->publish(toPolygonMsg(last_published_hull_));
    published_global_costmap_footprint_ = true;
    global_published_footprint_sub_.reset();
    RCLCPP_INFO(
      get_logger(),
      "Global costmap is up; published footprint on %s",
      global_costmap_footprint_topic_.c_str());
  }

  void localPublishedFootprintCallback(
    const geometry_msgs::msg::PolygonStamped::SharedPtr /*msg*/)
  {
    local_costmap_ready_ = true;
    publishLocalCostmapFootprintOnce();
  }

  void globalPublishedFootprintCallback(
    const geometry_msgs::msg::PolygonStamped::SharedPtr /*msg*/)
  {
    global_costmap_ready_ = true;
    publishGlobalCostmapFootprintOnce();
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
    self_filter_.updateArmSegment(tf_buffer_, frame_id_, time, get_logger(), *get_clock());
    self_filter_.updateSelfFilterBoxes(tf_buffer_, frame_id_, time, get_logger(), *get_clock());
    const auto hull = self_filter_.computeFootprintPolygon2d(base_polygon_);
    if (hull.size() < 3) {
      return false;
    }

    const float displacement = maxVertexDisplacement(hull, last_published_hull_);
    if (!force_publish && displacement <= static_cast<float>(footprint_change_epsilon_m_)) {
      return false;
    }

    last_published_hull_ = hull;
    publishFootprint(hull);

    return true;
  }

  void publishBaseFootprintOnly()
  {
    if (base_only_polygon_.size() < 3) {
      RCLCPP_ERROR(get_logger(), "Cannot publish base footprint; polygon has fewer than 3 vertices.");
      return;
    }
    last_published_hull_ = base_only_polygon_;
    publishFootprint(base_only_polygon_);
  }

  void joystickControlCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    joystick_control_ = request->data;
    if (joystick_control_) {
      // Joystick teleop: publish the fixed base polygon only (no arm).
      publishBaseFootprintOnly();
      response->success = true;
      response->message = "Joystick control enabled; publishing base footprint only.";
      RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
      return;
    }

    // Default / joystick off: expand footprint from live arm / joint TF.
    const bool updated = updateFootprintIfChanged(true);
    response->success = true;
    response->message = updated
      ? "Joystick control disabled; publishing joint-state / arm footprint."
      : "Joystick control disabled; waiting for joint-state / TF footprint update.";
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
  }

  void jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    // While joystick teleop is active, keep the fixed base footprint.
    if (joystick_control_) {
      return;
    }

    const bool first_joint_state = !received_first_joint_state_;
    received_first_joint_state_ = true;
    if (!first_joint_state && !jointsMovedEnough(*msg)) {
      return;
    }
    storeJointPositions(*msg);
    updateFootprintIfChanged(first_joint_state);
  }

  std::string frame_id_;
  std::string footprint_topic_;
  std::string footprint_stamped_topic_;
  std::string local_costmap_footprint_topic_;
  std::string global_costmap_footprint_topic_;
  std::string local_costmap_published_footprint_topic_;
  std::string global_costmap_published_footprint_topic_;
  std::string joint_states_topic_;
  double joint_change_threshold_m_{0.01};
  double joint_change_threshold_rad_{0.01};
  double footprint_change_epsilon_m_{0.01};
  bool publish_costmap_topics_{true};
  bool publish_footprint_stamped_{false};
  bool local_costmap_ready_{false};
  bool global_costmap_ready_{false};
  bool published_local_costmap_footprint_{false};
  bool published_global_costmap_footprint_{false};
  bool received_first_joint_state_{false};
  bool joystick_control_{false};
  std::size_t last_stamped_subscriber_count_{0};

  std::vector<Eigen::Vector2f> base_polygon_;
  std::vector<Eigen::Vector2f> base_only_polygon_;
  std::vector<Eigen::Vector2f> last_published_hull_;
  std::unordered_map<std::string, double> last_joint_positions_;

  stretch_core::RobotSelfFilterConfig self_filter_config_;
  stretch_core::RobotSelfFilter self_filter_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr footprint_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr footprint_stamped_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr local_footprint_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr global_footprint_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr local_published_footprint_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr global_published_footprint_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr joystick_control_service_;
  rclcpp::TimerBase::SharedPtr stamped_subscriber_watch_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotFootprintPublisherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
