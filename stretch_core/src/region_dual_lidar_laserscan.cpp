#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <Eigen/Dense>
#include <cmath>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>

class DualPointCloudToLaserScan : public rclcpp::Node {
public:
    DualPointCloudToLaserScan()
        : Node("dual_pointcloud_to_laserscan"),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_)
    {
        // general 
        this->declare_parameter<std::string>("lidar1_topic", "/lidar_points_right");
        this->declare_parameter<std::string>("lidar2_topic", "/lidar_points_left");
        this->declare_parameter<std::string>("lidar1_frame", "lidar_right_link");
        this->declare_parameter<std::string>("lidar2_frame", "lidar_left_link");
        this->declare_parameter<std::string>("frame_id", "base_footprint");
        this->declare_parameter<std::string>("output_topic", "/scan_filtered");
        this->declare_parameter<bool>("pub_pc", false);

        // region
        this->declare_parameter("z_min", 0.135);
        this->declare_parameter("z_max", 1.5);
        this->declare_parameter("range_min", 0.25);  // minimum range to avoid robot base, should be same as robot_raduis

        this->get_parameter("lidar1_topic", lidar1_topic_);
        this->get_parameter("lidar2_topic", lidar2_topic_);
        this->get_parameter("lidar1_frame", lidar1_frame_);
        this->get_parameter("lidar2_frame", lidar2_frame_);
        this->get_parameter("frame_id", target_frame_);
        this->get_parameter("output_topic", scan_topic_);
        this->get_parameter("pub_pc", pub_pc_);

        this->get_parameter("z_min", z_min_);
        this->get_parameter("z_max", z_max_);
        this->get_parameter("range_min", range_min_);

        pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(scan_topic_, rclcpp::SensorDataQoS());

        if (pub_pc_){
            pub_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/filtered_points", rclcpp::SensorDataQoS());
        }

        angle_min_ = -M_PI;
        angle_max_ = M_PI;
        angle_increment_ = 0.05 * M_PI / 180.0;
        range_max_ = 30.0;
    
        num_ranges_ = static_cast<int>((angle_max_ - angle_min_) / angle_increment_);
    
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&DualPointCloudToLaserScan::on_parameter_change, this, std::placeholders::_1));

    }

    rcl_interfaces::msg::SetParametersResult on_parameter_change(
        const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";

        for (const auto &param : parameters) {
            if (param.get_name() == "z_min") {
                z_min_ = param.as_double();
                RCLCPP_WARN(this->get_logger(), "z_min updated: %f", z_min_);
            } else if (param.get_name() == "z_max") {
                z_max_ = param.as_double();
                RCLCPP_WARN(this->get_logger(), "z_max updated: %f", z_max_);
            } else if (param.get_name() == "lidar1_frame") {
                lidar1_frame_ = param.as_string();
                RCLCPP_WARN(this->get_logger(), "lidar1_frame updated: %s", lidar1_frame_.c_str());
            } else if (param.get_name() == "lidar2_frame") {
                lidar2_frame_ = param.as_string();
                RCLCPP_WARN(this->get_logger(), "lidar2_frame updated: %s", lidar2_frame_.c_str());
            } else if (param.get_name() == "lidar1_topic") {
                lidar1_topic_ = param.as_string();
                RCLCPP_WARN(this->get_logger(), "lidar1_topic updated: %s", lidar1_topic_.c_str());
            } else if (param.get_name() == "lidar2_topic") {
                lidar2_topic_ = param.as_string();
                RCLCPP_WARN(this->get_logger(), "lidar2_topic updated: %s", lidar2_topic_.c_str());
            } else if (param.get_name() == "output_topic") {
                scan_topic_ = param.as_string();
                pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
                    scan_topic_, rclcpp::SensorDataQoS());
                RCLCPP_WARN(this->get_logger(), "output_topic updated: %s", scan_topic_.c_str());
            } else if (param.get_name() == "pub_pc") {
                pub_pc_ = param.as_bool();
                if (pub_pc_) {
                    pub_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                        "/filtered_points", rclcpp::SensorDataQoS());
                    RCLCPP_WARN(this->get_logger(), "PointCloud publisher enabled.");
                } else {
                    pub_cloud_.reset(); // disables publisher
                    RCLCPP_WARN(this->get_logger(), "PointCloud publisher disabled.");
                }
            } else if (param.get_name() == "frame_id") {
                result.successful = false;
                result.reason = "frame_id cannot be changed at runtime (only at launch).";
                RCLCPP_WARN(this->get_logger(), 
                    "Attempted to change parameter '%s' at runtime – rejected: %s", 
                    param.get_name().c_str(), 
                    result.reason.c_str());
            } else {
                result.successful = false;
                result.reason = "Unknown parameter";
                RCLCPP_WARN(this->get_logger(), "Unknown parameter: %s", param.get_name().c_str());
            }
        }
        return result;
    }

    bool lookup_static_transforms() {
        try {
            // change to be dynamic name
            RCLCPP_INFO(this->get_logger(), "Looking up transforms: target_frame='%s', lidar1_frame='%s', lidar2_frame='%s'",
                target_frame_.c_str(), lidar1_frame_.c_str(), lidar2_frame_.c_str());

            auto tf1 = tf_buffer_.lookupTransform(target_frame_, lidar1_frame_, tf2::TimePointZero);
            auto tf2 = tf_buffer_.lookupTransform(target_frame_, lidar2_frame_, tf2::TimePointZero);
            Eigen::Affine3d tf1_affine = tf2::transformToEigen(tf1.transform);
            Eigen::Affine3d tf2_affine = tf2::transformToEigen(tf2.transform);

            tf_lidar1_ = tf1_affine.matrix().cast<float>();
            tf_lidar2_ = tf2_affine.matrix().cast<float>();
        
            RCLCPP_INFO(this->get_logger(), "Transforms cached.");
            tf_available_ = true;
            return true;
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Waiting for TFs: %s", ex.what());
            return false;
        }
    }

    void activate_subscription() {

        RCLCPP_INFO(this->get_logger(), "Activate subscription.");
        // Setup subscribers with debug callbacks
        rclcpp::QoS qos_profile(rclcpp::KeepLast(100));
        qos_profile.reliable();
        using std::placeholders::_1;
        sub1_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                lidar1_topic_, qos_profile,
                std::bind(&DualPointCloudToLaserScan::pointcloud_callback1, this, _1));

        sub2_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                lidar2_topic_, qos_profile,
                std::bind(&DualPointCloudToLaserScan::pointcloud_callback2, this, _1));

        timer_ = this->create_wall_timer(
                std::chrono::milliseconds(100), 
                std::bind(&DualPointCloudToLaserScan::timer_callback, this));
    }


private:
    void pointcloud_callback1(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        if (!tf_available_) return;
        msg1 = msg;
    }

    void pointcloud_callback2(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        if (!tf_available_) return;
        msg2 = msg;
    }


    void timer_callback()
    {   
        // RCLCPP_INFO(this->get_logger(), "Wall timer triggered!");
        if (!msg1 || !msg2) {
            RCLCPP_WARN(this->get_logger(), "One or both LIDAR messages not yet received.");
            return;
    }

        std::vector<Eigen::Vector4f> combined_points;
        collect_points(msg1, combined_points, tf_lidar1_);
        collect_points(msg2, combined_points, tf_lidar2_);

        std::vector<float> ranges(num_ranges_, range_max_);
        // --- publish filtered pointcloud ---
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        for (const auto& pt_tf : combined_points) {
            float x = pt_tf.x(), y = pt_tf.y(), z = pt_tf.z();
            // if (z < z_min_ || z > z_max_) continue;
            
            if ( z > z_max_) continue;
            if ( z < z_min_) continue;

            float r = std::hypot(x, y);
            float theta = std::atan2(y, x);
            if (r < range_min_ || r > range_max_) continue;

            if (pub_pc_){
            filtered_cloud->points.emplace_back(x, y, z);
            }

            int idx = static_cast<int>((theta - angle_min_) / angle_increment_);
            if (idx >= 0 && idx < num_ranges_) {
                ranges[idx] = std::min(ranges[idx], r);
            }
            
        }

        if (pub_pc_) {
            sensor_msgs::msg::PointCloud2 cloud_msg;
            pcl::toROSMsg(*filtered_cloud, cloud_msg);
            cloud_msg.header.stamp = this->now();
            cloud_msg.header.frame_id = target_frame_;
            pub_cloud_->publish(cloud_msg);
        }

        auto scan = std::make_shared<sensor_msgs::msg::LaserScan>();
        scan->header.stamp = this->now();
        scan->header.frame_id = "laser";
        scan->angle_min = angle_min_;
        scan->angle_max = angle_max_;
        scan->angle_increment = angle_increment_;
        scan->range_min = range_min_;
        scan->range_max = range_max_;
        scan->ranges = ranges;
        pub_->publish(*scan);
    }

    void collect_points(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg, std::vector<Eigen::Vector4f> &points, const Eigen::Matrix4f &tf_matrix)
    {   
        
        size_t count = 0;
        
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
            Eigen::Vector4f pt(*iter_x, *iter_y, *iter_z, 1.0f);
            Eigen::Vector4f pt_tf = tf_matrix * pt;
            points.push_back(pt_tf);

            if(debug){
                ++count;
            }
        }

        if(debug){
            RCLCPP_INFO(this->get_logger(), "Collected %ld points from %s", count, msg->header.frame_id.c_str());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub1_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub2_;
    rclcpp::TimerBase::SharedPtr timer_;
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    sensor_msgs::msg::PointCloud2::ConstSharedPtr msg1;
    sensor_msgs::msg::PointCloud2::ConstSharedPtr msg2;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    std::string lidar1_topic_;
    std::string lidar2_topic_;
    std::string lidar1_frame_;
    std::string lidar2_frame_;
    std::string scan_topic_;
    std::string target_frame_;

    float angle_min_, angle_max_, angle_increment_;
    float range_min_, range_max_;
    float z_min_, z_max_;
    // float x_min_exclude_, x_max_exclude_;
    // float y_min_exclude_, y_max_exclude_;
    int num_ranges_;
    Eigen::Matrix4f tf_lidar1_;
    Eigen::Matrix4f tf_lidar2_;
    bool debug = false;
    bool tf_available_ = false;
    bool pub_pc_;

};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DualPointCloudToLaserScan>();

    // Block until the transform is acquired
    rclcpp::Rate rate(10);  // 10 Hz retry rate
    while (rclcpp::ok() && !node->lookup_static_transforms()) {
        rclcpp::spin_some(node);
        rate.sleep();
    }

    // Once TF is available, start main subscription
    node->activate_subscription();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
