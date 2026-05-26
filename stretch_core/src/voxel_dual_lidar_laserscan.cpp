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
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/filter.h>
#include <pcl/common/transforms.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <omp.h>


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
        this->declare_parameter<std::string>("lidar1_frame", "link_lidar_right");
        this->declare_parameter<std::string>("lidar2_frame", "link_lidar_left");
        this->declare_parameter<std::string>("frame_id", "base_link");
        this->declare_parameter<std::string>("output_topic", "/scan_filtered");
        this->declare_parameter<bool>("pub_pc", false);

        // region
        this->declare_parameter("z_min", 0.135);
        this->declare_parameter("z_max", 1.5);
        this->declare_parameter("range_min", 0.25);  

        // VOXEL_SOR
        this->declare_parameter("dist_rob", 2.5);
        // VOXEL only
        this->declare_parameter("leaf_size", 0.05);
        // SOR only
        this->declare_parameter("sor_mean_k", 50);
        this->declare_parameter("sor_stddev", 0.3);

    
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

        // major effect on performance in terms of time it can take
        this->get_parameter("leaf_size", leaf_size_);
        this->get_parameter("sor_mean_k", sor_mean_k_);
        this->get_parameter("sor_stddev", sor_stddev_);
        this->get_parameter("dist_rob", dist_rob_);

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

    pcl::PointCloud<pcl::PointXYZ>::Ptr transform_pointcloud(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
        const Eigen::Matrix4f& tf_matrix)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::transformPointCloud(*cloud_in, *cloud_out, tf_matrix);
        return cloud_out;
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
            } else if (param.get_name() == "leaf_size") {
                leaf_size_ = param.as_double();
                RCLCPP_INFO(this->get_logger(), "Updated leaf_size_ to: %f", leaf_size_);
            } else if (param.get_name() == "sor_mean_k") {
                sor_mean_k_ = param.as_int();
                RCLCPP_INFO(this->get_logger(), "Updated sor_mean_k_ to: %d", sor_mean_k_);
            } else if (param.get_name() == "sor_stddev") {
                sor_stddev_ = param.as_double();
                RCLCPP_INFO(this->get_logger(), "Updated sor_stddev_ to: %f", sor_stddev_);
            } else if (param.get_name() == "dist_rob") {
                dist_rob_ = param.as_double();
                RCLCPP_INFO(this->get_logger(), "Updated dist_rob to: %f", dist_rob_);
            } else if (param.get_name() == "output_topic") {
                scan_topic_ = param.as_string();
                pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
                    scan_topic_, rclcpp::SensorDataQoS());
                RCLCPP_WARN(this->get_logger(), "output_topic updated: %s", scan_topic_.c_str());
            } else if (param.get_name() == "plane_fitting_threshold") {
                plane_fitting_threshold = param.as_double();
                RCLCPP_INFO(this->get_logger(), "Updated plane_fitting_threshold: %f", plane_fitting_threshold);
            } else if (param.get_name() == "angle") {
                angle_ = param.as_double();
                RCLCPP_INFO(this->get_logger(), "Updated angle: %f", angle_);
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
            }  else if (param.get_name() == "frame_id") {
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

    pcl::PointCloud<pcl::PointXYZ>::Ptr filter_voxel_sor (const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in) {
        // only take close points
        pcl::PointCloud<pcl::PointXYZ>::Ptr roi_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr outside_roi_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        for (const auto& pt : cloud_in->points) {
            if (pt.x >= -dist_rob_ && pt.x <= dist_rob_ && pt.y >= -dist_rob_ && pt.y <= dist_rob_ ) { // square 
                roi_cloud->points.push_back(pt);
            } else {
                outside_roi_cloud->points.push_back(pt);
            }
        }

        if (!roi_cloud->empty()){
            // voxel filtering        
            pcl::VoxelGrid<pcl::PointXYZ> voxel;
            voxel.setInputCloud(roi_cloud);
            voxel.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
            pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_filtered(new pcl::PointCloud<pcl::PointXYZ>());
            voxel.filter(*voxel_filtered);

            if (voxel_filtered->empty()) {
                RCLCPP_WARN(this->get_logger(), "voxel_filtered cloud is empty, skipping SOR");
                return nullptr;
            }

            // Apply Statistical Outlier Removal
            pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
            sor.setInputCloud(voxel_filtered);
            sor.setMeanK(sor_mean_k_);
            sor.setStddevMulThresh(sor_stddev_);
            pcl::PointCloud<pcl::PointXYZ>::Ptr sor_filtered(new pcl::PointCloud<pcl::PointXYZ>());
            sor.filter(*sor_filtered);

            pcl::PointCloud<pcl::PointXYZ>::Ptr merged(new pcl::PointCloud<pcl::PointXYZ>);
            *merged = *sor_filtered;  // copy the filtered cloud
            *merged += *outside_roi_cloud;  // append the other cloud
            return merged;
        
        }else{
            RCLCPP_WARN(this->get_logger(), "roi_cloud is empty");
            return cloud_in;
        }

    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr filter_region (const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, std::vector<float> & ranges) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>());
        
        // region filtering
        for (const auto& point : cloud_in->points) {
            float x = point.x, y = point.y, z = point.z;
            
            if ( z > z_max_) continue;
            if ( z < z_min_) continue;

            float r = std::hypot(x, y);
            float theta = std::atan2(y, x);
            if (r < range_min_ || r > range_max_) continue;

            if (pub_pc_){
                cloud_out->points.push_back(point);
            }
            
            // Ranges passed by reference so no need to do anything. It will be updated.
            int idx = static_cast<int>((theta - angle_min_) / angle_increment_);
            if (idx >= 0 && idx < num_ranges_) {
                ranges[idx] = std::min(ranges[idx], r);
            }
        
        }

        return cloud_out;
    }
    
    void timer_callback()
    {   
        // RCLCPP_INFO(this->get_logger(), "Wall timer triggered!");

        if (!msg1 || !msg2) {
            RCLCPP_WARN(this->get_logger(), "One or both LIDAR messages not yet received.");
            return;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>());

        // Convert ROS msg to PCL cloud
        pcl::fromROSMsg(*msg1, *cloud1);
        pcl::fromROSMsg(*msg2, *cloud2);

        // Transform each cloud (in-place)
        pcl::transformPointCloud(*cloud1, *cloud1, tf_lidar1_);
        pcl::transformPointCloud(*cloud2, *cloud2, tf_lidar2_);

        // filter only close points 
        // run in paraller 
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2;
        #pragma omp parallel sections
        {
            #pragma omp section
            {
                cloud_1 = filter_voxel_sor(cloud1);
            }

            #pragma omp section
            {
                cloud_2 = filter_voxel_sor(cloud2);
            }
        }
        
        std::vector<float> ranges(num_ranges_, range_max_);

        if (!cloud_1 || !cloud_2) {
            RCLCPP_WARN(this->get_logger(), "One of the filtered pointclouds is null, skipping this cycle.");
            return;
        }
        
        // Combine
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_merged_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        *transformed_merged_cloud = *cloud_1 + *cloud_2;

        // Remove NaNs
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*transformed_merged_cloud, *transformed_merged_cloud, indices);

        // filter 
        auto filter_region_pc = filter_region(transformed_merged_cloud, ranges);

        if (pub_pc_) {
            sensor_msgs::msg::PointCloud2 cloud_msg;
            pcl::toROSMsg(*filter_region_pc, cloud_msg);
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

    
    double leaf_size_;
    int sor_mean_k_;
    double sor_stddev_;
    double dist_rob_;

    float angle_ ;  // angle for perpendicular plane fitting in degrees
    double plane_fitting_threshold;

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
