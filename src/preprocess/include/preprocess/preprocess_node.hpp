#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <gz/msgs/pose_v.pb.h>
#include <gz/transport/Node.hh>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <mutex>
#include <string>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "preprocess/msg/processed_scan.hpp"

class PreprocessNode : public rclcpp::Node
{
public:

    PreprocessNode();

private:


    //---------------------------------------
    // ROS
    //---------------------------------------

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_sub_;

    rclcpp::Publisher<preprocess::msg::ProcessedScan>::SharedPtr processed_scan_pub_;

    //---------------------------------------
    // Gazebo
    //---------------------------------------

    gz::transport::Node gz_node_;

    //---------------------------------------
    // Sensor extrinsic
    //---------------------------------------

    Eigen::Vector3f lidar_offset_;

    std::string robot_name_;
    std::mutex pose_mutex_;


    //---------------------------------------
    // Robot pose
    //---------------------------------------
    

    Eigen::Vector3f robot_position_;

    Eigen::Quaternionf robot_orientation_;

    bool pose_received_;
    uint64_t source_frame_id_{0};

    //---------------------------------------
    // Callbacks
    //---------------------------------------

    void pointCloudCallback(
        const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    void poseCallback(
        const gz::msgs::Pose_V &msg);

    //---------------------------------------
    // Helpers
    //---------------------------------------

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    void publishTF(const Eigen::Vector3f &pos, const Eigen::Quaternionf &q);

    sensor_msgs::msg::PointCloud2 transformCloud(
        const sensor_msgs::msg::PointCloud2 &cloud,
        const Eigen::Vector3f &pos,
        const Eigen::Quaternionf &q);
};