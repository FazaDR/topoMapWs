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

class PreprocessNode : public rclcpp::Node
{
public:

    PreprocessNode();

private:


    //---------------------------------------
    // ROS
    //---------------------------------------

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
        point_sub_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        global_cloud_pub_;

    //---------------------------------------
    // Gazebo
    //---------------------------------------

    gz::transport::Node gz_node_;

    //---------------------------------------
    // Sensor extrinsic
    //---------------------------------------

    Eigen::Vector3f lidar_offset_;


    //---------------------------------------
    // Robot pose
    //---------------------------------------
    

    Eigen::Vector3f robot_position_;

    Eigen::Quaternionf robot_orientation_;

    bool pose_received_;

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
    void publishTF();

    sensor_msgs::msg::PointCloud2 transformCloud(
        const sensor_msgs::msg::PointCloud2 &cloud);
};