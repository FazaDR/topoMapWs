#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <gz/transport/Node.hh>
#include <gz/msgs/pose_v.pb.h>

class MappingNode : public rclcpp::Node
{
public:
    MappingNode();

private:

    // ROS subscriber
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_sub_;

    // Gazebo subscriber
    gz::transport::Node gz_node_;

    // Callbacks
    void pointCloudCallback(
        const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    void poseCallback(
        const gz::msgs::Pose_V &msg);
};