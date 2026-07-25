#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class MappingNode : public rclcpp::Node
{
public:
    MappingNode();

private:
    void pointCloudCallback(
        const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_sub_;
};