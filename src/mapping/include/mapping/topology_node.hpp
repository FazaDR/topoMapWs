#pragma once

#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <Eigen/Core>

#include "mapping/atcdt.hpp"

class TopologyNode : public rclcpp::Node
{
public:

    TopologyNode();

private:

    //---------------------------------------
    // ROS
    //---------------------------------------

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr topology_pub_;

    //---------------------------------------
    // ATC-DT
    //---------------------------------------

    ATCDT atcdt_;

    //---------------------------------------
    // Callback
    //---------------------------------------

    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    //---------------------------------------
    // Helpers
    //---------------------------------------

    std::vector<Eigen::Vector3f>convertPointCloud(const sensor_msgs::msg::PointCloud2 &cloud) const;
    
    void publishTopology();
};