#include "mapping/mapping_node.hpp"

MappingNode::MappingNode()
: Node("mapping_node")
{
    point_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/points/points",
        10,
        std::bind(
            &MappingNode::pointCloudCallback,
            this,
            std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Mapping node started.");
}

void MappingNode::pointCloudCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    RCLCPP_INFO(
        get_logger(),
        "Cloud received | frame=%s | width=%u | height=%u",
        msg->header.frame_id.c_str(),
        msg->width,
        msg->height);
}