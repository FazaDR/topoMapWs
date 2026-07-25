#include "mapping/mapping_node.hpp"


#include <functional>

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

    gz_node_.Subscribe(
        "/world/winding_room/dynamic_pose/info",
        &MappingNode::poseCallback,
        this);

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

void MappingNode::poseCallback(const gz::msgs::Pose_V &msg)
{
    for (int i = 0; i < msg.pose_size(); ++i)
    {
        const auto &pose = msg.pose(i);

        if (pose.name() == "ddmr")
        {
            RCLCPP_INFO(
                get_logger(),
                "Robot | x=%.3f y=%.3f z=%.3f",
                pose.position().x(),
                pose.position().y(),
                pose.position().z());

            break;
        }
    }
}