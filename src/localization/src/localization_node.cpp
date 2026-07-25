#include "localization/localization_node.hpp"

#include <gz/math/Pose3.hh>

LocalizationNode::LocalizationNode()
: Node("localization_node")
{
    RCLCPP_INFO(get_logger(), "Localization node started.");

    gz_node_.Subscribe(
        "/world/winding_room/dynamic_pose/info",
        &LocalizationNode::poseCallback,
        this);
}

void LocalizationNode::poseCallback(const gz::msgs::Pose_V &msg)
{
    for (int i = 0; i < msg.pose_size(); ++i)
    {
        const auto &pose = msg.pose(i);

        if (pose.name() == "ddmr")
        {
            double x = pose.position().x();
            double y = pose.position().y();
            double z = pose.position().z();

            double qw = pose.orientation().w();
            double qx = pose.orientation().x();
            double qy = pose.orientation().y();
            double qz = pose.orientation().z();

            RCLCPP_INFO(
                get_logger(),
                "Robot: x=%.3f y=%.3f z=%.3f | q=(%.3f %.3f %.3f %.3f)",
                x, y, z,
                qx, qy, qz, qw);

            break;
        }
    }
}