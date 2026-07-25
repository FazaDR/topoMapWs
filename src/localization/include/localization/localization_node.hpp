#pragma once

#include <rclcpp/rclcpp.hpp>

#include <gz/transport/Node.hh>
#include <gz/msgs/pose_v.pb.h>

class LocalizationNode : public rclcpp::Node
{
public:
    LocalizationNode();

private:
    gz::transport::Node gz_node_;

    void poseCallback(const gz::msgs::Pose_V &msg);
};