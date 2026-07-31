    #pragma once

#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <gz/transport/Node.hh>
#include <gz/msgs/pose_v.pb.h>

class ControllerNode : public rclcpp::Node
{
public:
    ControllerNode();

private:

    struct Waypoint
    {
        double x;
        double y;
    };

    // Gazebo
    gz::transport::Node gz_node_;

    // ROS
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // Robot pose
    double robot_x_;
    double robot_y_;
    double robot_yaw_;

    // Local frame
    bool initialized_;
    double origin_x_;
    double origin_y_;
    double origin_yaw_;

    bool finished_;

    // Path
    std::vector<Waypoint> waypoints_;
    std::size_t current_waypoint_;

    void poseCallback(const gz::msgs::Pose_V &msg);
    void controlLoop();
};