#include "controller/controller_node.hpp"

#include <algorithm>
#include <cmath>

ControllerNode::ControllerNode()
: Node("controller_node"),
  robot_x_(0.0),
  robot_y_(0.0),
  robot_yaw_(0.0),
  initialized_(false),
  origin_x_(0.0),
  origin_y_(0.0),
  origin_yaw_(0.0),
  current_waypoint_(0),
  state_(State::ROTATE)
{
    waypoints_ = {
        {16, 0},
        {16, -4},
        {0.0, -4},
        {0.0, -8},
        {16.0, -8},
        {16.0, -12},
        {0.0, -12},
        {0.0, -16},
        {16.0, -16},
    };

    gz_node_.Subscribe(
        "/world/winding_room/dynamic_pose/info",
        &ControllerNode::poseCallback,
        this);

    cmd_vel_pub_ =
        create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel",
            10);

    control_timer_ =
        create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&ControllerNode::controlLoop, this));

    RCLCPP_INFO(get_logger(), "Controller node started.");
}

void ControllerNode::poseCallback(const gz::msgs::Pose_V &msg)
{
    for (int i = 0; i < msg.pose_size(); ++i)
    {
        const auto &pose = msg.pose(i);

        if (pose.name() != "ddmr")
            continue;

        robot_x_ = pose.position().x();
        robot_y_ = pose.position().y();

        const auto &q = pose.orientation();

        robot_yaw_ = std::atan2(
            2.0 * (q.w()*q.z() + q.x()*q.y()),
            1.0 - 2.0 * (q.y()*q.y() + q.z()*q.z()));

        if (!initialized_)
        {
            origin_x_ = robot_x_;
            origin_y_ = robot_y_;
            origin_yaw_ = robot_yaw_;

            initialized_ = true;

            RCLCPP_INFO(get_logger(), "Local frame initialized.");
        }

        break;
    }
}

void ControllerNode::controlLoop()
{
    if (!initialized_)
        return;

    if (state_ == State::FINISHED)
        return;

    geometry_msgs::msg::Twist cmd;

    //------------------------------------------
    // Local pose
    //------------------------------------------

    double x = robot_x_ - origin_x_;
    double y = robot_y_ - origin_y_;
    double yaw = robot_yaw_ - origin_yaw_;

    //------------------------------------------
    // Goal
    //------------------------------------------

    const auto &goal = waypoints_[current_waypoint_];

    Waypoint previous =
        (current_waypoint_ == 0)
        ? Waypoint{0.0, 0.0}
        : waypoints_[current_waypoint_ - 1];

    double distance = std::hypot(goal.x - x, goal.y - y);

    //------------------------------------------
    // Path direction
    //------------------------------------------

    double path_x = goal.x - previous.x;
    double path_y = goal.y - previous.y;

    double path_length = std::hypot(path_x, path_y);

    if (path_length < 1e-6)
    {
        RCLCPP_WARN(
            get_logger(),
            "Waypoint %zu is identical to previous waypoint.",
            current_waypoint_);
        return;
    }

    path_x /= path_length;
    path_y /= path_length;

    //------------------------------------------
    // Tracking errors
    //------------------------------------------

    double goal_to_robot_x = x - goal.x;
    double goal_to_robot_y = y - goal.y;

    double along_track =
        goal_to_robot_x * path_x +
        goal_to_robot_y * path_y;

    double cross_track =
        goal_to_robot_x * (-path_y) +
        goal_to_robot_y * path_x;

    double desired_heading =
        std::atan2(path_y, path_x)
        - std::atan2(2.0 * cross_track, 1.0);

    double heading_error = desired_heading - yaw;

    while (heading_error > M_PI)
        heading_error -= 2.0 * M_PI;

    while (heading_error < -M_PI)
        heading_error += 2.0 * M_PI;

    //------------------------------------------
    // State Machine
    //------------------------------------------

    constexpr double heading_threshold = 5.0 * M_PI / 180.0;
    constexpr double goal_threshold = 0.05;

    switch (state_)
    {
    case State::ROTATE:
    {
        cmd.angular.z = std::clamp(
            2.0 * heading_error,
            -5.0,
            5.0);

        if (std::abs(heading_error) < heading_threshold)
        {
            state_ = State::DRIVE;

            RCLCPP_INFO(
                get_logger(),
                "Heading aligned.");
        }

        break;
    }

    case State::DRIVE:
    {
        cmd.linear.x = std::clamp(
            0.7 * distance,
            0.0,
            1.0);

        if (along_track >= 0.0 || distance < goal_threshold)
        {
            current_waypoint_++;

            if (current_waypoint_ >= waypoints_.size())
            {
                state_ = State::FINISHED;

                cmd.linear.x = 0.0;
                cmd.angular.z = 0.0;

                RCLCPP_INFO(
                    get_logger(),
                    "Mission complete.");
            }
            else
            {
                state_ = State::ROTATE;

                RCLCPP_INFO(
                    get_logger(),
                    "Waypoint %zu reached.",
                    current_waypoint_ - 1);
            }
        }

        break;
    }

    case State::FINISHED:
        break;
    }

    cmd_vel_pub_->publish(cmd);

    RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "Pose (%.2f %.2f) Goal (%.2f %.2f) State %d",
        x,
        y,
        goal.x,
        goal.y,
        static_cast<int>(state_));
}