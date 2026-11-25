#include "controller/controller_node.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

using controller::StepContext;
using controller::wrapAngle;

ControllerNode::ControllerNode()
: controller::ControllerBase("ddmr_controller", "ddmr", true)
{
  angular_gain_ = declare_parameter<double>("angular_gain", 2.0);
  max_angular_ = declare_parameter<double>("max_angular", 5.0);
  linear_gain_ = declare_parameter<double>("linear_gain", 0.4);
  max_linear_ = declare_parameter<double>("max_linear", 3.0);
  heading_threshold_ = declare_parameter<double>("heading_threshold_deg", 5.0) * M_PI / 180.0;
  drive_steer_gain_ = declare_parameter<double>("drive_steer_gain", 0.0);
  const std::string cmd_topic = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");

  if (max_linear_ <= 0.0 || max_angular_ <= 0.0) {
    throw std::invalid_argument("max_linear and max_angular must be positive");
  }

  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_topic, 10);

  RCLCPP_INFO(
    get_logger(), "ddmr controller: model=%s world=%s cmd=%s period %.0f ms.",
    model_.c_str(), world_.c_str(), cmd_topic.c_str(), period_s_ * 1000.0);
}

void ControllerNode::publishStop()
{
  cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
}

bool ControllerNode::step(const StepContext & c)
{
  const auto & goal = c.target;

  // Leg direction (previous waypoint -> goal).
  double px = goal.x - c.previous.x;
  double py = goal.y - c.previous.y;
  const double path_length = std::hypot(px, py);
  if (path_length < 1e-6) {
    RCLCPP_WARN(get_logger(), "Waypoint is identical to the previous one, skipping it.");
    return true;
  }
  px /= path_length;
  py /= path_length;

  const double distance = std::hypot(goal.x - c.pose.x, goal.y - c.pose.y);

  // Tracking errors relative to the goal.
  const double gx = c.pose.x - goal.x;
  const double gy = c.pose.y - goal.y;
  const double along_track = gx * px + gy * py;
  const double cross_track = gx * (-py) + gy * px;

  const double desired_heading = std::atan2(py, px) - std::atan2(2.0 * cross_track, 1.0);
  const double heading_error = wrapAngle(desired_heading - c.pose.yaw);

  geometry_msgs::msg::Twist cmd;
  bool reached = false;

  if (state_ == State::ROTATE) {
    cmd.angular.z = std::clamp(angular_gain_ * heading_error, -max_angular_, max_angular_);
    if (std::abs(heading_error) < heading_threshold_) {
      state_ = State::DRIVE;
    }
  } else {
    cmd.linear.x = std::clamp(linear_gain_ * distance, 0.0, max_linear_);
    cmd.angular.z =
      std::clamp(drive_steer_gain_ * heading_error, -max_angular_, max_angular_);
    if (along_track >= 0.0 || distance < goal_tolerance_) {
      reached = true;
      cmd = geometry_msgs::msg::Twist();   // stop at the waypoint, then rotate
    }
  }

  cmd_vel_pub_->publish(cmd);

  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 1000, "Pose (%.2f %.2f) Goal (%.2f %.2f) State %s",
    c.pose.x, c.pose.y, goal.x, goal.y, state_ == State::ROTATE ? "ROTATE" : "DRIVE");
  return reached;
}
