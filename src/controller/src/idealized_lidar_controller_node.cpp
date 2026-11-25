#include "controller/idealized_lidar_controller_node.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/pose.pb.h>

using controller::StepContext;
using controller::Waypoint;
using controller::wrapAngle;

IdealizedLidarControllerNode::IdealizedLidarControllerNode()
: controller::ControllerBase("idealized_lidar_controller", "idealized_lidar", false)
{
  speed_ = declare_parameter<double>("speed", 0.5);
  max_yaw_rate_ = declare_parameter<double>("max_yaw_rate", 0.5);
  align_threshold_ = declare_parameter<double>("align_threshold_deg", 2.0) * M_PI / 180.0;
  dwell_s_ = declare_parameter<double>("turn_dwell_s", 0.5);

  const std::string turn_mode = declare_parameter<std::string>("turn_mode", "snap");
  if (turn_mode != "snap" && turn_mode != "smooth") {
    throw std::invalid_argument("turn_mode must be 'snap' or 'smooth'");
  }
  snap_turn_ = (turn_mode == "snap");

  if (speed_ <= 0.0 || max_yaw_rate_ <= 0.0 || dwell_s_ < 0.0) {
    throw std::invalid_argument("speed and max_yaw_rate must be positive, turn_dwell_s >= 0");
  }

  RCLCPP_INFO(
    get_logger(),
    "Idealized LiDAR controller: model=%s world=%s speed %.2f m/s (%.1f mm/step), "
    "turn_mode=%s (dwell %.2f s, smooth rate %.2f rad/s), period %.0f ms.",
    model_.c_str(), world_.c_str(), speed_, speed_ * period_s_ * 1000.0, turn_mode.c_str(),
    dwell_s_, max_yaw_rate_, period_s_ * 1000.0);
}

bool IdealizedLidarControllerNode::step(const StepContext & c)
{
  const double dx = c.target.x - c.pose.x;
  const double dy = c.target.y - c.pose.y;
  const double dz = c.target.z - c.pose.z;
  const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

  double yaw = c.pose.yaw;

  // ---------------- heading ----------------
  double heading_error = 0.0;
  if (std::hypot(dx, dy) > 1e-3) {
    const double desired = std::atan2(dy, dx);
    heading_error = wrapAngle(desired - yaw);

    if (snap_turn_) {
      if (c.now_s < hold_until_) {
        return false;                          // stationary during the dwell
      }
      if (std::abs(heading_error) > align_threshold_) {
        if (!snap_pending_) {                  // pause before the jump
          snap_pending_ = true;
          hold_until_ = c.now_s + dwell_s_;
          return false;
        }
        if (!teleport(toWorld(c.pose), desired)) {   // same position, new yaw
          return false;                        // retry next tick
        }
        setLocalState(c.pose.x, c.pose.y, c.pose.z, desired);
        snap_pending_ = false;
        hold_until_ = c.now_s + dwell_s_;      // pause after the jump
        return false;
      }
    } else {
      const double max_turn = max_yaw_rate_ * c.dt;
      yaw = wrapAngle(yaw + std::clamp(heading_error, -max_turn, max_turn));
      heading_error = wrapAngle(desired - yaw);
    }
  }
  const bool aligned = std::abs(heading_error) < align_threshold_;

  // ---------------- translation (never more than speed * dt) ----------------
  controller::Pose next = c.pose;
  bool arrived = false;
  if (aligned && distance > 1e-9) {
    const double move = std::min(speed_ * c.dt, distance);
    const double s = move / distance;
    next.x = c.pose.x + dx * s;
    next.y = c.pose.y + dy * s;
    next.z = c.pose.z + dz * s;
    arrived = (distance - move) <= goal_tolerance_;
  } else if (distance <= goal_tolerance_) {
    arrived = true;                            // already there (repeated waypoint)
  }
  next.yaw = yaw;

  if (!teleport(toWorld(next), yaw)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "Failed to move model '%s'.", model_.c_str());
    return false;                              // state unchanged, retry
  }
  setLocalState(next.x, next.y, next.z, yaw);
  return arrived;
}

bool IdealizedLidarControllerNode::teleport(const Waypoint & p, double yaw)
{
  gz::msgs::Pose request;
  request.set_name(model_);
  request.mutable_position()->set_x(p.x);
  request.mutable_position()->set_y(p.y);
  request.mutable_position()->set_z(p.z);
  request.mutable_orientation()->set_x(0.0);
  request.mutable_orientation()->set_y(0.0);
  request.mutable_orientation()->set_z(std::sin(yaw / 2.0));
  request.mutable_orientation()->set_w(std::cos(yaw / 2.0));

  gz::msgs::Boolean reply;
  bool result = false;
  const bool executed =
    gz_node_.Request("/world/" + world_ + "/set_pose", request, 100, reply, result);
  return executed && result && reply.data();
}
