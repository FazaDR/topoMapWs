#include "controller/idealized_lidar_controller_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

IdealizedLidarControllerNode::IdealizedLidarControllerNode()
: Node("idealized_lidar_controller")
{
  world_name_ = declare_parameter<std::string>("world", "winding_room");
  model_name_ = declare_parameter<std::string>("model", "idealized_lidar");
  speed_ = declare_parameter<double>("speed", 4.0);
  goal_tolerance_ = declare_parameter<double>("goal_tolerance", 0.05);
  max_yaw_rate_ = declare_parameter<double>("max_yaw_rate", 0.5);
  control_period_s_ = declare_parameter<double>("control_period", 0.02);
  align_threshold_ =
    declare_parameter<double>("align_threshold_deg", 2.0) * M_PI / 180.0;
  shutdown_on_finish_ = declare_parameter<bool>("shutdown_on_finish", false);

  const std::string turn_mode = declare_parameter<std::string>("turn_mode", "snap");
  if (turn_mode != "snap" && turn_mode != "smooth") {
    throw std::invalid_argument("turn_mode must be 'snap' or 'smooth'");
  }
  snap_turn_ = (turn_mode == "snap");
  dwell_s_ = declare_parameter<double>("turn_dwell_s", 0.5);

  if (speed_ <= 0.0 || goal_tolerance_ <= 0.0 || max_yaw_rate_ <= 0.0 ||
    control_period_s_ <= 0.0 || dwell_s_ < 0.0)
  {
    throw std::invalid_argument(
            "speed, goal_tolerance, max_yaw_rate, control_period must be positive");
  }

  // Waypoints are in the startup-local frame: (0, 0) is the spawn position.
  const std::vector<std::pair<double, double>> default_waypoints = {
    // {0.0, 0.0},
    // {0.0, 9.0},
    // {6.5, 9.0},
    // {6.5, 0.0},
    {16, 0},
    {16, -4},
    {0.0, -4},
    {0.0, -8},
    {16.0, -8},
    {16.0, -12},
    {0.0, -12},
    {0.0, -16},
    {16.0, -16},
    {16.0, -21},
    {-4.0, -21},
    {-4.0, 0.0},
    {0.0, 0.0},
  };
  waypoints_.reserve(default_waypoints.size());
  for (const auto &[x, y] : default_waypoints) {
    waypoints_.push_back({x, y, 0.0});
  }

  gz_node_.Subscribe(
    "/world/" + world_name_ + "/dynamic_pose/info",
    &IdealizedLidarControllerNode::poseCallback, this);

  trajectory_sub_ = create_subscription<nav_msgs::msg::Path>(
    "/idealized_lidar/trajectory", rclcpp::QoS(10),
    std::bind(
      &IdealizedLidarControllerNode::trajectoryCallback, this, std::placeholders::_1));

  // Node clock = sim time when use_sim_time is true.
  control_timer_ = rclcpp::create_timer(
    this, get_clock(),
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(control_period_s_)),
    std::bind(&IdealizedLidarControllerNode::controlLoop, this));

  RCLCPP_INFO(
    get_logger(),
    "Idealized LiDAR controller: %zu waypoints, speed %.2f m/s (%.1f mm/step), "
    "turn_mode=%s (dwell %.2f s, smooth rate %.2f rad/s), period %.0f ms.",
    waypoints_.size(), speed_, speed_ * control_period_s_ * 1000.0, turn_mode.c_str(),
    dwell_s_, max_yaw_rate_, control_period_s_ * 1000.0);
}

// Gazebo's pose only initializes the local frame. After that the controller
// dead-reckons, because late Gazebo samples would overwrite the commanded state.
void IdealizedLidarControllerNode::poseCallback(const gz::msgs::Pose_V &msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (initialized_) {
    return;
  }
  for (int i = 0; i < msg.pose_size(); ++i) {
    const auto &pose = msg.pose(i);
    if (pose.name() != model_name_) {
      continue;
    }
    x_ = origin_x_ = pose.position().x();
    y_ = origin_y_ = pose.position().y();
    z_ = origin_z_ = pose.position().z();
    const auto &q = pose.orientation();
    yaw_ = std::atan2(
      2.0 * (q.w() * q.z() + q.x() * q.y()),
      1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
    initialized_ = true;
    RCLCPP_INFO(get_logger(), "Local frame initialized.");
    return;
  }
}

void IdealizedLidarControllerNode::trajectoryCallback(
  const nav_msgs::msg::Path::SharedPtr msg)
{
  if (!msg->header.frame_id.empty() &&
    msg->header.frame_id != "odom" &&
    msg->header.frame_id != "base_link")
  {
    RCLCPP_ERROR(
      get_logger(),
      "Ignoring trajectory in unsupported frame '%s'; expected 'odom' or 'base_link'.",
      msg->header.frame_id.c_str());
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  waypoints_.clear();
  waypoints_.reserve(msg->poses.size());
  for (const auto &pose : msg->poses) {
    waypoints_.push_back({
      pose.pose.position.x, pose.pose.position.y, pose.pose.position.z});
  }
  current_waypoint_ = 0;
  finished_ = false;
  snap_pending_ = false;
  RCLCPP_INFO(get_logger(), "Received %zu trajectory waypoints.", waypoints_.size());
}

void IdealizedLidarControllerNode::controlLoop()
{
  const double now_s = get_clock()->now().seconds();
  if (last_sim_time_sec_ < 0.0) {
    last_sim_time_sec_ = now_s;
    return;
  }
  double dt = now_s - last_sim_time_sec_;
  last_sim_time_sec_ = now_s;
  if (dt <= 0.0) {
    return;                                   // sim time has not advanced
  }
  dt = std::min(dt, 2.0 * control_period_s_); // a late tick must not become a jump

  Waypoint target{};
  double cx, cy, cz, yaw;
  bool has_target = false;
  bool just_finished = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!initialized_) {
      return;
    }
    if (current_waypoint_ < waypoints_.size()) {
      target = waypoints_[current_waypoint_];
      has_target = true;
    } else if (!finished_) {
      finished_ = true;
      just_finished = true;
    }
    cx = x_ - origin_x_;
    cy = y_ - origin_y_;
    cz = z_ - origin_z_;
    yaw = yaw_;
  }

  if (just_finished) {
    RCLCPP_INFO(get_logger(), "Mission complete.");
    if (shutdown_on_finish_) {
      rclcpp::shutdown();
    }
    return;
  }
  if (!has_target) {
    return;
  }

  const double dx = target.x - cx;
  const double dy = target.y - cy;
  const double dz = target.z - cz;
  const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

  // ---------------- heading ----------------
  double heading_error = 0.0;
  if (std::hypot(dx, dy) > 1e-3) {
    const double desired = std::atan2(dy, dx);
    heading_error = std::atan2(std::sin(desired - yaw), std::cos(desired - yaw));

    if (snap_turn_) {
      if (now_s < hold_until_) {
        return;                               // stationary during the dwell
      }
      if (std::abs(heading_error) > align_threshold_) {
        if (!snap_pending_) {                 // pause before the jump
          snap_pending_ = true;
          hold_until_ = now_s + dwell_s_;
          return;
        }
        const Waypoint here{origin_x_ + cx, origin_y_ + cy, origin_z_ + cz};
        if (!setPose(here, desired)) {        // same position, new yaw
          return;                             // retry next tick
        }
        {
          std::lock_guard<std::mutex> lock(mutex_);
          yaw_ = desired;
        }
        snap_pending_ = false;
        hold_until_ = now_s + dwell_s_;       // pause after the jump
        return;
      }
    } else {
      const double max_turn = max_yaw_rate_ * dt;
      yaw += std::clamp(heading_error, -max_turn, max_turn);
      yaw = std::atan2(std::sin(yaw), std::cos(yaw));
      heading_error = std::atan2(std::sin(desired - yaw), std::cos(desired - yaw));
    }
  }
  const bool aligned = std::abs(heading_error) < align_threshold_;

  // ---------------- translation (never more than speed * dt) ----------------
  Waypoint next{cx, cy, cz};
  bool arrived = false;
  if (aligned && distance > 1e-9) {
    const double move = std::min(speed_ * dt, distance);
    const double s = move / distance;
    next = {cx + dx * s, cy + dy * s, cz + dz * s};
    arrived = (distance - move) <= goal_tolerance_;
  } else if (distance <= goal_tolerance_) {
    arrived = true;                           // already there (repeated waypoint)
  }

  const Waypoint world_pose{origin_x_ + next.x, origin_y_ + next.y, origin_z_ + next.z};
  if (!setPose(world_pose, yaw)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "Failed to move model '%s'.", model_name_.c_str());
    return;                                   // state unchanged, retry
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    x_ = world_pose.x;
    y_ = world_pose.y;
    z_ = world_pose.z;
    yaw_ = yaw;
    if (arrived) {
      RCLCPP_INFO(get_logger(), "Reached waypoint %zu.", current_waypoint_);
      ++current_waypoint_;
    }
  }
}

bool IdealizedLidarControllerNode::setPose(const Waypoint &pose, double yaw)
{
  gz::msgs::Pose request;
  request.set_name(model_name_);
  request.mutable_position()->set_x(pose.x);
  request.mutable_position()->set_y(pose.y);
  request.mutable_position()->set_z(pose.z);
  request.mutable_orientation()->set_x(0.0);
  request.mutable_orientation()->set_y(0.0);
  request.mutable_orientation()->set_z(std::sin(yaw / 2.0));
  request.mutable_orientation()->set_w(std::cos(yaw / 2.0));

  gz::msgs::Boolean reply;
  bool result = false;
  const bool executed = gz_node_.Request(
    "/world/" + world_name_ + "/set_pose", request, 100, reply, result);
  return executed && result && reply.data();
}