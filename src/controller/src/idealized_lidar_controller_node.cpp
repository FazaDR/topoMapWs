#include "controller/idealized_lidar_controller_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

IdealizedLidarControllerNode::IdealizedLidarControllerNode()
: Node("idealized_lidar_controller"),
  world_name_(declare_parameter("world", "winding_room")),
  model_name_(declare_parameter("model", "idealized_lidar")),
  speed_(declare_parameter("speed", 0.5)),
  goal_tolerance_(declare_parameter("goal_tolerance", 0.75))
{
  if (speed_ <= 0.0 || goal_tolerance_ <= 0.0) {
    throw std::invalid_argument("speed and goal_tolerance must be positive");
  }

  // Waypoints use the same startup-local frame as the DDMR controller. The
  // initial pose is (0, 0, 0), so this path starts at the spawned pose.
  const std::vector<std::pair<double, double>> default_waypoints = {
    {0.0, 0.0},
    {0.0, 9.0},
    {6.5, 9.0},
    {6.5, 0.0},
    // {16, 0},
    // {16, -4},
    // {0.0, -4},
    // {0.0, -8},
    // {16.0, -8},
    // {16.0, -12},
    // {0.0, -12},
    // {0.0, -16},
    // {16.0, -16},
    // {16.0, -21},
    // {-4.0, -21},
    // {-4.0, 0.0},
    // {0.0, 0.0},
  };
  waypoints_.reserve(default_waypoints.size());
  for (const auto &[x, y] : default_waypoints) {
    waypoints_.push_back({x, y, 0.0});
  }

  gz_node_.Subscribe(
    "/world/" + world_name_ + "/dynamic_pose/info",
    &IdealizedLidarControllerNode::poseCallback,
    this);

  trajectory_sub_ = create_subscription<nav_msgs::msg::Path>(
    "/idealized_lidar/trajectory",
    rclcpp::QoS(10),
    std::bind(
      &IdealizedLidarControllerNode::trajectoryCallback,
      this,
      std::placeholders::_1));

  control_timer_ = rclcpp::create_timer(
    this,
    get_clock(),
    std::chrono::milliseconds(50),
    std::bind(&IdealizedLidarControllerNode::controlLoop, this));

  RCLCPP_INFO(
    get_logger(),
    "Idealized LiDAR controller started with %zu default waypoints. "
    "Publish a replacement startup-local trajectory on "
    "/idealized_lidar/trajectory.",
    waypoints_.size());
}

void IdealizedLidarControllerNode::poseCallback(const gz::msgs::Pose_V &msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (int i = 0; i < msg.pose_size(); ++i) {
    const auto &pose = msg.pose(i);
    if (pose.name() != model_name_) {
      continue;
    }
    x_ = pose.position().x();
    y_ = pose.position().y();
    z_ = pose.position().z();
    pose_received_ = true;
    if (!initialized_) {
      origin_x_ = x_;
      origin_y_ = y_;
      origin_z_ = z_;
      initialized_ = true;
      RCLCPP_INFO(get_logger(), "Local frame initialized.");
      const auto &q = pose.orientation();
      yaw_ = std::atan2(2.0 * (q.w()*q.z() + q.x()*q.y()),
                        1.0 - 2.0 * (q.y()*q.y() + q.z()*q.z()));
    }
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
      pose.pose.position.x,
      pose.pose.position.y,
      pose.pose.position.z,
    });
  }
  current_waypoint_ = 0;
  RCLCPP_INFO(get_logger(), "Received %zu trajectory waypoints.", waypoints_.size());
}

void IdealizedLidarControllerNode::controlLoop()
{
  // Read simulation time.
  const double current_sim_time = get_clock()->now().seconds();

  // Initialize the timestamp on the first callback.
  if (last_sim_time_sec_ < 0.0) {
    last_sim_time_sec_ = current_sim_time;
    return;
  }

  const double dt = current_sim_time - last_sim_time_sec_;
  last_sim_time_sec_ = current_sim_time;

  // Do not move if simulation time has not advanced.
  if (dt <= 0.0) {
    return;
  }

  Waypoint target;
  double current_x;
  double current_y;
  double current_z; 
  double yaw = yaw_;


  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!initialized_ || current_waypoint_ >= waypoints_.size()) {
      return;
    }

    target = waypoints_[current_waypoint_];
    current_x = x_ - origin_x_;
    current_y = y_ - origin_y_;
    current_z = z_ - origin_z_;
    yaw = yaw_;
  }

  const double dx = target.x - current_x;
  const double dy = target.y - current_y;
  const double dz = target.z - current_z;

  const double distance = std::sqrt(
    dx * dx + dy * dy + dz * dz);

  const double step = speed_ * dt;

  // Turn toward the travel direction first (same idea as the DDMR's ROTATE state).
  double heading_error = 0.0;
  if (std::hypot(dx, dy) > 1e-3) {
    const double desired = std::atan2(dy, dx);
    heading_error = std::atan2(std::sin(desired - yaw), std::cos(desired - yaw));
    const double max_turn = max_yaw_rate_ * dt;
    yaw += std::clamp(heading_error, -max_turn, max_turn);
    yaw = std::atan2(std::sin(yaw), std::cos(yaw));
    heading_error = std::atan2(std::sin(desired - yaw), std::cos(desired - yaw));
  }
  const bool aligned = std::abs(heading_error) < 5.0 * M_PI / 180.0;   // same threshold as DDMR

  const bool arrived = distance <= step || distance <= goal_tolerance_;

  Waypoint next{current_x, current_y, current_z};     // stay put while turning
  if (arrived) {
    next = target;
  } else if (aligned) {
    const double scale = step / distance;
    next = {current_x + dx * scale, current_y + dy * scale, current_z + dz * scale};
  }

  const Waypoint world_pose{origin_x_ + next.x, origin_y_ + next.y, origin_z_ + next.z};

  if (!setPose(world_pose, yaw)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "Failed to move model '%s'.", model_name_.c_str());
    return;
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
  bool executed = gz_node_.Request(
    "/world/" + world_name_ + "/set_pose",
    request,
    100,
    reply,
    result);
  return executed && result && reply.data();
}
