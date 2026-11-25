#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <nav_msgs/msg/path.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gz/msgs/pose_v.pb.h>
#include <gz/transport/Node.hh>

#include "controller/waypoint_plan.hpp"

namespace controller
{

inline double wrapAngle(double a) {return std::atan2(std::sin(a), std::cos(a));}

// Pose in the startup-local frame: (0, 0, 0) is the spawn position, axes are
// parallel to the world axes, yaw is the absolute world yaw.
struct Pose
{
  double x{0.0}, y{0.0}, z{0.0}, yaw{0.0};
};

struct StepContext
{
  double now_s{0.0};     // node clock (sim time when use_sim_time)
  double dt{0.0};        // clamped to 2 * control_period
  Pose pose;             // local frame
  Waypoint target;       // local frame
  Waypoint previous;     // start of the current leg
};

// Shared parts of both controllers: parameters, Gazebo pose / local frame,
// waypoint plan (from YAML or a nav_msgs/Path), the control timer and mission
// bookkeeping. Subclasses only implement the motion law in step().
class ControllerBase : public rclcpp::Node
{
protected:
  ControllerBase(
    const std::string & node_name, const std::string & default_model, bool follow_gz_pose)
  : rclcpp::Node(node_name), follow_gz_pose_(follow_gz_pose)
  {
    world_ = declare_parameter<std::string>("world", "winding_room");
    model_ = declare_parameter<std::string>("model", default_model);
    period_s_ = declare_parameter<double>("control_period", 0.02);
    goal_tolerance_ = declare_parameter<double>("goal_tolerance", 0.05);
    shutdown_on_finish_ = declare_parameter<bool>("shutdown_on_finish", false);
    const std::string traj_topic =
      declare_parameter<std::string>("trajectory_topic", "/" + model_ + "/trajectory");

    if (period_s_ <= 0.0 || goal_tolerance_ <= 0.0) {
      throw std::invalid_argument("control_period and goal_tolerance must be positive");
    }

    plan_.reset(readWaypointParam());
    if (plan_.empty()) {
      RCLCPP_WARN(
        get_logger(), "No 'waypoints' parameter: idle until a Path arrives on %s.",
        traj_topic.c_str());
    } else {
      RCLCPP_INFO(
        get_logger(), "Mission from parameters: %zu waypoints, path length %.2f m.",
        plan_.size(), plan_.totalLength());
    }

    trajectory_sub_ = create_subscription<nav_msgs::msg::Path>(
      traj_topic, rclcpp::QoS(10),
      [this](const nav_msgs::msg::Path::SharedPtr msg) {trajectoryCallback(*msg);});

    // The node clock is sim time when use_sim_time is true.
    control_timer_ = rclcpp::create_timer(
      this, get_clock(),
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(period_s_)),
      [this]() {tick();});

    // Last: the callback runs on a Gazebo thread and needs everything above.
    if (!gz_node_.Subscribe(
        "/world/" + world_ + "/dynamic_pose/info", &ControllerBase::poseCallback, this))
    {
      RCLCPP_ERROR(get_logger(), "Failed to subscribe to Gazebo dynamic_pose/info.");
    }
  }

  // ---- implemented by subclasses ----

  // Called once per tick while a target exists. Return true when the target
  // waypoint is reached; the base then advances the plan.
  virtual bool step(const StepContext & ctx) = 0;
  // The target changed (waypoint reached, or the plan was replaced).
  virtual void onNewTarget() {}
  // The last waypoint was reached.
  virtual void onFinished() {}

  // ---- helpers for subclasses ----

  // Dead-reckoning: overwrite the stored pose (local frame).
  void setLocalState(double x, double y, double z, double yaw)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    x_ = origin_x_ + x;
    y_ = origin_y_ + y;
    z_ = origin_z_ + z;
    yaw_ = yaw;
  }

  Waypoint toWorld(const Pose & local) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return {origin_x_ + local.x, origin_y_ + local.y, origin_z_ + local.z};
  }

  gz::transport::Node gz_node_;
  std::string world_;
  std::string model_;
  double period_s_{0.02};
  double goal_tolerance_{0.05};

private:
  std::vector<Waypoint> readWaypointParam()
  {
    // dynamic_typing so that `[0, 9]` (integers) in YAML does not fail.
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.dynamic_typing = true;
    declare_parameter("waypoints", rclcpp::ParameterValue(std::vector<double>{}), desc);
    const auto p = get_parameter("waypoints");

    std::vector<double> flat;
    switch (p.get_type()) {
      case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY:
        flat = p.as_double_array();
        break;
      case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY:
        for (const auto v : p.as_integer_array()) {
          flat.push_back(static_cast<double>(v));
        }
        break;
      case rclcpp::ParameterType::PARAMETER_NOT_SET:
        break;
      default:
        throw std::invalid_argument("'waypoints' must be a flat numeric array [x0,y0,x1,y1,...]");
    }
    if (flat.size() % 2 != 0) {
      throw std::invalid_argument("'waypoints' needs an even number of values (x,y pairs)");
    }

    std::vector<Waypoint> out;
    out.reserve(flat.size() / 2);
    for (std::size_t i = 0; i < flat.size(); i += 2) {
      out.push_back({flat[i], flat[i + 1], 0.0});
    }
    return out;
  }

  void poseCallback(const gz::msgs::Pose_V & msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (initialized_ && !follow_gz_pose_) {
      return;
    }
    for (int i = 0; i < msg.pose_size(); ++i) {
      const auto & pose = msg.pose(i);
      if (pose.name() != model_) {
        continue;
      }
      const auto & q = pose.orientation();
      x_ = pose.position().x();
      y_ = pose.position().y();
      z_ = pose.position().z();
      yaw_ = std::atan2(
        2.0 * (q.w() * q.z() + q.x() * q.y()),
        1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
      if (!initialized_) {
        origin_x_ = x_;
        origin_y_ = y_;
        origin_z_ = z_;
        initialized_ = true;
        RCLCPP_INFO(get_logger(), "Local frame initialized at (%.2f, %.2f).", x_, y_);
      }
      return;
    }
  }

  void trajectoryCallback(const nav_msgs::msg::Path & msg)
  {
    const auto & frame = msg.header.frame_id;
    if (!frame.empty() && frame != "odom" && frame != "base_link") {
      RCLCPP_ERROR(
        get_logger(), "Ignoring trajectory in frame '%s'; expected 'odom' or 'base_link'.",
        frame.c_str());
      return;
    }
    std::vector<Waypoint> wps;
    wps.reserve(msg.poses.size());
    for (const auto & p : msg.poses) {
      wps.push_back({p.pose.position.x, p.pose.position.y, p.pose.position.z});
    }

    std::size_t n;
    double len;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      plan_.reset(std::move(wps));
      finished_ = false;
      plan_changed_ = true;
      ++plan_gen_;
      n = plan_.size();
      len = plan_.totalLength();
    }
    RCLCPP_INFO(
      get_logger(), "Received trajectory: %zu waypoints, path length %.2f m.", n, len);
  }

  void tick()
  {
    const double now_s = get_clock()->now().seconds();
    if (last_t_ < 0.0) {
      last_t_ = now_s;
      return;
    }
    double dt = now_s - last_t_;
    last_t_ = now_s;
    if (dt <= 0.0) {
      return;                              // sim time has not advanced
    }
    dt = std::min(dt, 2.0 * period_s_);    // a late tick must not become a jump

    StepContext ctx;
    ctx.now_s = now_s;
    ctx.dt = dt;
    bool has_target = false, plan_changed = false, just_finished = false;
    std::uint64_t gen = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!initialized_) {
        return;
      }
      plan_changed = plan_changed_;
      plan_changed_ = false;
      gen = plan_gen_;
      ctx.pose = {x_ - origin_x_, y_ - origin_y_, z_ - origin_z_, yaw_};
      if (!plan_.done()) {
        ctx.target = plan_.current();
        ctx.previous = plan_.previous();
        has_target = true;
      } else if (!plan_.empty() && !finished_) {
        finished_ = true;
        just_finished = true;
      }
    }

    if (plan_changed) {
      onNewTarget();
    }
    if (just_finished) {
      RCLCPP_INFO(get_logger(), "Mission complete.");
      onFinished();
      if (shutdown_on_finish_) {
        rclcpp::shutdown();
      }
      return;
    }
    if (!has_target) {
      return;
    }

    if (step(ctx)) {
      std::size_t reached;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (gen != plan_gen_) {
          return;                          // plan was replaced during the step
        }
        reached = plan_.index();
        plan_.advance();
      }
      RCLCPP_INFO(get_logger(), "Reached waypoint %zu.", reached);
      onNewTarget();
    }
  }

  const bool follow_gz_pose_;
  bool shutdown_on_finish_{false};

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr trajectory_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  mutable std::mutex mutex_;
  // World-frame pose and the origin captured at startup.
  double x_{0.0}, y_{0.0}, z_{0.0}, yaw_{0.0};
  double origin_x_{0.0}, origin_y_{0.0}, origin_z_{0.0};
  bool initialized_{false};

  WaypointPlan plan_;
  bool finished_{false};
  bool plan_changed_{false};
  std::uint64_t plan_gen_{0};

  double last_t_{-1.0};
};

}  // namespace controller
