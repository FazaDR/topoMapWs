#pragma once

#include <cstddef>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/pose.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/transport/Node.hh>

class IdealizedLidarControllerNode : public rclcpp::Node
{
public:
  IdealizedLidarControllerNode();

private:
  struct Waypoint
  {
    double x;
    double y;
    double z;
  };

  gz::transport::Node gz_node_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr trajectory_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  // Parameters
  std::string world_name_;
  std::string model_name_;
  double speed_{0.5};               // [m/s]
  double goal_tolerance_{0.05};     // [m]
  double max_yaw_rate_{0.5};        // [rad/s] (smooth mode only)
  double control_period_s_{0.02};   // [s] sim time
  double align_threshold_{0.035};   // [rad]
  bool snap_turn_{true};            // true: pause, jump yaw, pause. false: rate-limited turn
  double dwell_s_{0.5};             // [s] pause before and after a snap turn
  bool shutdown_on_finish_{false};

  // State (dead-reckoned; Gazebo's pose is only used for initialization)
  double last_sim_time_sec_{-1.0};
  double x_{0.0};
  double y_{0.0};
  double z_{0.0};
  double yaw_{0.0};
  double origin_x_{0.0};
  double origin_y_{0.0};
  double origin_z_{0.0};
  bool initialized_{false};
  bool finished_{false};

  // Touched only from the timer thread
  bool snap_pending_{false};
  double hold_until_{-1.0};

  std::vector<Waypoint> waypoints_;
  std::size_t current_waypoint_{0};
  std::mutex mutex_;

  void poseCallback(const gz::msgs::Pose_V &msg);
  void trajectoryCallback(const nav_msgs::msg::Path::SharedPtr msg);
  void controlLoop();
  bool setPose(const Waypoint &pose, double yaw);
};