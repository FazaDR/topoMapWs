#pragma once

#include <cstddef>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gz/msgs/pose.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/msgs/boolean.pb.h>
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

  std::string world_name_;
  std::string model_name_;
  double last_sim_time_sec_{-1.0};
  double speed_;
  double goal_tolerance_;
  double x_{0.0};
  double y_{0.0};
  double z_{1.0};
  double origin_x_{0.0};
  double origin_y_{0.0};
  double origin_z_{1.0};
  bool pose_received_{false};
  bool initialized_{false};
  std::vector<Waypoint> waypoints_;
  std::size_t current_waypoint_{0};
  std::mutex mutex_;

  double yaw_{0.0};
  double max_yaw_rate_{1.5};
  bool setPose(const Waypoint &pose, double yaw);
  
  void poseCallback(const gz::msgs::Pose_V &msg);
  void trajectoryCallback(const nav_msgs::msg::Path::SharedPtr msg);
  void controlLoop();
  bool setPose(const Waypoint &pose);
};
