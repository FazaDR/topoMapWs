#pragma once

#include <chrono>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <random>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <gz/msgs/pose_v.pb.h>
#include <gz/transport/Node.hh>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "preprocess/msg/processed_scan.hpp"

class PreprocessNode : public rclcpp::Node
{
public:
  PreprocessNode();

private:
  // One Gazebo pose sample, stamped with Gazebo's sim time.
  struct PoseSample
  {
    double t{0.0};
    Eigen::Vector3f p{Eigen::Vector3f::Zero()};
    Eigen::Quaternionf q{Eigen::Quaternionf::Identity()};
  };

  // A scan that is due (stamp = exact multiple of the scan period) and waits for its pose.
  struct PendingScan
  {
    double stamp{0.0};
    std::chrono::steady_clock::time_point created;
  };

  // Oriented box from the world file. Rt = transpose of the box rotation (world -> box).
  struct Box
  {
    std::string name;
    Eigen::Vector3f c{Eigen::Vector3f::Zero()};
    Eigen::Matrix3f Rt{Eigen::Matrix3f::Identity()};
    Eigen::Vector3f h{Eigen::Vector3f::Zero()};
  };

  // ROS
  rclcpp::Publisher<preprocess::msg::ProcessedScan>::SharedPtr processed_scan_pub_;
  rclcpp::TimerBase::SharedPtr process_timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Gazebo
  gz::transport::Node gz_node_;

  // Parameters
  std::string robot_name_;
  Eigen::Vector3f lidar_offset_;
  double pose_buffer_s_{3.0};
  double max_wait_s_{0.25};
  double max_interp_gap_s_{0.1};
  double tf_period_s_{0.01};
  double last_tf_t_{-1e9};          // Gazebo thread only
  bool hold_pose_{false};

  // Virtual lidar
  std::vector<Box> boxes_;
  std::vector<Eigen::Vector3f> ray_dirs_;   // unit vectors in the sensor frame
  double scan_period_s_{0.2};
  float min_range_{0.3f};
  float max_range_{20.0f};
  double range_noise_std_{0.0};
  std::string remove_box_;                  // box name to drop after remove_after_s_ (sim time)
  double remove_after_s_{30.0};
  std::mt19937 rng_;

  // Scan scheduling (guarded by mutex_)
  int64_t next_scan_k_{-1};
  double last_now_{0.0};

  // Buffers (guarded by mutex_)
  std::mutex mutex_;
  std::deque<PoseSample> pose_buf_;
  std::deque<PendingScan> pending_;

  // Stats and ids (timer thread only, except n_dropped_ under mutex_)
  uint64_t source_frame_id_{0};
  uint64_t n_processed_{0};
  uint64_t n_held_{0};
  uint64_t n_dropped_{0};

  // Callbacks
  void poseCallback(const gz::msgs::Pose_V &msg);
  void processPending();

  // Helpers
  void loadBoxes(const std::string &path);
  void buildRays(int h_samples, int v_samples, double v_half_fov_rad);
  void scheduleScans();                                            // call with mutex_ held
  bool lookupPose(double ts, PoseSample &out, bool &held) const;   // call with mutex_ held
  void publishTF(const PoseSample &s);

  sensor_msgs::msg::PointCloud2 castScan(
    double stamp, const Eigen::Vector3f &pos, const Eigen::Quaternionf &q);
};