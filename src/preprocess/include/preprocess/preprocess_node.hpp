#pragma once

#include <chrono>
#include <deque>
#include <memory>
#include <mutex>
#include <string>

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

  // A cloud waiting for a pose sample at or after its stamp.
  struct PendingScan
  {
    sensor_msgs::msg::PointCloud2::SharedPtr msg;
    double stamp{0.0};
    std::chrono::steady_clock::time_point arrived;
  };

  // ROS
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_sub_;
  rclcpp::Publisher<preprocess::msg::ProcessedScan>::SharedPtr processed_scan_pub_;
  rclcpp::TimerBase::SharedPtr process_timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Gazebo
  gz::transport::Node gz_node_;

  // Parameters
  std::string robot_name_;
  Eigen::Vector3f lidar_offset_;
  double pose_buffer_s_{3.0};       // how much pose history to keep [sim s]
  double max_wait_s_{0.25};         // wall-time limit to wait for a newer pose
  double max_interp_gap_s_{0.1};    // do not interpolate across bigger gaps [sim s]
  double tf_period_s_{0.01};        // publish TF at most every this many sim s (100 Hz)
  double last_tf_t_{-1e9};          // sim time of the last published TF (Gazebo thread only)
  bool hold_pose_{false};           // staircase pose (set_pose): use latest sample, no interpolation

  // Buffers (guarded by mutex_)
  std::mutex mutex_;
  std::deque<PoseSample> pose_buf_;
  std::deque<PendingScan> pending_;

  // Stats and ids (only touched from the timer thread)
  uint64_t source_frame_id_{0};
  uint64_t n_processed_{0};
  uint64_t n_held_{0};      // processed with a pose older than the scan (timeout)
  uint64_t n_dropped_{0};   // no pose available for the scan

  // Callbacks
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void poseCallback(const gz::msgs::Pose_V &msg);
  void processPending();

  // Helpers
  bool lookupPose(double ts, PoseSample &out, bool &held) const;   // call with mutex_ held
  void publishTF(const PoseSample &s);

  sensor_msgs::msg::PointCloud2 transformCloud(
    const sensor_msgs::msg::PointCloud2 &cloud,
    const Eigen::Vector3f &pos,
    const Eigen::Quaternionf &q);
};