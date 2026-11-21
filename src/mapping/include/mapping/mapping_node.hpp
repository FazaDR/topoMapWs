#pragma once

#include <fstream>

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "preprocess/msg/processed_scan.hpp"

#include "atc_core/atc.hpp"
#include "atc_core/free_area.hpp"
#include "atc_core/mlatc.hpp"

class MappingNode : public rclcpp::Node
{
public:
  MappingNode();
  ~MappingNode() override;

  // 0 = ok, 1 = gaps in source_frame_id, 2 = bag open error, 3 = no messages in bag.
  int exit_code() const {return exit_code_;}

private:
  enum class LayerMode { Layer1, All, Hierarchy };
  enum class FreeViz { Off, Nodes, Area, Both };

  struct FreeAreaViz
  {
    bool valid = false;
    Eigen::Vector3f sensor = Eigen::Vector3f::Zero();
    atc_core::Proximity prox;
    std::vector<uint8_t> node_flags;
  };

  // ---------------- metrics ----------------
  void openMetrics(const std::string & dir_param, const std::string & label, const std::string & meta);
  void writeMetricsRow(double stamp_s, size_t points, double total_ms, int total_nodes);
  void saveMap(const std::string & prefix) const;

  std::ofstream metrics_file_;
  bool metrics_enabled_ = true;
  bool save_map_on_shutdown_ = false;
  std::string output_dir_;
  std::string run_prefix_;

  // ---------------- processing ----------------
  void scanCallback(const preprocess::msg::ProcessedScan::SharedPtr msg);
  void processScan(const preprocess::msg::ProcessedScan & msg);

  bool convertCloud(const sensor_msgs::msg::PointCloud2 & in, atc_core::Cloud & out) const;

  // ---------------- replay ----------------
  void runReplay();

  std::string replay_bag_;
  std::string replay_storage_;
  double replay_rate_ = 0.0;
  rclcpp::TimerBase::SharedPtr replay_timer_;
  int exit_code_ = 0;
  uint64_t frames_processed_ = 0;

  // ---------------- markers ----------------
  void publishMarkers(const std_msgs::msg::Header & header, const FreeAreaViz & free_viz);

  void appendLayerMarkers(
    visualization_msgs::msg::MarkerArray & arr,
    const atc_core::TopologicalMap & map,
    int layer_index, bool attributes,
    const std_msgs::msg::Header & header,
    const std::vector<uint8_t> * free_flags) const;

  void appendHierarchyMarkers(
    visualization_msgs::msg::MarkerArray & arr,
    const std_msgs::msg::Header & header) const;

  void appendFreeAreaMarkers(
    visualization_msgs::msg::MarkerArray & arr,
    const FreeAreaViz & free_viz,
    const std_msgs::msg::Header & header) const;

  // ROS
  rclcpp::Subscription<preprocess::msg::ProcessedScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};   // log throttling that ignores sim time

  // Algorithm (exactly one is non-null)
  std::unique_ptr<atc_core::ATC> atc_;
  std::unique_ptr<atc_core::MLATC> mlatc_;
  std::unique_ptr<atc_core::FreeAreaDetector> viz_free_;

  // Parameters
  std::string input_topic_;
  atc_core::UpAxis up_axis_ = atc_core::UpAxis::Z;
  Eigen::Vector3f lidar_offset_ = Eigen::Vector3f::Zero();
  std::string marker_frame_;
  double node_size_ = 0.12;
  double edge_width_ = 0.02;
  double layer_z_offset_ = 3.0;
  double marker_period_ = 0.2;
  LayerMode layer_mode_ = LayerMode::All;
  FreeViz free_viz_mode_ = FreeViz::Both;

  // Bookkeeping
  std::chrono::steady_clock::time_point last_marker_wall_{};
  bool have_frame_id_ = false;
  uint64_t last_frame_id_ = 0;
  uint64_t dropped_frames_ = 0;
};