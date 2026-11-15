#pragma once

#include <fstream>

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include <Eigen/Core>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "preprocess/msg/processed_scan.hpp"

#include "atc_core/atc.hpp"
#include "atc_core/mlatc.hpp"

class MappingNode : public rclcpp::Node
{
public:
  MappingNode();
  ~MappingNode() override;

private:

  void openMetrics(const std::string & dir);
  void writeMetricsRow(
    uint64_t frame_id, double stamp_s, size_t points, double convert_ms, double process_ms,
    int total_nodes, double heading);
  void saveMap(const std::string & prefix) const;

  std::ofstream metrics_file_;
  bool metrics_enabled_ = true;
  bool save_map_on_shutdown_ = false;
  std::string output_dir_;
  std::string run_prefix_;

  void scanCallback(const preprocess::msg::ProcessedScan::SharedPtr msg);

  // PointCloud2 -> Cloud. Drops non-finite points. Returns false if x/y/z are missing.
  bool convertCloud(const sensor_msgs::msg::PointCloud2 & in, atc_core::Cloud & out) const;

  void publishMarkers(const std_msgs::msg::Header & header);
  void appendLayerMarkers(
    visualization_msgs::msg::MarkerArray & arr,
    const atc_core::TopologicalMap & map,
    int layer_index, bool attributes,
    const std_msgs::msg::Header & header) const;

  // ROS
  rclcpp::Subscription<preprocess::msg::ProcessedScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  // Algorithm (exactly one is non-null)
  std::unique_ptr<atc_core::ATC> atc_;
  std::unique_ptr<atc_core::MLATC> mlatc_;

  // Parameters
  atc_core::UpAxis up_axis_ = atc_core::UpAxis::Z;
  Eigen::Vector3f lidar_offset_ = Eigen::Vector3f::Zero();
  std::string marker_frame_;
  double node_size_ = 0.12;
  double edge_width_ = 0.02;
  double layer_z_offset_ = 0.0;
  double marker_period_ = 0.2;

  // Bookkeeping
  std::chrono::steady_clock::time_point last_marker_wall_{};
  bool have_frame_id_ = false;
  uint64_t last_frame_id_ = 0;
  uint64_t dropped_frames_ = 0;
};