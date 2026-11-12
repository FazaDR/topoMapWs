#pragma once

#include "mapping/mlatc.hpp"
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include "preprocess/msg/processed_scan.hpp"
#include <visualization_msgs/msg/marker_array.hpp>

class MlatcNode : public rclcpp::Node
{
public:
  MlatcNode();

private:
  void callback(const preprocess::msg::ProcessedScan::SharedPtr msg);
  void publish();
  mlatc::MLATC mapper_;
  double layer_z_offset_;
  bool show_parent_child_;
  uint64_t received_frames_{0};
  uint64_t processed_frames_{0};
  uint64_t skipped_frames_{0};
  double sample_period_ms_{100.0};
  int64_t previous_stamp_ns_{-1};
  std::ofstream metrics_file_;
  rclcpp::Subscription<preprocess::msg::ProcessedScan>::SharedPtr subscription_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
};
