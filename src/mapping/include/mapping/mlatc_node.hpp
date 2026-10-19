#pragma once

#include "mapping/mlatc.hpp"
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

class MlatcNode : public rclcpp::Node
{
public:
  MlatcNode();

private:
  void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void publish();
  mlatc::MLATC mapper_;
  double layer_z_offset_;
  bool show_parent_child_;
  uint64_t received_frames_{0};
  uint64_t processed_frames_{0};
  std::ofstream metrics_file_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
};
