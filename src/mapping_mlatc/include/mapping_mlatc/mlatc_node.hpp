#pragma once

#include "mapping_mlatc/mlatc.hpp"
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
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
};
