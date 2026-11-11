#pragma once

#include <memory>
#include <fstream>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "preprocess/msg/processed_scan.hpp"
#include <visualization_msgs/msg/marker_array.hpp>

#include <Eigen/Core>

#include "mapping/atcdt.hpp"

class TopologyNode : public rclcpp::Node
{
public:

    TopologyNode();

private:

    //---------------------------------------
    // ROS
    //---------------------------------------

    rclcpp::Subscription<preprocess::msg::ProcessedScan>::SharedPtr point_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr topology_pub_;

    //---------------------------------------
    // ATC-DT
    //---------------------------------------

    ATCDT atcdt_;
    uint64_t received_frames_{0};
    uint64_t processed_frames_{0};
    uint64_t skipped_frames_{0};
    double sample_period_ms_{100.0};
    int64_t previous_stamp_ns_{-1};
    std::ofstream metrics_file_;

    //---------------------------------------
    // Callback
    //---------------------------------------

    void pointCloudCallback(const preprocess::msg::ProcessedScan::SharedPtr msg);

    //---------------------------------------
    // Helpers
    //---------------------------------------

    std::vector<Eigen::Vector3f>convertPointCloud(const sensor_msgs::msg::PointCloud2 &cloud) const;
    
    void publishTopology();
};