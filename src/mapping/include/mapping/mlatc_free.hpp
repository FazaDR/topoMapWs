#pragma once

#include <cmath>
#include <cstdint>
#include <fstream>
#include <vector>

#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "mapping/mlatc.hpp"
#include "preprocess/msg/processed_scan.hpp"

class MlatcFreeNode : public rclcpp::Node
{
public:
    MlatcFreeNode();

private:
    void callback(const preprocess::msg::ProcessedScan::SharedPtr msg);
    void publish();
    std::vector<Eigen::Vector3f> convertPointCloud(const sensor_msgs::msg::PointCloud2 &cloud) const;
    std::vector<Eigen::Vector3f> findProximityPoints(
        const std::vector<Eigen::Vector3f> &cloud,
        const Eigen::Vector3f &robot_position,
        float heading) const;
    bool nodeInFreeArea(
        const Eigen::Vector3f &node_position,
        const std::vector<Eigen::Vector3f> &proximity_points,
        const Eigen::Vector3f &robot_position,
        float heading,
        float margin = 0.1f) const;
    void deleteFreeAreaNodes(
        const std::vector<Eigen::Vector3f> &point_cloud,
        const Eigen::Vector3f &robot_position,
        float heading = 0.0f);

    mlatc::MLATC mapper_;
    double layer_z_offset_{4.0};
    bool show_parent_child_{false};
    float free_area_distance_{0.5f};
    int free_area_sectors_{24};
    float free_area_fov_{M_PI};
    float horizontal_band_{0.5f};
    float free_area_margin_{0.1f};
    uint64_t received_frames_{0};
    uint64_t processed_frames_{0};
    uint64_t skipped_frames_{0};
    double sample_period_ms_{100.0};
    int64_t previous_stamp_ns_{-1};
    std::ofstream metrics_file_;
    rclcpp::Subscription<preprocess::msg::ProcessedScan>::SharedPtr subscription_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
};
