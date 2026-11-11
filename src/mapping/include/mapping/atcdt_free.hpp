#pragma once

#include <cmath>
#include <fstream>
#include <memory>
#include <vector>

#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>
#include "preprocess/msg/processed_scan.hpp"
#include <visualization_msgs/msg/marker_array.hpp>

#include "mapping/atcdt.hpp"

#include <cstdint>
#include <map>
#include <geometry_msgs/msg/pose_stamped.hpp>


class AtcdtFreeNode : public rclcpp::Node
{
public:
    AtcdtFreeNode();

private:

    struct RobotPose
    {
        Eigen::Vector3f position;   // scan origin, map frame
        float heading;              // ground-truth yaw (rad)
    };
    void pointCloudCallback(const preprocess::msg::ProcessedScan::SharedPtr msg);
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
        float heading);
    void publishTopology();

    ATCDT atcdt_;
    float free_area_distance_{0.9f};
    int free_area_sectors_{24};
    float free_area_fov_{static_cast<float>(M_PI)};   // 180 deg, as in the Python main
    float horizontal_band_{0.15f};
    float free_area_margin_{0.1f};

    uint64_t received_frames_{0};
    uint64_t processed_frames_{0};
    uint64_t skipped_frames_{0};
    double sample_period_ms_{100.0};
    int64_t previous_stamp_ns_{-1};
    std::ofstream metrics_file_;

    rclcpp::Subscription<preprocess::msg::ProcessedScan>::SharedPtr point_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr topology_pub_;
};