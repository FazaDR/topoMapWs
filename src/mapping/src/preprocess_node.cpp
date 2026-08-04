#include "mapping/preprocess_node.hpp"

#include <functional>

#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

PreprocessNode::PreprocessNode()
: Node("preprocess_node"),
  pose_received_(false)
{
    lidar_offset_ << 0.0f, 0.0f, 0.49f;

    point_sub_ =
        create_subscription<sensor_msgs::msg::PointCloud2>(
            "/points/points",
            10,
            std::bind(
                &PreprocessNode::pointCloudCallback,
                this,
                std::placeholders::_1));

    global_cloud_pub_ =
        create_publisher<sensor_msgs::msg::PointCloud2>(
            "/mapping/global_cloud",
            10);

    gz_node_.Subscribe(
        "/world/winding_room/dynamic_pose/info",
        &PreprocessNode::poseCallback,
        this);

    RCLCPP_INFO(get_logger(), "Preprocess node started.");
}

void PreprocessNode::poseCallback(
    const gz::msgs::Pose_V &msg)
{
    for (int i = 0; i < msg.pose_size(); ++i)
    {
        const auto &pose = msg.pose(i);

        if (pose.name() != "ddmr")
            continue;

        robot_position_ <<
            pose.position().x(),
            pose.position().y(),
            pose.position().z();

        robot_orientation_ =
            Eigen::Quaternionf(
                pose.orientation().w(),
                pose.orientation().x(),
                pose.orientation().y(),
                pose.orientation().z());
        robot_orientation_.normalize();
        pose_received_ = true;

        break;
    }
}

void PreprocessNode::pointCloudCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    if (!pose_received_)
        return;

    auto output = transformCloud(*msg);

    global_cloud_pub_->publish(output);
}

sensor_msgs::msg::PointCloud2
PreprocessNode::transformCloud(
    const sensor_msgs::msg::PointCloud2 &cloud_msg)
{
    pcl::PointCloud<pcl::PointXYZ> input;
    pcl::fromROSMsg(cloud_msg, input);

    pcl::PointCloud<pcl::PointXYZ> output;
    output.reserve(input.size());

    Eigen::Matrix3f R =
        robot_orientation_.toRotationMatrix();

    for (const auto &p : input.points)
    {
        Eigen::Vector3f local(
            p.x,
            p.y,
            p.z);

        Eigen::Vector3f world =
            R * (local + lidar_offset_)
            + robot_position_;

        output.emplace_back(
            world.x(),
            world.y(),
            world.z());
    }

    sensor_msgs::msg::PointCloud2 result;

    pcl::toROSMsg(output, result);

    result.header.frame_id = "map";
    result.header.stamp = cloud_msg.header.stamp;

    return result;
}