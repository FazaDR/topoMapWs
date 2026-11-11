#include "preprocess/preprocess_node.hpp"

#include <functional>

#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

PreprocessNode::PreprocessNode()
: Node("preprocess_node"),
  pose_received_(false)
{
    robot_name_ = declare_parameter<std::string>("robot_name", "ddmr");
    const auto world = declare_parameter<std::string>("world", "winding_room");
    const double offset_z = declare_parameter<double>("lidar_offset_z", 0.49);

    tf_broadcaster_ =
        std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    lidar_offset_ << 0.0f, 0.0f, static_cast<float>(offset_z);

    point_sub_ =
        create_subscription<sensor_msgs::msg::PointCloud2>(
            "/points/points",
            rclcpp::SensorDataQoS(),
            std::bind(
                &PreprocessNode::pointCloudCallback,
                this,
                std::placeholders::_1));

    processed_scan_pub_ = create_publisher<preprocess::msg::ProcessedScan>(
        "/mapping/processed_scan", rclcpp::SensorDataQoS());

    gz_node_.Subscribe(
        "/world/" + world + "/dynamic_pose/info",
        &PreprocessNode::poseCallback,
        this);

    RCLCPP_INFO(get_logger(), "Preprocess node started for robot '%s'.", robot_name_.c_str());
}

void PreprocessNode::publishTF(const Eigen::Vector3f &pos, const Eigen::Quaternionf &q)
{
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = this->now();
    tf.header.frame_id = "map";
    tf.child_frame_id = "base_link";
    tf.transform.translation.x = pos.x();
    tf.transform.translation.y = pos.y();
    tf.transform.translation.z = pos.z();
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(tf);
}

void PreprocessNode::poseCallback(const gz::msgs::Pose_V &msg)
{
    for (int i = 0; i < msg.pose_size(); ++i)
    {
        const auto &pose = msg.pose(i);
        if (pose.name() != robot_name_)
            continue;

        const Eigen::Vector3f pos(
            pose.position().x(), pose.position().y(), pose.position().z());
        Eigen::Quaternionf q(
            pose.orientation().w(), pose.orientation().x(),
            pose.orientation().y(), pose.orientation().z());
        q.normalize();

        {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            robot_position_ = pos;
            robot_orientation_ = q;
            pose_received_ = true;
        }
        publishTF(pos, q);
        break;
    }
}

void PreprocessNode::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    Eigen::Vector3f pos;
    Eigen::Quaternionf q;
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        if (!pose_received_)
            return;
        pos = robot_position_;
        q = robot_orientation_;
    }

    auto output = transformCloud(*msg, pos, q);

    const Eigen::Vector3f sensor = pos + q * lidar_offset_;   // scan origin in map
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = msg->header.stamp;                    // same stamp as the cloud
    pose.pose.position.x = sensor.x();
    pose.pose.position.y = sensor.y();
    pose.pose.position.z = sensor.z();
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();

    preprocess::msg::ProcessedScan processed;
    processed.header = output.header;
    processed.source_frame_id = source_frame_id_++;
    processed.cloud = output;
    processed.robot_pose = pose;
    processed_scan_pub_->publish(processed);
}

sensor_msgs::msg::PointCloud2
PreprocessNode::transformCloud(
    const sensor_msgs::msg::PointCloud2 &cloud_msg,
    const Eigen::Vector3f &pos,
    const Eigen::Quaternionf &q)
{
    pcl::PointCloud<pcl::PointXYZ> input;
    pcl::fromROSMsg(cloud_msg, input);

    pcl::PointCloud<pcl::PointXYZ> output;
    output.reserve(input.size());

    const Eigen::Matrix3f R = q.toRotationMatrix();

    for (const auto &p : input.points)
    {
        Eigen::Vector3f local(
            p.x,
            p.y,
            p.z);

        Eigen::Vector3f world =
            R * (local + lidar_offset_) + pos;

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