#include "mapping/topology_node.hpp"

#include <chrono>
#include <cmath>
#include <stdexcept>
#include <sensor_msgs/point_cloud2_iterator.hpp>

TopologyNode::TopologyNode()
: Node("topology_node"),
  atcdt_(0.9f, 4000),
sample_period_ms_(declare_parameter("sample_period_ms", 100.0)),
metrics_file_(declare_parameter("metrics_csv", "atcdt_metrics.csv"))
{
  if (sample_period_ms_ <= 0.0)
      throw std::invalid_argument("sample_period_ms must be positive");
    if (!metrics_file_.is_open())
    {
        throw std::runtime_error(
            "ATC-DT could not open metrics CSV: " +
            get_parameter("metrics_csv").as_string());
    }
    metrics_file_ << "frame,source_frame_id,stamp_sec,stamp_nanosec,processing_ms,sample_period_ms,skipped_frames,processed_frames,total_seen_frames,processed_ratio,node_count\n";

    point_sub_ =
        create_subscription<preprocess::msg::ProcessedScan>(
            "/mapping/processed_scan",
            rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
            std::bind(
                &TopologyNode::pointCloudCallback,
                this,
                std::placeholders::_1));

    topology_pub_ =
    create_publisher<
        visualization_msgs::msg::MarkerArray>(
            "/topology",
            1);
}

void TopologyNode::pointCloudCallback(
    const preprocess::msg::ProcessedScan::SharedPtr msg)
{
    ++received_frames_;
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "ATC-DT received source_frame_id=%lu with %u points",
        msg->source_frame_id, msg->cloud.width * msg->cloud.height);
    const int64_t stamp_ns =
        static_cast<int64_t>(msg->header.stamp.sec) * 1000000000LL +
        static_cast<int64_t>(msg->header.stamp.nanosec);
    if (previous_stamp_ns_ >= 0) {
        const double delta_ms =
            static_cast<double>(stamp_ns - previous_stamp_ns_) / 1e6;
        const auto expected_frames = static_cast<int64_t>(
            std::llround(delta_ms / sample_period_ms_));
        if (expected_frames > 1)
            skipped_frames_ += static_cast<uint64_t>(expected_frames - 1);
    }
    previous_stamp_ns_ = stamp_ns;
    const auto start = std::chrono::steady_clock::now();
    auto points = convertPointCloud(msg->cloud);

    atcdt_.processFrame(points);
    publishTopology();
    ++processed_frames_;

    RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "Nodes: %d  Edges: %d",
        atcdt_.map.node_count,
        atcdt_.map.numEdges());
    RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "ATC-DT frames: received=%lu processed=%lu",
        received_frames_,
        processed_frames_);

    const auto elapsed = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start).count();
    const auto total_seen_frames = processed_frames_ + skipped_frames_;
    metrics_file_ << processed_frames_ << ','
                  << msg->source_frame_id << ','
                  << msg->header.stamp.sec << ','
                  << msg->header.stamp.nanosec << ','
                  << elapsed << ','
                  << sample_period_ms_ << ','
                  << skipped_frames_ << ','
                  << processed_frames_ << ','
                  << total_seen_frames << ','
                  << (total_seen_frames == 0 ? 0.0 : static_cast<double>(processed_frames_) / static_cast<double>(total_seen_frames)) << ','
                  << atcdt_.map.node_count << '\n';
    metrics_file_.flush();
}

std::vector<Eigen::Vector3f> TopologyNode::convertPointCloud(
    const sensor_msgs::msg::PointCloud2 &cloud) const
{
    std::vector<Eigen::Vector3f> points;
    size_t invalid_points = 0;

    sensor_msgs::PointCloud2ConstIterator<float>
        iter_x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float>
        iter_y(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float>
        iter_z(cloud, "z");

    for (; iter_x != iter_x.end();
         ++iter_x,
         ++iter_y,
         ++iter_z)
    {
        const Eigen::Vector3f point(*iter_x, *iter_y, *iter_z);
        if (point.allFinite())
        {
            points.push_back(point);
        }
        else
        {
            ++invalid_points;
        }
    }

    if (invalid_points > 0)
    {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            5000,
            "ATC-DT dropped %zu non-finite input points",
            invalid_points);
    }

    return points;
}

void TopologyNode::publishTopology()
{
    visualization_msgs::msg::MarkerArray array;

    //
    // Nodes
    //

    visualization_msgs::msg::Marker nodes;

    nodes.header.frame_id = "map";
    nodes.header.stamp = now();

    nodes.ns = "topology";
    nodes.id = 0;

    nodes.type =
        visualization_msgs::msg::Marker::SPHERE_LIST;

    nodes.action =
        visualization_msgs::msg::Marker::ADD;

    nodes.scale.x = 0.08;
    nodes.scale.y = 0.08;
    nodes.scale.z = 0.08;

    nodes.color.r = 1.0;
    nodes.color.g = 0.0;
    nodes.color.b = 0.0;
    nodes.color.a = 1.0;

    for (const auto &p : atcdt_.map.positions)
    {
        if (!p.allFinite())
        {
            continue;
        }
        geometry_msgs::msg::Point point;

        point.x = p.x();
        point.y = p.y();
        point.z = p.z();

        nodes.points.push_back(point);
    }

    array.markers.push_back(nodes);

    //
    // Edges
    //

    visualization_msgs::msg::Marker edges;

    edges.header = nodes.header;

    edges.ns = "topology";
    edges.id = 1;

    edges.type =
        visualization_msgs::msg::Marker::LINE_LIST;

    edges.action =
        visualization_msgs::msg::Marker::ADD;

    edges.scale.x = 0.02;

    edges.color.r = 0.0;
    edges.color.g = 1.0;
    edges.color.b = 0.0;
    edges.color.a = 1.0;

    for (const auto &[a, b] : atcdt_.map.edges())
    {
        geometry_msgs::msg::Point p1;
        geometry_msgs::msg::Point p2;

        const auto &v1 =
            atcdt_.map.positions[a];

        const auto &v2 =
            atcdt_.map.positions[b];
        if (!v1.allFinite() || !v2.allFinite())
        {
            continue;
        }

        p1.x = v1.x();
        p1.y = v1.y();
        p1.z = v1.z();

        p2.x = v2.x();
        p2.y = v2.y();
        p2.z = v2.z();

        edges.points.push_back(p1);
        edges.points.push_back(p2);
    }

    array.markers.push_back(edges);

    topology_pub_->publish(array);
}