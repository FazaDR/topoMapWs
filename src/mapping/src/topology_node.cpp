#include "mapping/topology_node.hpp"

#include <sensor_msgs/point_cloud2_iterator.hpp>

TopologyNode::TopologyNode()
: Node("topology_node"),
  atcdt_(1.0f, 5000)
{
    point_sub_ =
        create_subscription<sensor_msgs::msg::PointCloud2>(
            "/mapping/global_cloud",
            rclcpp::SensorDataQoS(),
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
    const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    auto points =
        convertPointCloud(*msg);

    atcdt_.processFrame(points);
    publishTopology();

    RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "Nodes: %d  Edges: %d",
        atcdt_.map.node_count,
        atcdt_.map.numEdges());
}

std::vector<Eigen::Vector3f> TopologyNode::convertPointCloud(
    const sensor_msgs::msg::PointCloud2 &cloud) const
{
    std::vector<Eigen::Vector3f> points;

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
        points.emplace_back(
            *iter_x,
            *iter_y,
            *iter_z);
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