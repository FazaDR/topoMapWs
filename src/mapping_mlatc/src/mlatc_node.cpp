#include "mapping_mlatc/mlatc_node.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>

MlatcNode::MlatcNode()
: Node("mlatc_node"),
  mapper_(declare_parameter("base_vigilance", 0.5), declare_parameter("alpha", 4.0),
          declare_parameter("lambda_points", 4000))
{
  subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "/mapping/global_cloud", rclcpp::SensorDataQoS(),
    std::bind(&MlatcNode::callback, this, std::placeholders::_1));
  publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>("/mlatc/topology", 1);
}

void MlatcNode::callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  std::vector<Eigen::Vector3f> points;
  sensor_msgs::PointCloud2ConstIterator<float> x(*msg, "x"), y(*msg, "y"), z(*msg, "z");
  for (; x != x.end(); ++x, ++y, ++z) points.emplace_back(*x, *y, *z);
  mapper_.processFrame(points);
  publish();
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Layers: %zu, base nodes: %d",
                       mapper_.layers().size(), mapper_.layers().front().nodeCount());
}

void MlatcNode::publish()
{
  visualization_msgs::msg::MarkerArray output;
  int id = 0;
  for (size_t layer_index = 0; layer_index < mapper_.layers().size(); ++layer_index) {
    const auto & layer = mapper_.layers()[layer_index];
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = now();
    marker.ns = "mlatc";
    marker.id = id++;
    marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = marker.scale.y = marker.scale.z = layer_index == 0 ? 0.08 : 0.16;
    marker.color.r = 1.0;
    marker.color.g = layer_index == 0 ? 0.0 : 1.0;
    marker.color.a = 1.0;
    for (const int node_id : layer.nodeIds()) {
      geometry_msgs::msg::Point point;
      const auto & p = layer.node(node_id).position;
      point.x = p.x(); point.y = p.y(); point.z = p.z();
      marker.points.push_back(point);
    }
    output.markers.push_back(marker);
  }
  publisher_->publish(output);
}
