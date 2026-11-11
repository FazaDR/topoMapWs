#include "mapping/mlatc_node.hpp"
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <sensor_msgs/point_cloud2_iterator.hpp>

MlatcNode::MlatcNode()
: Node("mlatc_node"),
  mapper_(declare_parameter("base_vigilance", 0.5), declare_parameter("alpha", 4.0),
          declare_parameter("lambda_points", 4000)),
  layer_z_offset_(declare_parameter("layer_z_offset", 4.0)),
  show_parent_child_(declare_parameter("show_parent_child", false)),
  sample_period_ms_(declare_parameter("sample_period_ms", 100.0)),
  metrics_file_(declare_parameter("metrics_csv", "mlatc_metrics.csv"))
{
  if (sample_period_ms_ <= 0.0)
    throw std::invalid_argument("sample_period_ms must be positive");

  if (!metrics_file_.is_open()) {
    throw std::runtime_error(
      "MLATC could not open metrics CSV: " +
      get_parameter("metrics_csv").as_string());
  }
  metrics_file_ << "frame,source_frame_id,stamp_sec,stamp_nanosec,processing_ms,sample_period_ms,skipped_frames,processed_frames,total_seen_frames,processed_ratio,layer1_node_count\n";

  subscription_ = create_subscription<preprocess::msg::ProcessedScan>(
    "/mapping/processed_scan", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
    std::bind(&MlatcNode::callback, this, std::placeholders::_1));
  publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>("/mlatc/topology", 1);
}

void MlatcNode::callback(const preprocess::msg::ProcessedScan::SharedPtr msg)
{
  ++received_frames_;
  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 1000,
    "MLATC received source_frame_id=%lu with %u points",
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
  std::vector<Eigen::Vector3f> points;
  size_t invalid_points = 0;
  sensor_msgs::PointCloud2ConstIterator<float> x(msg->cloud, "x"), y(msg->cloud, "y"), z(msg->cloud, "z");
  for (; x != x.end(); ++x, ++y, ++z) {
    const Eigen::Vector3f point(*x, *y, *z);
    if (point.allFinite()) {
      points.push_back(point);
    } else {
      ++invalid_points;
    }
  }
  if (invalid_points > 0) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "MLATC dropped %zu non-finite input points", invalid_points);
  }
  mapper_.processFrame(points);
  publish();
  ++processed_frames_;
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Layers: %zu, base nodes: %d",
                       mapper_.layers().size(), mapper_.layers().front().nodeCount());
  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 5000,
    "MLATC frames: received=%lu processed=%lu",
    received_frames_, processed_frames_);

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
                << mapper_.layers().front().nodeCount() << '\n';
  metrics_file_.flush();
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

    // Use the golden-ratio hue step so adjacent layers remain distinct even
    // when the hierarchy grows beyond a fixed color palette.
    const float hue = std::fmod(static_cast<float>(layer_index) * 0.618034F, 1.0F);
    const float scaled_hue = hue * 6.0F;
    const int sector = static_cast<int>(scaled_hue);
    const float fraction = scaled_hue - static_cast<float>(sector);
    const float q = 1.0F - 0.9F * fraction;
    const float t = 1.0F - 0.9F * (1.0F - fraction);
    switch (sector % 6) {
      case 0: marker.color.r = 1.0F; marker.color.g = t; marker.color.b = 0.1F; break;
      case 1: marker.color.r = q; marker.color.g = 1.0F; marker.color.b = 0.1F; break;
      case 2: marker.color.r = 0.1F; marker.color.g = 1.0F; marker.color.b = t; break;
      case 3: marker.color.r = 0.1F; marker.color.g = q; marker.color.b = 1.0F; break;
      case 4: marker.color.r = t; marker.color.g = 0.1F; marker.color.b = 1.0F; break;
      default: marker.color.r = 1.0F; marker.color.g = 0.1F; marker.color.b = q; break;
    }
    marker.color.a = 1.0;
    for (const int node_id : layer.nodeIds()) {
      const auto & p = layer.node(node_id).position;
      if (!p.allFinite()) continue;
      geometry_msgs::msg::Point point;
      point.x = p.x();
      point.y = p.y();
      point.z = p.z() + static_cast<float>(layer_index) * layer_z_offset_;
      marker.points.push_back(point);
    }
    output.markers.push_back(marker);

    visualization_msgs::msg::Marker edges;
    edges.header = marker.header;
    edges.ns = "mlatc_edges";
    edges.id = id++;
    edges.type = visualization_msgs::msg::Marker::LINE_LIST;
    edges.action = visualization_msgs::msg::Marker::ADD;
    edges.scale.x = 0.025;
    edges.color.r = marker.color.r;
    edges.color.g = marker.color.g;
    edges.color.b = marker.color.b;
    edges.color.a = 0.85;
    for (const auto & [first, second] : layer.edges()) {
      const auto & p1 = layer.node(first).position;
      const auto & p2 = layer.node(second).position;
      if (!p1.allFinite() || !p2.allFinite()) continue;
      geometry_msgs::msg::Point point1, point2;
      point1.x = p1.x(); point1.y = p1.y();
      point1.z = p1.z() + static_cast<float>(layer_index) * layer_z_offset_;
      point2.x = p2.x(); point2.y = p2.y();
      point2.z = p2.z() + static_cast<float>(layer_index) * layer_z_offset_;
      edges.points.push_back(point1);
      edges.points.push_back(point2);
    }
    output.markers.push_back(edges);
  }

  if (show_parent_child_) {
    visualization_msgs::msg::Marker hierarchy;
    hierarchy.header.frame_id = "map";
    hierarchy.header.stamp = now();
    hierarchy.ns = "mlatc_hierarchy";
    hierarchy.id = id++;
    hierarchy.type = visualization_msgs::msg::Marker::LINE_LIST;
    hierarchy.action = visualization_msgs::msg::Marker::ADD;
    hierarchy.scale.x = 0.015;
    hierarchy.color.r = 1.0;
    hierarchy.color.g = 1.0;
    hierarchy.color.b = 1.0;
    hierarchy.color.a = 0.8;
    for (const auto & [upper_layer, parent, child] : mapper_.hierarchyEdges()) {
      const auto & parent_position = mapper_.layers()[static_cast<size_t>(upper_layer)].node(parent).position;
      const auto & child_position = mapper_.layers()[static_cast<size_t>(upper_layer - 1)].node(child).position;
      if (!parent_position.allFinite() || !child_position.allFinite()) continue;
      geometry_msgs::msg::Point parent_point, child_point;
      parent_point.x = parent_position.x(); parent_point.y = parent_position.y();
      parent_point.z = parent_position.z() + static_cast<float>(upper_layer) * layer_z_offset_;
      child_point.x = child_position.x(); child_point.y = child_position.y();
      child_point.z = child_position.z() + static_cast<float>(upper_layer - 1) * layer_z_offset_;
      hierarchy.points.push_back(parent_point);
      hierarchy.points.push_back(child_point);
    }
    output.markers.push_back(hierarchy);
  }
  publisher_->publish(output);
}
