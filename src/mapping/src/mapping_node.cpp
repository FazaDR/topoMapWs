#include "mapping/mapping_node.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <stdexcept>
#include <vector>

#include <Eigen/Geometry>

#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace
{

using Marker = visualization_msgs::msg::Marker;
using MarkerArray = visualization_msgs::msg::MarkerArray;

std_msgs::msg::ColorRGBA rgba(float r, float g, float b, float a = 1.0f)
{
  std_msgs::msg::ColorRGBA c;
  c.r = r; c.g = g; c.b = b; c.a = a;
  return c;
}

// Same rules as the Python viz: gray (no normal) / red (untraversable) /
// green (traversable + contour) / blue (traversable).
std_msgs::msg::ColorRGBA attributeColor(const atc_core::TopologicalMap & m, int slot)
{
  if (m.normals[slot].norm() <= 1e-8f) {
    return rgba(0.6f, 0.6f, 0.6f);
  }
  if (!m.traversable[slot]) {
    return rgba(1.0f, 0.0f, 0.0f);
  }
  if (m.contour[slot]) {
    return rgba(0.0f, 1.0f, 0.0f);
  }
  return rgba(0.0f, 0.0f, 1.0f);
}

// Upper (attribute-less) layers get one color per layer.
std_msgs::msg::ColorRGBA layerColor(int layer)
{
  static const std::array<std::array<float, 3>, 4> palette = {{
    {1.0f, 0.5f, 0.0f},
    {0.8f, 0.0f, 0.8f},
    {0.0f, 0.8f, 0.8f},
    {1.0f, 1.0f, 0.0f}}};
  const auto & p = palette[static_cast<size_t>(std::max(layer - 1, 0)) % palette.size()];
  return rgba(p[0], p[1], p[2]);
}

geometry_msgs::msg::Point makePoint(const atc_core::Vec3 & v, float dz)
{
  geometry_msgs::msg::Point p;
  p.x = v.x();
  p.y = v.y();
  p.z = v.z() + dz;
  return p;
}

}  // namespace

MappingNode::MappingNode()
: rclcpp::Node("mapping_node")
{
  // ---------------- parameters ----------------
  const std::string algorithm = declare_parameter<std::string>("algorithm", "mlatc");
  const bool free_enabled = declare_parameter<bool>("free_area_enabled", true);

  const std::string up = declare_parameter<std::string>("up_axis", "z");
  if (up == "z" || up == "Z") {
    up_axis_ = atc_core::UpAxis::Z;
  } else if (up == "y" || up == "Y") {
    up_axis_ = atc_core::UpAxis::Y;
  } else {
    RCLCPP_FATAL(get_logger(), "up_axis must be 'z' or 'y', got '%s'", up.c_str());
    throw std::invalid_argument("invalid up_axis");
  }

  const auto offset = declare_parameter<std::vector<double>>(
    "lidar_offset", std::vector<double>{0.0, 0.0, 0.0});
  if (offset.size() != 3) {
    RCLCPP_FATAL(get_logger(), "lidar_offset must have 3 elements");
    throw std::invalid_argument("invalid lidar_offset");
  }
  lidar_offset_ = Eigen::Vector3f(
    static_cast<float>(offset[0]), static_cast<float>(offset[1]), static_cast<float>(offset[2]));

  const std::string input_topic =
    declare_parameter<std::string>("input_topic", "/mapping/processed_scan");
  const std::string marker_topic =
    declare_parameter<std::string>("marker_topic", "/mapping/markers");
  const int qos_depth = static_cast<int>(declare_parameter<int>("qos_depth", 10));

  marker_frame_ = declare_parameter<std::string>("marker_frame", "");  // empty -> cloud frame
  node_size_ = declare_parameter<double>("node_size", 0.12);
  edge_width_ = declare_parameter<double>("edge_width", 0.02);
  layer_z_offset_ = declare_parameter<double>("layer_z_offset", 0.0);
  marker_period_ = declare_parameter<double>("marker_period", 0.2);

  // Metrics
  metrics_enabled_ = declare_parameter<bool>("metrics.enabled", true);
  save_map_on_shutdown_ = declare_parameter<bool>("metrics.save_map_on_shutdown", false);
  output_dir_ = declare_parameter<std::string>("metrics.output_dir", "");

  const int lambda_points = static_cast<int>(declare_parameter<int>("lambda_points", 3500));
  const uint64_t seed = static_cast<uint64_t>(declare_parameter<int>("seed", 42));
  const float deg_max =
    static_cast<float>(declare_parameter<double>("deg_max_deg", 30.0) * M_PI / 180.0);
  const float theta_thr =
    static_cast<float>(declare_parameter<double>("theta_thr_deg", 180.0) * M_PI / 180.0);

  // ATC only
  const float vigilance = static_cast<float>(declare_parameter<double>("vigilance", 1.0));
  const bool random_init = declare_parameter<bool>("random_init", true);

  // MLATC only
  const float base_vigilance =
    static_cast<float>(declare_parameter<double>("base_vigilance", 1.0));
  const float alpha = static_cast<float>(declare_parameter<double>("alpha", 4.0));

  // Free area
  atc_core::FreeAreaConfig fa;
  fa.sectors = static_cast<int>(declare_parameter<int>("free_area.sectors", 24));
  fa.horizontal_band =
    static_cast<float>(declare_parameter<double>("free_area.horizontal_band", 0.15));
  fa.distance = static_cast<float>(declare_parameter<double>("free_area.distance", 0.5));
  fa.fov_degrees = static_cast<float>(declare_parameter<double>("free_area.fov_degrees", 180.0));
  fa.margin = static_cast<float>(declare_parameter<double>("free_area.margin", 0.1));

  // ---------------- algorithm ----------------
  if (algorithm == "atc") {
    atc_core::ATCConfig c;
    c.vigilance = vigilance;
    c.lambda_points = lambda_points;
    c.seed = seed;
    c.random_init = random_init;
    c.deg_max = deg_max;
    c.theta_thr = theta_thr;
    c.up_axis = up_axis_;
    if (free_enabled) {
      c.free_area = fa;
    }
    atc_ = std::make_unique<atc_core::ATC>(c);
  } else if (algorithm == "mlatc") {
    atc_core::MLATCConfig c;
    c.base_vigilance = base_vigilance;
    c.alpha = alpha;
    c.lambda_points = lambda_points;
    c.seed = seed;
    c.deg_max = deg_max;
    c.theta_thr = theta_thr;
    c.up_axis = up_axis_;
    if (free_enabled) {
      c.free_area = fa;
    }
    mlatc_ = std::make_unique<atc_core::MLATC>(c);
  } else {
    RCLCPP_FATAL(get_logger(), "algorithm must be 'atc' or 'mlatc', got '%s'", algorithm.c_str());
    throw std::invalid_argument("invalid algorithm");
  }

  // ---------------- ROS interfaces ----------------
  marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    marker_topic, rclcpp::QoS(rclcpp::KeepLast(1)));

  scan_sub_ = create_subscription<preprocess::msg::ProcessedScan>(
    input_topic, rclcpp::QoS(rclcpp::KeepLast(qos_depth)).best_effort(),
    std::bind(&MappingNode::scanCallback, this, std::placeholders::_1));

  // ---------------- metrics ----------------
  if (metrics_enabled_ || save_map_on_shutdown_) {
    openMetrics(output_dir_);
  }

  RCLCPP_INFO(
    get_logger(), "mapping_node ready: algorithm=%s free_area=%s up_axis=%s input=%s",
    algorithm.c_str(), free_enabled ? "on" : "off", up.c_str(), input_topic.c_str());
}

MappingNode::~MappingNode()
{
  if (save_map_on_shutdown_ && !run_prefix_.empty()) {
    saveMap(run_prefix_);
  }
}

// ======================================================================
// Metrics
// ======================================================================

void MappingNode::openMetrics(const std::string & dir_param)
{
  std::string dir = dir_param;
  if (dir.empty()) {
    const char * home = std::getenv("HOME");
    dir = std::string(home ? home : ".") + "/mapping_metrics";
  }

  std::error_code ec;
  std::filesystem::create_directories(dir, ec);
  if (ec) {
    RCLCPP_ERROR(get_logger(), "Cannot create '%s': %s", dir.c_str(), ec.message().c_str());
    metrics_enabled_ = false;
    save_map_on_shutdown_ = false;
    return;
  }

  const std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  char ts[32];
  std::strftime(ts, sizeof(ts), "%Y%m%d_%H%M%S", std::localtime(&now));
  run_prefix_ = dir + "/" + get_name() + "_" + ts;

  if (metrics_enabled_) {
    const std::string path = run_prefix_ + ".csv";
    metrics_file_.open(path);
    if (!metrics_file_) {
      RCLCPP_ERROR(get_logger(), "Cannot open %s", path.c_str());
      metrics_enabled_ = false;
      return;
    }
    metrics_file_ <<
      "frame_id,stamp_s,points,convert_ms,process_ms,total_nodes,layer1_nodes,"
      "layer1_edges,num_layers,dropped_total,heading_rad\n";
    RCLCPP_INFO(get_logger(), "Recording metrics to %s", path.c_str());
  }
}

void MappingNode::writeMetricsRow(
  uint64_t frame_id, double stamp_s, size_t points, double convert_ms, double process_ms,
  int total_nodes, double heading)
{
  if (!metrics_enabled_ || !metrics_file_) {
    return;
  }

  const atc_core::TopologicalMap & l1 = atc_ ? atc_->map() : mlatc_->layers()[0]->map();
  const int num_layers = atc_ ? 1 : static_cast<int>(mlatc_->layers().size());

  metrics_file_ << frame_id << ',' << std::fixed << std::setprecision(6) << stamp_s << ','
                << points << ',' << std::setprecision(3) << convert_ms << ',' << process_ms << ','
                << total_nodes << ',' << l1.node_count() << ',' << l1.num_edges() << ','
                << num_layers << ',' << dropped_frames_ << ','
                << std::setprecision(4) << heading << '\n';
  metrics_file_.flush();
}

void MappingNode::saveMap(const std::string & prefix) const
{
  auto dump = [&](const atc_core::TopologicalMap & m, int layer) {
      const std::string base = prefix + "_layer" + std::to_string(layer);

      std::ofstream nf(base + "_nodes.csv");
      nf << "id,x,y,z,nx,ny,nz,slope_rad,traversable,contour\n";
      for (int s = 0; s < m.node_count(); ++s) {
        const auto & p = m.positions[static_cast<size_t>(s)];
        const auto & n = m.normals[static_cast<size_t>(s)];
        nf << m.node_ids[static_cast<size_t>(s)] << ','
           << p.x() << ',' << p.y() << ',' << p.z() << ','
           << n.x() << ',' << n.y() << ',' << n.z() << ','
           << m.slope_angles[static_cast<size_t>(s)] << ','
           << static_cast<int>(m.traversable[static_cast<size_t>(s)]) << ','
           << static_cast<int>(m.contour[static_cast<size_t>(s)]) << '\n';
      }

      std::ofstream ef(base + "_edges.csv");
      ef << "a,b\n";
      for (const auto & e : m.edges()) {
        ef << e.first << ',' << e.second << '\n';
      }
    };

  if (atc_) {
    dump(atc_->map(), 0);
  } else {
    const auto & layers = mlatc_->layers();
    for (size_t i = 0; i < layers.size(); ++i) {
      dump(layers[i]->map(), static_cast<int>(i));
    }
    std::ofstream hf(prefix + "_hierarchy.csv");
    hf << "upper_layer,parent_id,child_id\n";
    for (const auto & r : mlatc_->hierarchy().rows()) {
      hf << r[0] << ',' << r[1] << ',' << r[2] << '\n';
    }
  }

  RCLCPP_INFO(get_logger(), "Map saved with prefix %s", prefix.c_str());
}

// ======================================================================
// Cloud conversion
// ======================================================================

bool MappingNode::convertCloud(
  const sensor_msgs::msg::PointCloud2 & in, atc_core::Cloud & out) const
{
  out.clear();
  const size_t n = static_cast<size_t>(in.width) * static_cast<size_t>(in.height);
  if (n == 0) {
    return true;
  }

  try {
    sensor_msgs::PointCloud2ConstIterator<float> ix(in, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iy(in, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iz(in, "z");

    out.reserve(n);
    for (size_t i = 0; i < n; ++i, ++ix, ++iy, ++iz) {
      const float x = *ix, y = *iy, z = *iz;
      if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
        out.emplace_back(x, y, z);
      }
    }
  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 5000, "Cannot read x/y/z float32 fields: %s", e.what());
    return false;
  }
  return true;
}

// ======================================================================
// Main callback
// ======================================================================

void MappingNode::scanCallback(const preprocess::msg::ProcessedScan::SharedPtr msg)
{
  const auto t0 = std::chrono::steady_clock::now();

  // Frame bookkeeping: gaps mean mapping is slower than the input rate (or lost frames).
  if (have_frame_id_ && msg->source_frame_id > last_frame_id_ + 1) {
    dropped_frames_ += msg->source_frame_id - last_frame_id_ - 1;
  }
  last_frame_id_ = msg->source_frame_id;
  have_frame_id_ = true;

  // Convert (timed)
  const auto tc0 = std::chrono::steady_clock::now();
  atc_core::Cloud cloud;
  if (!convertCloud(msg->cloud, cloud)) {
    return;
  }
  const double convert_ms =
    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - tc0).count();

  if (cloud.empty()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Empty / all non-finite cloud, skipping");
    return;
  }

  // Pose -> sensor position + heading
  const auto & p = msg->robot_pose.pose.position;
  const auto & o = msg->robot_pose.pose.orientation;
  Eigen::Quaternionf q(
    static_cast<float>(o.w), static_cast<float>(o.x),
    static_cast<float>(o.y), static_cast<float>(o.z));
  if (q.norm() < 1e-6f) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Zero quaternion in robot_pose, skipping");
    return;
  }
  q.normalize();

  const atc_core::Vec3 base(
    static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z));
  const atc_core::Vec3 sensor = base + q * lidar_offset_;
  const float heading = atc_core::heading_from_quaternion(q, up_axis_);

  // Run (timed)
  int total_nodes = 0;
  const auto tp0 = std::chrono::steady_clock::now();
  if (atc_) {
    atc_->process_frame(cloud, sensor, heading);
    total_nodes = atc_->total_nodes();
  } else {
    mlatc_->process_frame(cloud, sensor, heading);
    total_nodes = mlatc_->total_nodes();
  }
  const double process_ms =
    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - tp0).count();

  writeMetricsRow(
    msg->source_frame_id, rclcpp::Time(msg->header.stamp).seconds(), cloud.size(),
    convert_ms, process_ms, total_nodes, static_cast<double>(heading));

  const auto t1 = std::chrono::steady_clock::now();
  const double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 2000,
    "frame %lu | %zu pts | %.1f ms | nodes %d | heading %.2f rad | dropped %lu",
    static_cast<unsigned long>(msg->source_frame_id), cloud.size(), ms, total_nodes,
    static_cast<double>(heading), static_cast<unsigned long>(dropped_frames_));

  // Throttled marker publishing (wall clock, so it also works with sim time)
  if (marker_pub_->get_subscription_count() > 0) {
    const double since =
      std::chrono::duration<double>(t1 - last_marker_wall_).count();
    if (since >= marker_period_) {
      publishMarkers(msg->cloud.header);
      last_marker_wall_ = t1;
    }
  }
}

// ======================================================================
// Markers
// ======================================================================

void MappingNode::publishMarkers(const std_msgs::msg::Header & cloud_header)
{
  std_msgs::msg::Header header = cloud_header;
  if (!marker_frame_.empty()) {
    header.frame_id = marker_frame_;
  }
  if (header.frame_id.empty()) {
    header.frame_id = "world";
  }

  MarkerArray arr;

  Marker clear;
  clear.header = header;
  clear.action = Marker::DELETEALL;
  arr.markers.push_back(clear);

  if (atc_) {
    appendLayerMarkers(arr, atc_->map(), 0, true, header);
  } else {
    const auto & layers = mlatc_->layers();
    for (size_t i = 0; i < layers.size(); ++i) {
      appendLayerMarkers(
        arr, layers[i]->map(), static_cast<int>(i), layers[i]->attributes_enabled(), header);
    }
  }

  marker_pub_->publish(arr);
}

void MappingNode::appendLayerMarkers(
  MarkerArray & arr, const atc_core::TopologicalMap & map,
  int layer_index, bool attributes, const std_msgs::msg::Header & header) const
{
  const int n = map.node_count();
  if (n == 0) {
    return;
  }

  const float dz = static_cast<float>(layer_index * layer_z_offset_);

  std::vector<std_msgs::msg::ColorRGBA> colors(static_cast<size_t>(n));
  for (int s = 0; s < n; ++s) {
    colors[static_cast<size_t>(s)] = attributes ? attributeColor(map, s) : layerColor(layer_index);
  }

  // ---- nodes ----
  Marker nodes;
  nodes.header = header;
  nodes.ns = "nodes";
  nodes.id = layer_index;
  nodes.type = Marker::SPHERE_LIST;
  nodes.action = Marker::ADD;
  nodes.pose.orientation.w = 1.0;
  nodes.scale.x = nodes.scale.y = nodes.scale.z = node_size_;
  nodes.points.reserve(static_cast<size_t>(n));
  for (int s = 0; s < n; ++s) {
    nodes.points.push_back(makePoint(map.positions[static_cast<size_t>(s)], dz));
  }
  nodes.colors = colors;
  arr.markers.push_back(std::move(nodes));

  // ---- edges ----
  Marker edges;
  edges.header = header;
  edges.ns = "edges";
  edges.id = layer_index;
  edges.type = Marker::LINE_LIST;
  edges.action = Marker::ADD;
  edges.pose.orientation.w = 1.0;
  edges.scale.x = edge_width_;

  for (const auto & e : map.edges()) {
    if (!map.has_node(e.first) || !map.has_node(e.second)) {
      continue;
    }
    const int sa = map.get_slot(e.first);
    const int sb = map.get_slot(e.second);
    edges.points.push_back(makePoint(map.positions[static_cast<size_t>(sa)], dz));
    edges.points.push_back(makePoint(map.positions[static_cast<size_t>(sb)], dz));

    const auto & ca = colors[static_cast<size_t>(sa)];
    const auto & cb = colors[static_cast<size_t>(sb)];
    const auto avg = rgba((ca.r + cb.r) * 0.5f, (ca.g + cb.g) * 0.5f, (ca.b + cb.b) * 0.5f);
    edges.colors.push_back(avg);
    edges.colors.push_back(avg);
  }
  if (!edges.points.empty()) {
    arr.markers.push_back(std::move(edges));
  }
}