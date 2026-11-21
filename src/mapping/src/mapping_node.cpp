#include "mapping/mapping_node.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <vector>

#include <Eigen/Geometry>

#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/converter_options.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
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

std_msgs::msg::ColorRGBA freeAreaNodeColor()
{
  return rgba(0.75f, 0.2f, 0.95f);
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

  input_topic_ = declare_parameter<std::string>("input_topic", "/mapping/processed_scan");
  const std::string marker_topic =
    declare_parameter<std::string>("marker_topic", "/mapping/markers");
  const int qos_depth = static_cast<int>(declare_parameter<int>("qos_depth", 10));

  // Replay
  replay_bag_ = declare_parameter<std::string>("replay.bag", "");
  replay_storage_ = declare_parameter<std::string>("replay.storage_id", "sqlite3");
  replay_rate_ = declare_parameter<double>("replay.rate", 0.0);
  const bool replay_save_map = declare_parameter<bool>("replay.save_map", true);

  // Visualization
  marker_frame_ = declare_parameter<std::string>("marker_frame", "");
  node_size_ = declare_parameter<double>("node_size", 0.12);
  edge_width_ = declare_parameter<double>("edge_width", 0.02);
  layer_z_offset_ = declare_parameter<double>("layer_z_offset", 3.0);
  marker_period_ = declare_parameter<double>("marker_period", 0.2);

  const std::string layer_mode = declare_parameter<std::string>("layer_mode", "all");
  if (layer_mode == "layer1") {
    layer_mode_ = LayerMode::Layer1;
  } else if (layer_mode == "all") {
    layer_mode_ = LayerMode::All;
  } else if (layer_mode == "hierarchy") {
    layer_mode_ = LayerMode::Hierarchy;
  } else {
    RCLCPP_FATAL(
      get_logger(), "layer_mode must be 'layer1', 'all' or 'hierarchy', got '%s'",
      layer_mode.c_str());
    throw std::invalid_argument("invalid layer_mode");
  }

  const std::string free_viz = declare_parameter<std::string>("free_area_viz", "both");
  if (free_viz == "off") {
    free_viz_mode_ = FreeViz::Off;
  } else if (free_viz == "nodes") {
    free_viz_mode_ = FreeViz::Nodes;
  } else if (free_viz == "area") {
    free_viz_mode_ = FreeViz::Area;
  } else if (free_viz == "both") {
    free_viz_mode_ = FreeViz::Both;
  } else {
    RCLCPP_FATAL(
      get_logger(), "free_area_viz must be 'off', 'nodes', 'area' or 'both', got '%s'",
      free_viz.c_str());
    throw std::invalid_argument("invalid free_area_viz");
  }

  // Metrics
  metrics_enabled_ = declare_parameter<bool>("metrics.enabled", true);
  save_map_on_shutdown_ = declare_parameter<bool>("metrics.save_map_on_shutdown", false);
  output_dir_ = declare_parameter<std::string>("metrics.output_dir", "");
  const std::string run_name = declare_parameter<std::string>("metrics.run_name", "");
  if (!replay_bag_.empty() && replay_save_map) {
    save_map_on_shutdown_ = true;
  }

  const int lambda_points = static_cast<int>(declare_parameter<int>("lambda_points", 3500));
  const uint64_t seed = static_cast<uint64_t>(declare_parameter<int>("seed", 42));
  const float deg_max =
    static_cast<float>(declare_parameter<double>("deg_max_deg", 30.0) * M_PI / 180.0);
  const float theta_thr =
    static_cast<float>(declare_parameter<double>("theta_thr_deg", 180.0) * M_PI / 180.0);
  const std::string pool_str = declare_parameter<std::string>("pool_mode", "s1");
  const atc_core::PoolMode pool_mode =
    (pool_str == "one_hop") ? atc_core::PoolMode::OneHop : atc_core::PoolMode::S1Only;
  if (pool_str != "s1" && pool_str != "one_hop") {
    RCLCPP_WARN(get_logger(), "Unknown pool_mode '%s', using 's1'", pool_str.c_str());
  }

  const float vigilance = static_cast<float>(declare_parameter<double>("vigilance", 1.0));
  const bool random_init = declare_parameter<bool>("random_init", true);

  const float base_vigilance =
    static_cast<float>(declare_parameter<double>("base_vigilance", 1.0));
  const float alpha = static_cast<float>(declare_parameter<double>("alpha", 4.0));

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
    c.pool_mode = pool_mode;
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
    c.pool_mode = pool_mode;
    if (free_enabled) {
      c.free_area = fa;
    }
    mlatc_ = std::make_unique<atc_core::MLATC>(c);
  } else {
    RCLCPP_FATAL(get_logger(), "algorithm must be 'atc' or 'mlatc', got '%s'", algorithm.c_str());
    throw std::invalid_argument("invalid algorithm");
  }

  if (free_enabled && free_viz_mode_ != FreeViz::Off) {
    atc_core::FreeAreaConfig vfa = fa;
    vfa.up_axis = up_axis_;
    viz_free_ = std::make_unique<atc_core::FreeAreaDetector>(vfa);
  }

  // ---------------- ROS interfaces ----------------
  marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    marker_topic, rclcpp::QoS(rclcpp::KeepLast(1)));

  if (replay_bag_.empty()) {
    scan_sub_ = create_subscription<preprocess::msg::ProcessedScan>(
      input_topic_, rclcpp::QoS(rclcpp::KeepLast(qos_depth)).best_effort(),
      std::bind(&MappingNode::scanCallback, this, std::placeholders::_1));
  } else {
    // One-shot wall timer: runs the whole replay once the executor is spinning.
    replay_timer_ = create_wall_timer(
      std::chrono::milliseconds(200), std::bind(&MappingNode::runReplay, this));
  }

  // ---------------- metrics ----------------
  if (metrics_enabled_ || save_map_on_shutdown_) {
    std::string bag = replay_bag_;
    while (!bag.empty() && bag.back() == '/') {
      bag.pop_back();
    }
    const std::string bag_name =
      bag.empty() ? "live" : std::filesystem::path(bag).filename().string();
    const std::string label = !run_name.empty() ?
      run_name : bag_name + "_" + algorithm + (free_enabled ? "_free" : "_nofree");

    std::ostringstream meta;
    meta << "algorithm: " << algorithm << "\n"
         << "free_area_enabled: " << free_enabled << "\n"
         << "source: " << (replay_bag_.empty() ? "live" : replay_bag_) << "\n"
         << "lambda_points: " << lambda_points << "\n"
         << "seed: " << seed << "\n"
         << "vigilance: " << vigilance << "\n"
         << "base_vigilance: " << base_vigilance << "\n"
         << "alpha: " << alpha << "\n"
         << "deg_max_deg: " << deg_max * 180.0 / M_PI << "\n"
         << "theta_thr_deg: " << theta_thr * 180.0 / M_PI << "\n"
         << "pool_mode: " << pool_str << "\n"
         << "free_area.sectors: " << fa.sectors << "\n"
         << "free_area.horizontal_band: " << fa.horizontal_band << "\n"
         << "free_area.distance: " << fa.distance << "\n"
         << "free_area.fov_degrees: " << fa.fov_degrees << "\n"
         << "free_area.margin: " << fa.margin << "\n";

    openMetrics(output_dir_, label, meta.str());
  }

  RCLCPP_INFO(
    get_logger(),
    "mapping_node ready: algorithm=%s free_area=%s up_axis=%s layer_mode=%s free_area_viz=%s | %s",
    algorithm.c_str(), free_enabled ? "on" : "off", up.c_str(), layer_mode.c_str(),
    viz_free_ ? free_viz.c_str() : "off",
    replay_bag_.empty() ? ("live, input=" + input_topic_).c_str() :
    ("REPLAY " + replay_bag_ + " rate=" + std::to_string(replay_rate_)).c_str());
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

void MappingNode::openMetrics(
  const std::string & dir_param, const std::string & label, const std::string & meta)
{
  std::string dir = dir_param;
  if (dir.empty()) {
    const char * home = std::getenv("HOME");
    dir = std::string(home ? home : ".") + "/mapping_metrics";
  }

  const std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  char ts[32];
  std::strftime(ts, sizeof(ts), "%Y%m%d_%H%M%S", std::localtime(&now));
  const std::string run_dir = dir + "/" + label + "_" + ts;

  std::error_code ec;
  std::filesystem::create_directories(run_dir, ec);
  if (ec) {
    RCLCPP_ERROR(get_logger(), "Cannot create '%s': %s", run_dir.c_str(), ec.message().c_str());
    metrics_enabled_ = false;
    save_map_on_shutdown_ = false;
    return;
  }

  run_prefix_ = run_dir + "/map";

  std::ofstream(run_dir + "/params.txt") << meta;

  if (metrics_enabled_) {
    const std::string path = run_dir + "/metrics.csv";
    metrics_file_.open(path);
    if (!metrics_file_) {
      RCLCPP_ERROR(get_logger(), "Cannot open %s", path.c_str());
      metrics_enabled_ = false;
      return;
    }
    metrics_file_ <<
      "stamp_s,points,total_ms,total_nodes,layer1_nodes,dropped_total\n";
    RCLCPP_INFO(get_logger(), "Run folder: %s", run_dir.c_str());
  }
}

void MappingNode::writeMetricsRow(
  double stamp_s, size_t points, double total_ms, int total_nodes)
{
  if (!metrics_enabled_ || !metrics_file_) {
    return;
  }

  const atc_core::TopologicalMap & l1 = atc_ ? atc_->map() : mlatc_->layers()[0]->map();

  metrics_file_ << std::fixed << std::setprecision(6) << stamp_s << ','
                << points << ',' << std::setprecision(3) << total_ms << ','
                << total_nodes << ',' << l1.node_count() << ',' << dropped_frames_ << '\n';
  // Flushing every row is costly; in replay it is flushed once at the end.
  if (replay_bag_.empty()) {
    metrics_file_.flush();
  }
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
      get_logger(), steady_clock_, 5000, "Cannot read x/y/z float32 fields: %s", e.what());
    return false;
  }
  return true;
}

// ======================================================================
// Processing (shared by live callback and replay)
// ======================================================================

void MappingNode::scanCallback(const preprocess::msg::ProcessedScan::SharedPtr msg)
{
  processScan(*msg);
}

void MappingNode::processScan(const preprocess::msg::ProcessedScan & msg)
{
  const auto t0 = std::chrono::steady_clock::now();

  // Gaps mean lost frames (live: mapper too slow; replay: gaps in the recording itself).
  if (have_frame_id_ && msg.source_frame_id > last_frame_id_ + 1) {
    dropped_frames_ += msg.source_frame_id - last_frame_id_ - 1;
  }
  last_frame_id_ = msg.source_frame_id;
  have_frame_id_ = true;

  atc_core::Cloud cloud;
  if (!convertCloud(msg.cloud, cloud)) {
    return;
  }

  if (cloud.empty()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), steady_clock_, 5000, "Empty / all non-finite cloud, skipping");
    return;
  }

  const auto & p = msg.robot_pose.pose.position;
  const auto & o = msg.robot_pose.pose.orientation;
  Eigen::Quaternionf q(
    static_cast<float>(o.w), static_cast<float>(o.x),
    static_cast<float>(o.y), static_cast<float>(o.z));
  if (q.norm() < 1e-6f) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), steady_clock_, 5000, "Zero quaternion in robot_pose, skipping");
    return;
  }
  q.normalize();

  const atc_core::Vec3 base(
    static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z));
  const atc_core::Vec3 sensor = base + q * lidar_offset_;
  const float heading = atc_core::heading_from_quaternion(q, up_axis_);

  int total_nodes = 0;
  if (atc_) {
    atc_->process_frame(cloud, sensor, heading);
    total_nodes = atc_->total_nodes();
  } else {
    mlatc_->process_frame(cloud, sensor, heading);
    total_nodes = mlatc_->total_nodes();
  }

  ++frames_processed_;

  const auto t1 = std::chrono::steady_clock::now();
  const double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

  writeMetricsRow(
    rclcpp::Time(msg.header.stamp).seconds(), cloud.size(), ms, total_nodes);

  RCLCPP_INFO_THROTTLE(
    get_logger(), steady_clock_, 2000,
    "frame %lu | %zu pts | %.1f ms | nodes %d | heading %.2f rad | dropped %lu",
    static_cast<unsigned long>(msg.source_frame_id), cloud.size(), ms, total_nodes,
    static_cast<double>(heading), static_cast<unsigned long>(dropped_frames_));

  // Throttled marker publishing (wall clock)
  if (marker_pub_->get_subscription_count() > 0) {
    const double since = std::chrono::duration<double>(t1 - last_marker_wall_).count();
    if (since >= marker_period_) {
      FreeAreaViz fv;
      if (viz_free_) {
        const atc_core::TopologicalMap & l1 =
          atc_ ? atc_->map() : mlatc_->layers()[0]->map();
        const int n = l1.node_count();
        viz_free_->set_heading(heading);
        fv.prox = viz_free_->proximity_points(cloud, sensor);
        fv.node_flags = viz_free_->nodes_in_free_area(
          std::vector<atc_core::Vec3>(l1.positions.begin(), l1.positions.begin() + n),
          fv.prox, sensor);
        fv.sensor = sensor;
        fv.valid = true;
      }

      publishMarkers(msg.cloud.header, fv);
      last_marker_wall_ = t1;
    }
  }
}

// ======================================================================
// Replay
// ======================================================================

void MappingNode::runReplay()
{
  replay_timer_->cancel();   // one-shot

  rosbag2_storage::StorageOptions storage;
  storage.uri = replay_bag_;
  storage.storage_id = replay_storage_;

  rosbag2_cpp::ConverterOptions converter;
  converter.input_serialization_format = "cdr";
  converter.output_serialization_format = "cdr";

  rosbag2_cpp::readers::SequentialReader reader;
  try {
    reader.open(storage, converter);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(
      get_logger(), "Cannot open bag '%s' (storage '%s'): %s",
      replay_bag_.c_str(), replay_storage_.c_str(), e.what());
    exit_code_ = 2;
    rclcpp::shutdown();
    return;
  }

  RCLCPP_INFO(
    get_logger(), "Replaying '%s' topic %s, rate %s",
    replay_bag_.c_str(), input_topic_.c_str(),
    replay_rate_ > 0.0 ? std::to_string(replay_rate_).c_str() : "max");

  rclcpp::Serialization<preprocess::msg::ProcessedScan> serialization;

  const auto wall0 = std::chrono::steady_clock::now();
  bool have_first = false;
  double first_stamp = 0.0;
  double max_lag_s = 0.0;
  uint64_t read_count = 0;

  while (rclcpp::ok() && reader.has_next()) {
    const auto bag_msg = reader.read_next();
    if (bag_msg->topic_name != input_topic_) {
      continue;
    }

    rclcpp::SerializedMessage serialized(*bag_msg->serialized_data);
    preprocess::msg::ProcessedScan msg;
    serialization.deserialize_message(&serialized, &msg);
    ++read_count;

    // Pacing (rate > 0): wait until this frame's recorded offset has elapsed.
    if (replay_rate_ > 0.0) {
      const double stamp = rclcpp::Time(msg.header.stamp).seconds();
      if (!have_first) {
        first_stamp = stamp;
        have_first = true;
      }
      const double offset_s = (stamp - first_stamp) / replay_rate_;
      const auto target = wall0 + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(offset_s));
      const auto now = std::chrono::steady_clock::now();
      if (target > now) {
        std::this_thread::sleep_until(target);
      } else {
        max_lag_s = std::max(max_lag_s, std::chrono::duration<double>(now - target).count());
      }
    }

    processScan(msg);
  }

  if (metrics_file_.is_open()) {
    metrics_file_.flush();
  }

  const double wall_s =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - wall0).count();

  if (read_count == 0) {
    RCLCPP_ERROR(
      get_logger(), "No messages on topic '%s' in the bag (check input_topic).",
      input_topic_.c_str());
    exit_code_ = 3;
  } else if (dropped_frames_ > 0) {
    exit_code_ = 1;
  }

  RCLCPP_INFO(
    get_logger(),
    "Replay finished: %lu frames read, %lu processed, %lu gaps in source_frame_id, "
    "%.1f s wall%s",
    static_cast<unsigned long>(read_count), static_cast<unsigned long>(frames_processed_),
    static_cast<unsigned long>(dropped_frames_), wall_s,
    replay_rate_ > 0.0 ? (", max lag " + std::to_string(max_lag_s) + " s").c_str() : "");

  rclcpp::shutdown();
}

// ======================================================================
// Markers
// ======================================================================

void MappingNode::publishMarkers(
  const std_msgs::msg::Header & cloud_header, const FreeAreaViz & free_viz)
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

  const bool want_free_nodes =
    free_viz.valid && (free_viz_mode_ == FreeViz::Nodes || free_viz_mode_ == FreeViz::Both);
  const bool want_free_area =
    free_viz.valid && (free_viz_mode_ == FreeViz::Area || free_viz_mode_ == FreeViz::Both);

  const std::vector<uint8_t> * flags = want_free_nodes ? &free_viz.node_flags : nullptr;

  if (atc_) {
    appendLayerMarkers(arr, atc_->map(), 0, true, header, flags);
  } else {
    const auto & layers = mlatc_->layers();
    const size_t shown =
      (layer_mode_ == LayerMode::Layer1) ? std::min<size_t>(1, layers.size()) : layers.size();

    for (size_t i = 0; i < shown; ++i) {
      appendLayerMarkers(
        arr, layers[i]->map(), static_cast<int>(i), layers[i]->attributes_enabled(), header,
        i == 0 ? flags : nullptr);
    }

    if (layer_mode_ == LayerMode::Hierarchy) {
      appendHierarchyMarkers(arr, header);
    }
  }

  if (want_free_area) {
    appendFreeAreaMarkers(arr, free_viz, header);
  }

  marker_pub_->publish(arr);
}

void MappingNode::appendLayerMarkers(
  MarkerArray & arr, const atc_core::TopologicalMap & map,
  int layer_index, bool attributes, const std_msgs::msg::Header & header,
  const std::vector<uint8_t> * free_flags) const
{
  const int n = map.node_count();
  if (n == 0) {
    return;
  }

  const float dz = static_cast<float>(layer_index * layer_z_offset_);

  std::vector<std_msgs::msg::ColorRGBA> colors(static_cast<size_t>(n));
  for (int s = 0; s < n; ++s) {
    const size_t i = static_cast<size_t>(s);
    if (free_flags != nullptr && i < free_flags->size() && (*free_flags)[i]) {
      colors[i] = freeAreaNodeColor();
    } else {
      colors[i] = attributes ? attributeColor(map, s) : layerColor(layer_index);
    }
  }

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

void MappingNode::appendHierarchyMarkers(
  MarkerArray & arr, const std_msgs::msg::Header & header) const
{
  if (!mlatc_) {
    return;
  }
  const auto & layers = mlatc_->layers();

  Marker links;
  links.header = header;
  links.ns = "hierarchy";
  links.id = 0;
  links.type = Marker::LINE_LIST;
  links.action = Marker::ADD;
  links.pose.orientation.w = 1.0;
  links.scale.x = std::max(edge_width_ * 0.5, 0.005);
  links.color = rgba(0.6f, 0.6f, 0.6f, 0.25f);

  for (const auto & r : mlatc_->hierarchy().rows()) {
    const int upper = r[0];
    if (upper < 1 || upper >= static_cast<int>(layers.size())) {
      continue;
    }
    const auto & up_map = layers[static_cast<size_t>(upper)]->map();
    const auto & lo_map = layers[static_cast<size_t>(upper - 1)]->map();
    if (!up_map.has_node(r[1]) || !lo_map.has_node(r[2])) {
      continue;
    }
    links.points.push_back(makePoint(
      up_map.get_position(r[1]), static_cast<float>(upper * layer_z_offset_)));
    links.points.push_back(makePoint(
      lo_map.get_position(r[2]), static_cast<float>((upper - 1) * layer_z_offset_)));
  }

  if (!links.points.empty()) {
    arr.markers.push_back(std::move(links));
  }
}

void MappingNode::appendFreeAreaMarkers(
  MarkerArray & arr, const FreeAreaViz & fv, const std_msgs::msg::Header & header) const
{
  const auto & pts = fv.prox.points;
  const auto & valid = fv.prox.valid;
  const size_t S = std::min(pts.size(), valid.size());
  if (S < 2) {
    return;
  }

  Marker fan;
  fan.header = header;
  fan.ns = "free_area";
  fan.id = 0;
  fan.type = Marker::TRIANGLE_LIST;
  fan.action = Marker::ADD;
  fan.pose.orientation.w = 1.0;
  fan.scale.x = fan.scale.y = fan.scale.z = 1.0;
  fan.color = rgba(0.2f, 0.9f, 0.3f, 0.25f);

  Marker prox;
  prox.header = header;
  prox.ns = "free_proximity";
  prox.id = 0;
  prox.type = Marker::SPHERE_LIST;
  prox.action = Marker::ADD;
  prox.pose.orientation.w = 1.0;
  prox.scale.x = prox.scale.y = prox.scale.z = node_size_ * 0.7;
  prox.color = rgba(0.0f, 0.9f, 0.9f, 1.0f);

  const geometry_msgs::msg::Point origin = makePoint(fv.sensor, 0.0f);

  for (size_t s = 0; s < S; ++s) {
    if (!valid[s]) {
      continue;
    }
    prox.points.push_back(makePoint(pts[s], 0.0f));

    if (s + 1 < S && valid[s + 1]) {
      fan.points.push_back(origin);
      fan.points.push_back(makePoint(pts[s], 0.0f));
      fan.points.push_back(makePoint(pts[s + 1], 0.0f));
    }
  }

  if (!fan.points.empty()) {
    arr.markers.push_back(std::move(fan));
  }
  if (!prox.points.empty()) {
    arr.markers.push_back(std::move(prox));
  }
}