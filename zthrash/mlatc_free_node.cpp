#include "mapping/mlatc_free.hpp"
#include "mapping/free_area_kdtree.hpp"

#include <chrono>
#include <cmath>
#include <algorithm>
#include <limits>
#include <stdexcept>

#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace
{
inline float normalizeAngle(float angle)
{
    while (angle > M_PI)
        angle -= 2.0f * static_cast<float>(M_PI);
    while (angle < -M_PI)
        angle += 2.0f * static_cast<float>(M_PI);
    return angle;
}
}  // namespace

inline Eigen::Vector2f ground(const Eigen::Vector3f &p) { return {p.x(), p.y()}; }
inline float heightOf(const Eigen::Vector3f &p) { return p.z(); }

MlatcFreeNode::MlatcFreeNode()
: Node("mlatc_free_node"),
  mapper_(0.5f, 4.0f, 4000),
layer_z_offset_(declare_parameter("layer_z_offset", 4.0)),
show_parent_child_(declare_parameter("show_parent_child", false)),
sample_period_ms_(declare_parameter("sample_period_ms", 100.0)),
metrics_file_(declare_parameter("metrics_csv", "mlatc_free_metrics.csv"))
{
  if (sample_period_ms_ <= 0.0)
      throw std::invalid_argument("sample_period_ms must be positive");
    if (!metrics_file_.is_open())
    {
        throw std::runtime_error(
            "MLATC free-area could not open metrics CSV: " +
            get_parameter("metrics_csv").as_string());
    }
    metrics_file_ << "frame,source_frame_id,stamp_sec,stamp_nanosec,processing_ms,sample_period_ms,skipped_frames,processed_frames,total_seen_frames,processed_ratio,layer1_node_count\n";

    subscription_ = create_subscription<preprocess::msg::ProcessedScan>(
        "/mapping/processed_scan",
        rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
        std::bind(&MlatcFreeNode::callback, this, std::placeholders::_1));

    publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/mlatc_free/topology",
        1);

    free_area_sectors_ = declare_parameter("free_area_sectors", 24);
    free_area_distance_ = static_cast<float>(declare_parameter("free_area_distance", 0.45));
    free_area_fov_ = static_cast<float>(declare_parameter("free_area_fov_degrees", 360.0) * M_PI / 180.0);
    horizontal_band_ = static_cast<float>(declare_parameter("horizontal_band", 0.15));
    free_area_margin_ = static_cast<float>(declare_parameter("free_area_margin", 0.1));

}

std::vector<Eigen::Vector3f> MlatcFreeNode::convertPointCloud(
    const sensor_msgs::msg::PointCloud2 &cloud) const
{
    std::vector<Eigen::Vector3f> points;
    sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> y(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> z(cloud, "z");
    for (; x != x.end(); ++x, ++y, ++z)
    {
        const Eigen::Vector3f point(*x, *y, *z);
        if (point.allFinite())
            points.push_back(point);
    }
    return points;
}

std::vector<Eigen::Vector3f> MlatcFreeNode::findProximityPoints(
    const std::vector<Eigen::Vector3f> &cloud,
    const Eigen::Vector3f &robot_position,
    float heading) const
{
    std::vector<Eigen::Vector3f> proximity(free_area_sectors_, Eigen::Vector3f(NAN, NAN, NAN));
    if (cloud.empty())
        return proximity;

    const float half_fov = free_area_fov_ / 2.0f;
    const float sector_width = free_area_fov_ / static_cast<float>(free_area_sectors_);
    for (const auto &point : cloud)
    {
        if (std::abs(heightOf(point) - heightOf(robot_position)) > horizontal_band_)
            continue;
        const Eigen::Vector2f robot_xz = ground(robot_position);
        const Eigen::Vector2f point_xz = ground(point);
        const Eigen::Vector2f delta = point_xz - robot_xz;
        const float distance = delta.norm();
        if (distance < 1e-6f)
            continue;
        const float relative = normalizeAngle(std::atan2(delta.y(), delta.x()) - heading);
        if (std::abs(relative) > half_fov)
            continue;
        const int sector = std::clamp(
            static_cast<int>(std::floor((relative + half_fov) / sector_width)),
            0,
            free_area_sectors_ - 1);
        const auto &current = proximity[sector];
        if (!std::isfinite(current.x()))
        {
            proximity[sector] = point;
            continue;
        }
        const Eigen::Vector2f current_xz = ground(current);
        const float current_distance = (current_xz - robot_xz).norm();
        if (distance < current_distance)
            proximity[sector] = point;
    }
    return proximity;
}

bool MlatcFreeNode::nodeInFreeArea(
    const Eigen::Vector3f &node_position,
    const std::vector<Eigen::Vector3f> &proximity_points,
    const Eigen::Vector3f &robot_position,
    float heading,
    float margin) const
{
    const Eigen::Vector2f robot_xz = ground(robot_position);
    const Eigen::Vector2f node_xz = ground(node_position);
    const float half_fov = free_area_fov_ / 2.0f;
    const float sector_width = free_area_fov_ / static_cast<float>(free_area_sectors_);

    const Eigen::Vector2f delta = node_xz - robot_xz;
    const float relative = normalizeAngle(std::atan2(delta.y(), delta.x()) - heading);
    if (std::abs(relative) > half_fov)
        return false;

    const int sector = std::clamp(
        static_cast<int>(std::floor((relative + half_fov) / sector_width)),
        0,
        free_area_sectors_ - 1);
    if (!proximity_points[sector].allFinite())
        return false;

    for (const auto &proximity : proximity_points)
    {
        if (!proximity.allFinite())
            continue;
        const Eigen::Vector2f prox_xz = ground(proximity);
        const Eigen::Vector2f obstacle_direction = prox_xz - robot_xz;
        const float norm = obstacle_direction.norm();
        if (norm < 1e-6f)
            continue;

        const Eigen::Vector2f q = obstacle_direction / norm;
        if (q.dot(prox_xz - node_xz) <= margin)
            return false;
    }
    return true;
}

void MlatcFreeNode::deleteFreeAreaNodes(
    const std::vector<Eigen::Vector3f> &point_cloud,
    const Eigen::Vector3f &robot_position,
    float heading)
{
    if (point_cloud.empty())
        return;

    const auto proximity_points = findProximityPoints(point_cloud, robot_position, heading);
    int valid_count = 0;
    for (const auto &point : proximity_points)
    {
        if (point.allFinite())
            ++valid_count;
    }
    if (valid_count < 2)
        return;

    if (mapper_.layers().empty())
        return;

    const auto &layer = mapper_.layers().front();
    const mapping::FreeAreaKdTree point_tree(point_cloud);
    std::vector<int> nodes_to_delete;
    for (int node_id : layer.nodeIds())
    {
        const Eigen::Vector3f &node_position = layer.node(node_id).position;
        if (!nodeInFreeArea(
                node_position,
                proximity_points,
                robot_position,
                heading,
                free_area_margin_))
            continue;

        if (!point_tree.hasPointWithin(node_position, free_area_distance_))
            nodes_to_delete.push_back(node_id);
    }
    std::sort(nodes_to_delete.begin(), nodes_to_delete.end(), std::greater<int>());
    for (int node_id : nodes_to_delete)
        mapper_.removeLeaf(node_id);
}

void MlatcFreeNode::callback(const preprocess::msg::ProcessedScan::SharedPtr msg)
{
    ++received_frames_;
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "MLATC free received source_frame_id=%lu with %u points",
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
    mapper_.processFrame(points);
    if (!points.empty())
    {
        const auto &q = msg->robot_pose.pose.orientation;
        const Eigen::Vector3f robot_position(
            static_cast<float>(msg->robot_pose.pose.position.x),
            static_cast<float>(msg->robot_pose.pose.position.y),
            static_cast<float>(msg->robot_pose.pose.position.z));
        const float heading = static_cast<float>(std::atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)));
        deleteFreeAreaNodes(points, robot_position, heading);
    }
    ++processed_frames_;

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
                      << (mapper_.layers().empty() ? 0 : mapper_.layers().front().nodeCount()) << '\n';
    metrics_file_.flush();

    publish();
}

void MlatcFreeNode::publish()
{
    visualization_msgs::msg::MarkerArray output;
    int id = 0;
    for (size_t layer_index = 0; layer_index < mapper_.layers().size(); ++layer_index)
    {
        const auto &layer = mapper_.layers()[layer_index];
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = now();
        marker.ns = "mlatc_free";
        marker.id = id++;
        marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = marker.scale.y = marker.scale.z = layer_index == 0 ? 0.08 : 0.16;
        const float hue = std::fmod(static_cast<float>(layer_index) * 0.618034F, 1.0F);
        const float scaled_hue = hue * 6.0F;
        const int sector = static_cast<int>(scaled_hue);
        const float fraction = scaled_hue - static_cast<float>(sector);
        const float q = 1.0F - 0.9F * fraction;
        const float t = 1.0F - 0.9F * (1.0F - fraction);
        switch (sector % 6)
        {
            case 0: marker.color.r = 1.0F; marker.color.g = t; marker.color.b = 0.1F; break;
            case 1: marker.color.r = q; marker.color.g = 1.0F; marker.color.b = 0.1F; break;
            case 2: marker.color.r = 0.1F; marker.color.g = 1.0F; marker.color.b = t; break;
            case 3: marker.color.r = 0.1F; marker.color.g = q; marker.color.b = 1.0F; break;
            case 4: marker.color.r = t; marker.color.g = 0.1F; marker.color.b = 1.0F; break;
            default: marker.color.r = 1.0F; marker.color.g = 0.1F; marker.color.b = q; break;
        }
        marker.color.a = 1.0;

        for (int node_id : layer.nodeIds())
        {
            const auto &p = layer.node(node_id).position;
            if (!p.allFinite())
                continue;
            geometry_msgs::msg::Point point;
            point.x = p.x();
            point.y = p.y();
            point.z = p.z() + static_cast<float>(layer_index) * layer_z_offset_;
            marker.points.push_back(point);
        }
        output.markers.push_back(marker);

        visualization_msgs::msg::Marker edges;
        edges.header = marker.header;
        edges.ns = "mlatc_free_edges";
        edges.id = id++;
        edges.type = visualization_msgs::msg::Marker::LINE_LIST;
        edges.action = visualization_msgs::msg::Marker::ADD;
        edges.scale.x = 0.025;
        edges.color.r = marker.color.r;
        edges.color.g = marker.color.g;
        edges.color.b = marker.color.b;
        edges.color.a = 0.85;
        for (const auto &[first, second] : layer.edges())
        {
            const auto &p1 = layer.node(first).position;
            const auto &p2 = layer.node(second).position;
            if (!p1.allFinite() || !p2.allFinite())
                continue;
            geometry_msgs::msg::Point point1, point2;
            point1.x = p1.x();
            point1.y = p1.y();
            point1.z = p1.z() + static_cast<float>(layer_index) * layer_z_offset_;
            point2.x = p2.x();
            point2.y = p2.y();
            point2.z = p2.z() + static_cast<float>(layer_index) * layer_z_offset_;
            edges.points.push_back(point1);
            edges.points.push_back(point2);
        }
        output.markers.push_back(edges);
    }

    if (show_parent_child_)
    {
        visualization_msgs::msg::Marker hierarchy;
        hierarchy.header.frame_id = "map";
        hierarchy.header.stamp = now();
        hierarchy.ns = "mlatc_free_hierarchy";
        hierarchy.id = id++;
        hierarchy.type = visualization_msgs::msg::Marker::LINE_LIST;
        hierarchy.action = visualization_msgs::msg::Marker::ADD;
        hierarchy.scale.x = 0.015;
        hierarchy.color.r = 1.0;
        hierarchy.color.g = 1.0;
        hierarchy.color.b = 1.0;
        hierarchy.color.a = 0.8;
        for (const auto &[upper_layer, parent, child] : mapper_.hierarchyEdges())
        {
            if (upper_layer <= 0 ||
                static_cast<size_t>(upper_layer) >= mapper_.layers().size() ||
                static_cast<size_t>(upper_layer - 1) >= mapper_.layers().size() ||
                parent < 0 ||
                child < 0 ||
                parent >= mapper_.layers()[static_cast<size_t>(upper_layer)].nodeCount() ||
                child >= mapper_.layers()[static_cast<size_t>(upper_layer - 1)].nodeCount())
                continue;

            const auto &parent_position =
                mapper_.layers()[static_cast<size_t>(upper_layer)].node(parent).position;
            const auto &child_position =
                mapper_.layers()[static_cast<size_t>(upper_layer - 1)].node(child).position;
            if (!parent_position.allFinite() || !child_position.allFinite())
                continue;

            geometry_msgs::msg::Point parent_point, child_point;
            parent_point.x = parent_position.x();
            parent_point.y = parent_position.y();
            parent_point.z = parent_position.z() + static_cast<float>(upper_layer) * layer_z_offset_;
            child_point.x = child_position.x();
            child_point.y = child_position.y();
            child_point.z = child_position.z() + static_cast<float>(upper_layer - 1) * layer_z_offset_;
            hierarchy.points.push_back(parent_point);
            hierarchy.points.push_back(child_point);
        }
        output.markers.push_back(hierarchy);
    }
    publisher_->publish(output);
}
