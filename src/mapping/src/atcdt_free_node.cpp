#include "mapping/atcdt_free.hpp"
#include "mapping/free_area_kdtree.hpp"

#include <algorithm>
#include <chrono>
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

inline Eigen::Vector2f ground(const Eigen::Vector3f &p) { return {p.x(), p.y()}; }
inline float heightOf(const Eigen::Vector3f &p) { return p.z(); }

}  // namespace

AtcdtFreeNode::AtcdtFreeNode()
: Node("atcdt_free_node"),
  atcdt_(1.0f, 4000,
       30.0f * static_cast<float>(M_PI) / 180.0f,
       static_cast<float>(M_PI),
       Eigen::Vector3f(0.0f, 0.0f, -1.0f)),
sample_period_ms_(declare_parameter("sample_period_ms", 100.0)),
metrics_file_(declare_parameter("metrics_csv", "atcdt_free_metrics.csv"))
{
  if (sample_period_ms_ <= 0.0)
      throw std::invalid_argument("sample_period_ms must be positive");
    if (!metrics_file_.is_open())
    {
        throw std::runtime_error(
            "ATC free-area could not open metrics CSV: " +
            get_parameter("metrics_csv").as_string());
    }
    metrics_file_ << "frame,source_frame_id,stamp_sec,stamp_nanosec,processing_ms,sample_period_ms,skipped_frames,processed_frames,total_seen_frames,processed_ratio,node_count\n";

    point_sub_ = create_subscription<preprocess::msg::ProcessedScan>(
        "/mapping/processed_scan",
        rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
        std::bind(&AtcdtFreeNode::pointCloudCallback, this, std::placeholders::_1));

    topology_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/atcdt_free/topology",
        1);

    free_area_sectors_  = static_cast<int>(declare_parameter("free_area_sectors", 24));
    free_area_distance_ = static_cast<float>(declare_parameter("free_area_distance", 0.9));
    free_area_fov_      = static_cast<float>(declare_parameter("free_area_fov_degrees", 360.0) * M_PI / 180.0);
    horizontal_band_    = static_cast<float>(declare_parameter("horizontal_band", 0.15));
    free_area_margin_   = static_cast<float>(declare_parameter("free_area_margin", 0.1));

}

std::vector<Eigen::Vector3f> AtcdtFreeNode::convertPointCloud(
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

std::vector<Eigen::Vector3f> AtcdtFreeNode::findProximityPoints(
    const std::vector<Eigen::Vector3f> &cloud,
    const Eigen::Vector3f &robot_position,
    float heading) const
{
    std::vector<Eigen::Vector3f> proximity(free_area_sectors_, Eigen::Vector3f(NAN, NAN, NAN));
    std::vector<float> best(free_area_sectors_, std::numeric_limits<float>::infinity());

    const Eigen::Vector2f robot_g = ground(robot_position);
    const float band_center = heightOf(robot_position);   // pose is already the scan origin
    const float half_fov = free_area_fov_ / 2.0f;
    const float sector_width = free_area_fov_ / static_cast<float>(free_area_sectors_);

    for (const auto &point : cloud)
    {
        if (std::abs(heightOf(point) - band_center) > horizontal_band_) continue;

        const Eigen::Vector2f delta = ground(point) - robot_g;
        const float distance = delta.norm();
        if (distance < 1e-6f) continue;

        const float relative = normalizeAngle(std::atan2(delta.y(), delta.x()) - heading);
        if (std::abs(relative) > half_fov) continue;

        const int sector = std::clamp(
            static_cast<int>(std::floor((relative + half_fov) / sector_width)),
            0, free_area_sectors_ - 1);

        if (distance < best[sector]) { best[sector] = distance; proximity[sector] = point; }
    }
    return proximity;
}

bool AtcdtFreeNode::nodeInFreeArea(
    const Eigen::Vector3f &node_position,
    const std::vector<Eigen::Vector3f> &proximity_points,
    const Eigen::Vector3f &robot_position,
    float heading,
    float margin) const
{
    const Eigen::Vector2f robot_g = ground(robot_position);
    const Eigen::Vector2f node_g = ground(node_position);
    const float half_fov = free_area_fov_ / 2.0f;
    const float sector_width = free_area_fov_ / static_cast<float>(free_area_sectors_);

    const Eigen::Vector2f delta = node_g - robot_g;
    const float relative = normalizeAngle(std::atan2(delta.y(), delta.x()) - heading);
    if (std::abs(relative) > half_fov) return false;

    const int sector = std::clamp(
        static_cast<int>(std::floor((relative + half_fov) / sector_width)),
        0, free_area_sectors_ - 1);
    if (!proximity_points[sector].allFinite()) return false;

    int valid = 0;
    for (const auto &prox : proximity_points)
    {
        if (!prox.allFinite()) continue;
        const Eigen::Vector2f r = ground(prox);
        Eigen::Vector2f q = r - robot_g;              // robot -> obstacle (was flipped)
        const float n = q.norm();
        if (n < 1e-6f) continue;
        q /= n;
        ++valid;
        if (q.dot(r - node_g) <= margin) return false;   // Python: np.all(signed > margin)
    }
    return valid >= 2;
}

void AtcdtFreeNode::deleteFreeAreaNodes(
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

    const mapping::FreeAreaKdTree point_tree(point_cloud);
    std::vector<int> nodes_to_delete;
    for (int i = 0; i < atcdt_.map.node_count; ++i)
    {
        const Eigen::Vector3f &node_position = atcdt_.map.positions[i];
        if (!nodeInFreeArea(node_position, proximity_points, robot_position, heading))
            continue;

        if (!point_tree.hasPointWithin(node_position, free_area_distance_))
            nodes_to_delete.push_back(i);
    }

    std::sort(nodes_to_delete.begin(), nodes_to_delete.end(), std::greater<int>());
    for (int id : nodes_to_delete)
        atcdt_.removeNode(id);
}

void AtcdtFreeNode::pointCloudCallback(const preprocess::msg::ProcessedScan::SharedPtr msg)
{
    ++received_frames_;
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "ATC-DT free received source_frame_id=%lu with %u points",
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

    atcdt_.processFrame(points);             // learn first, then delete (Python order)

    if (!points.empty())
    {
        const auto &q = msg->robot_pose.pose.orientation;
        const Eigen::Vector3f position(
            static_cast<float>(msg->robot_pose.pose.position.x),
            static_cast<float>(msg->robot_pose.pose.position.y),
            static_cast<float>(msg->robot_pose.pose.position.z));
        const float heading = static_cast<float>(std::atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)));
        deleteFreeAreaNodes(points, position, heading);
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
                  << atcdt_.map.node_count << '\n';
    metrics_file_.flush();

    publishTopology();
}

void AtcdtFreeNode::publishTopology()
{
    visualization_msgs::msg::MarkerArray array;

    visualization_msgs::msg::Marker nodes;
    nodes.header.frame_id = "map";
    nodes.header.stamp = now();
    nodes.ns = "atcdt_free";
    nodes.id = 0;
    nodes.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    nodes.action = visualization_msgs::msg::Marker::ADD;
    nodes.scale.x = 0.08;
    nodes.scale.y = 0.08;
    nodes.scale.z = 0.08;
    nodes.color.r = 1.0;
    nodes.color.g = 0.1;
    nodes.color.b = 0.1;
    nodes.color.a = 1.0;

    for (int i = 0; i < atcdt_.map.node_count; ++i)
    {
        geometry_msgs::msg::Point p;
        p.x = atcdt_.map.positions[i].x();
        p.y = atcdt_.map.positions[i].y();
        p.z = atcdt_.map.positions[i].z();
        nodes.points.push_back(p);
    }
    array.markers.push_back(nodes);

    visualization_msgs::msg::Marker edges;
    edges.header = nodes.header;
    edges.ns = "atcdt_free_edges";
    edges.id = 1;
    edges.type = visualization_msgs::msg::Marker::LINE_LIST;
    edges.action = visualization_msgs::msg::Marker::ADD;
    edges.scale.x = 0.02;
    edges.color.r = 0.0;
    edges.color.g = 1.0;
    edges.color.b = 0.0;
    edges.color.a = 1.0;
    for (const auto &[a, b] : atcdt_.map.edges())
    {
        geometry_msgs::msg::Point p1, p2;
        p1.x = atcdt_.map.positions[a].x();
        p1.y = atcdt_.map.positions[a].y();
        p1.z = atcdt_.map.positions[a].z();
        p2.x = atcdt_.map.positions[b].x();
        p2.y = atcdt_.map.positions[b].y();
        p2.z = atcdt_.map.positions[b].z();
        edges.points.push_back(p1);
        edges.points.push_back(p2);
    }
    array.markers.push_back(edges);

    topology_pub_->publish(array);
}
