#include "preprocess/preprocess_node.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <utility>
#include <vector>

#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace
{
builtin_interfaces::msg::Time toStamp(double t)
{
  builtin_interfaces::msg::Time s;
  const double sec = std::floor(t);
  s.sec = static_cast<int32_t>(sec);
  s.nanosec = static_cast<uint32_t>(std::llround((t - sec) * 1e9));
  if (s.nanosec >= 1000000000u) {
    s.sec += 1;
    s.nanosec -= 1000000000u;
  }
  return s;
}
}  // namespace

PreprocessNode::PreprocessNode()
: Node("preprocess_node")
{
  robot_name_ = declare_parameter<std::string>("robot_name", "ddmr");
  const auto world = declare_parameter<std::string>("world", "winding_room");
  const double offset_z = declare_parameter<double>("lidar_offset_z", 0.49);
  pose_buffer_s_ = declare_parameter<double>("pose_buffer_s", 3.0);
  max_wait_s_ = declare_parameter<double>("max_wait_s", 0.25);
  max_interp_gap_s_ = declare_parameter<double>("max_interp_gap_s", 0.1);
  tf_period_s_ = declare_parameter<double>("tf_period_s", 0.01);
  hold_pose_ = declare_parameter<bool>("hold_pose", false);

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  lidar_offset_ << 0.0f, 0.0f, static_cast<float>(offset_z);

  point_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "/points/points", rclcpp::SensorDataQoS(),
    std::bind(&PreprocessNode::pointCloudCallback, this, std::placeholders::_1));

  processed_scan_pub_ = create_publisher<preprocess::msg::ProcessedScan>(
    "/mapping/processed_scan", rclcpp::SensorDataQoS());

  gz_node_.Subscribe(
    "/world/" + world + "/dynamic_pose/info", &PreprocessNode::poseCallback, this);

  // Scans are processed from this timer only, so output order and ids stay consistent.
  process_timer_ = create_wall_timer(
    std::chrono::milliseconds(5), std::bind(&PreprocessNode::processPending, this));

  RCLCPP_INFO(
    get_logger(), "Preprocess node started for robot '%s' (stamped pose lookup).",
    robot_name_.c_str());
}

// ---------------------------------------------------------------------------
// Gazebo pose -> buffer (stamped with Gazebo's own sim time)
// ---------------------------------------------------------------------------
void PreprocessNode::poseCallback(const gz::msgs::Pose_V &msg)
{
  for (int i = 0; i < msg.pose_size(); ++i) {
    const auto &pose = msg.pose(i);
    if (pose.name() != robot_name_) {
      continue;
    }

    PoseSample s;
    s.t = msg.has_header() ?
      msg.header().stamp().sec() + msg.header().stamp().nsec() * 1e-9 :
      this->now().seconds();      // fallback (approximate)
    s.p = Eigen::Vector3f(pose.position().x(), pose.position().y(), pose.position().z());
    s.q = Eigen::Quaternionf(
      pose.orientation().w(), pose.orientation().x(),
      pose.orientation().y(), pose.orientation().z());
    s.q.normalize();

    {
      std::lock_guard<std::mutex> lock(mutex_);
      // Simulation restarted (time went backwards): drop old history.
      if (!pose_buf_.empty() && s.t < pose_buf_.back().t - 1.0) {
        pose_buf_.clear();
      }
      if (pose_buf_.empty() || s.t > pose_buf_.back().t) {
        pose_buf_.push_back(s);
      }
      while (!pose_buf_.empty() && pose_buf_.front().t < s.t - pose_buffer_s_) {
        pose_buf_.pop_front();
      }
    }
    if (s.t - last_tf_t_ >= tf_period_s_ || s.t < last_tf_t_) {   // 2nd case: sim restarted
      publishTF(s);
      last_tf_t_ = s.t;
    }
    break;
  }
}

void PreprocessNode::publishTF(const PoseSample &s)
{
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = toStamp(s.t);          // Gazebo time, not arrival time
  tf.header.frame_id = "map";
  tf.child_frame_id = "base_link";
  tf.transform.translation.x = s.p.x();
  tf.transform.translation.y = s.p.y();
  tf.transform.translation.z = s.p.z();
  tf.transform.rotation.x = s.q.x();
  tf.transform.rotation.y = s.q.y();
  tf.transform.rotation.z = s.q.z();
  tf.transform.rotation.w = s.q.w();
  tf_broadcaster_->sendTransform(tf);
}

// ---------------------------------------------------------------------------
// Clouds are queued, then paired with the pose at their stamp
// ---------------------------------------------------------------------------
void PreprocessNode::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  PendingScan p;
  p.msg = msg;
  p.stamp = rclcpp::Time(msg->header.stamp).seconds();
  p.arrived = std::chrono::steady_clock::now();

  std::lock_guard<std::mutex> lock(mutex_);
  pending_.push_back(std::move(p));
  while (pending_.size() > 50) {      // safety cap
    pending_.pop_front();
    ++n_dropped_;
  }
}

// Pose at time ts: interpolated between the two bracketing samples.
// held = true when the newest sample is older than ts (pose held, not extrapolated).
bool PreprocessNode::lookupPose(double ts, PoseSample &out, bool &held) const
{
  held = false;
  if (pose_buf_.empty() || ts < pose_buf_.front().t) {
    return false;
  }
  auto it = std::lower_bound(
    pose_buf_.begin(), pose_buf_.end(), ts,
    [](const PoseSample & s, double v) {return s.t < v;});

  if (it == pose_buf_.end()) {
    out = pose_buf_.back();
    held = true;
    return true;
  }
  if (hold_pose_) {
    out = *(it - 1);      // latest sample before the scan stamp
    return true;
  }

  const PoseSample & a = *(it - 1);
  const PoseSample & b = *it;
  const double gap = b.t - a.t;
  if (gap > max_interp_gap_s_) {
    out = a;      // Gazebo publishes only changed poses: robot stood still, then moved.
    return true;
  }
  const float alpha = static_cast<float>((ts - a.t) / gap);
  out.t = ts;
  out.p = a.p + alpha * (b.p - a.p);
  out.q = a.q.slerp(alpha, b.q);
  return true;
}

void PreprocessNode::processPending()
{
  struct Ready
  {
    PendingScan scan;
    PoseSample pose;
  };
  std::vector<Ready> ready;

  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto now = std::chrono::steady_clock::now();
    while (!pending_.empty()) {
      PendingScan & s = pending_.front();
      const bool have_newer = !pose_buf_.empty() && pose_buf_.back().t >= s.stamp;
      const bool timed_out =
        std::chrono::duration<double>(now - s.arrived).count() > max_wait_s_;
      if (!have_newer && !timed_out) {
        break;    // wait for a pose at or after the scan stamp
      }
      PoseSample ps;
      bool held = false;
      if (lookupPose(s.stamp, ps, held)) {
        if (held) {
          ++n_held_;
        }
        ready.push_back({std::move(s), ps});
      } else {
        ++n_dropped_;     // older than the pose buffer, or no pose yet
      }
      pending_.pop_front();
    }
  }

  for (auto & r : ready) {
    const Eigen::Vector3f & pos = r.pose.p;
    const Eigen::Quaternionf & q = r.pose.q;

    auto output = transformCloud(*r.scan.msg, pos, q);

    const Eigen::Vector3f sensor = pos + q * lidar_offset_;
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = r.scan.msg->header.stamp;
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
    ++n_processed_;
  }

  if (!ready.empty()) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "scans: %lu processed, %lu held (pose older than scan), %lu dropped (no pose)",
      static_cast<unsigned long>(n_processed_), static_cast<unsigned long>(n_held_),
      static_cast<unsigned long>(n_dropped_));
  }
}

sensor_msgs::msg::PointCloud2 PreprocessNode::transformCloud(
  const sensor_msgs::msg::PointCloud2 & cloud_msg,
  const Eigen::Vector3f & pos,
  const Eigen::Quaternionf & q)
{
  pcl::PointCloud<pcl::PointXYZ> input;
  pcl::fromROSMsg(cloud_msg, input);

  pcl::PointCloud<pcl::PointXYZ> output;
  output.reserve(input.size());

  const Eigen::Matrix3f R = q.toRotationMatrix();
  for (const auto & p : input.points) {
    const Eigen::Vector3f world = R * (Eigen::Vector3f(p.x, p.y, p.z) + lidar_offset_) + pos;
    output.emplace_back(world.x(), world.y(), world.z());
  }

  sensor_msgs::msg::PointCloud2 result;
  pcl::toROSMsg(output, result);
  result.header.frame_id = "map";
  result.header.stamp = cloud_msg.header.stamp;
  return result;
}