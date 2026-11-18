#include "preprocess/preprocess_node.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <functional>
#include <limits>
#include <sstream>
#include <stdexcept>
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

  // virtual lidar
  const std::string boxes_file = declare_parameter<std::string>("boxes_file", "");
  const double rate_hz = declare_parameter<double>("scan_rate_hz", 5.0);
  const int h_samples = static_cast<int>(declare_parameter<int>("h_samples", 256));
  const int v_samples = static_cast<int>(declare_parameter<int>("v_samples", 64));
  const double v_half_fov_deg = declare_parameter<double>("v_half_fov_deg", 45.0);
  min_range_ = static_cast<float>(declare_parameter<double>("min_range", 0.3));
  max_range_ = static_cast<float>(declare_parameter<double>("max_range", 20.0));
  range_noise_std_ = declare_parameter<double>("range_noise_std", 0.0);
  remove_box_ = declare_parameter<std::string>("remove_box", "");
  remove_after_s_ = declare_parameter<double>("remove_after_sec", 30.0);
  rng_.seed(static_cast<uint32_t>(declare_parameter<int>("seed", 1)));

  if (rate_hz <= 0.0 || h_samples < 1 || v_samples < 1) {
    throw std::invalid_argument("scan_rate_hz, h_samples, v_samples must be positive");
  }
  scan_period_s_ = 1.0 / rate_hz;

  loadBoxes(boxes_file);
  buildRays(h_samples, v_samples, v_half_fov_deg * M_PI / 180.0);

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  lidar_offset_ << 0.0f, 0.0f, static_cast<float>(offset_z);

  processed_scan_pub_ = create_publisher<preprocess::msg::ProcessedScan>(
    "/mapping/processed_scan", rclcpp::SensorDataQoS());

  gz_node_.Subscribe(
    "/world/" + world + "/dynamic_pose/info", &PreprocessNode::poseCallback, this);

  process_timer_ = create_wall_timer(
    std::chrono::milliseconds(5), std::bind(&PreprocessNode::processPending, this));

  RCLCPP_INFO(
    get_logger(),
    "Virtual lidar for '%s': %zu boxes, %d x %d rays, %.1f Hz, range [%.2f, %.1f] m, noise %.3f m.",
    robot_name_.c_str(), boxes_.size(), h_samples, v_samples, rate_hz,
    min_range_, max_range_, range_noise_std_);
}

// ---------------------------------------------------------------------------
// World geometry and ray table
// ---------------------------------------------------------------------------
void PreprocessNode::loadBoxes(const std::string &path)
{
  std::ifstream in(path);
  if (!in) {
    throw std::runtime_error("cannot open boxes_file '" + path + "'");
  }
  std::string line;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#') {
      continue;
    }
    std::istringstream ss(line);
    Box b;
    float v[15];
    if (!(ss >> b.name)) {
      continue;
    }
    for (float &x : v) {
      ss >> x;
    }
    if (ss.fail()) {
      RCLCPP_WARN(get_logger(), "Skipping malformed box line: %s", line.c_str());
      continue;
    }
    b.c = Eigen::Vector3f(v[0], v[1], v[2]);
    Eigen::Matrix3f R;
    R << v[3], v[4], v[5],
      v[6], v[7], v[8],
      v[9], v[10], v[11];
    b.Rt = R.transpose();
    b.h = Eigen::Vector3f(v[12], v[13], v[14]);
    boxes_.push_back(b);
  }
  if (boxes_.empty()) {
    throw std::runtime_error("no boxes loaded from '" + path + "'");
  }
}

// Same layout as the old gpu_lidar: azimuth 0..2pi (+X, counter-clockwise), elevation +-fov.
void PreprocessNode::buildRays(int h_samples, int v_samples, double v_half_fov_rad)
{
  ray_dirs_.clear();
  ray_dirs_.reserve(static_cast<size_t>(h_samples) * v_samples);
  for (int j = 0; j < v_samples; ++j) {
    const double el = (v_samples > 1) ?
      -v_half_fov_rad + j * (2.0 * v_half_fov_rad) / (v_samples - 1) : 0.0;
    for (int i = 0; i < h_samples; ++i) {
      const double az = i * 2.0 * M_PI / h_samples;     // no duplicate ray at 2*pi
      ray_dirs_.emplace_back(
        static_cast<float>(std::cos(el) * std::cos(az)),
        static_cast<float>(std::cos(el) * std::sin(az)),
        static_cast<float>(std::sin(el)));
    }
  }
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
      if (!pose_buf_.empty() && s.t < pose_buf_.back().t - 1.0) {
        pose_buf_.clear();        // simulation restarted
      }
      if (pose_buf_.empty() || s.t > pose_buf_.back().t) {
        pose_buf_.push_back(s);
      }
      while (!pose_buf_.empty() && pose_buf_.front().t < s.t - pose_buffer_s_) {
        pose_buf_.pop_front();
      }
    }
    if (s.t - last_tf_t_ >= tf_period_s_ || s.t < last_tf_t_) {
      publishTF(s);
      last_tf_t_ = s.t;
    }
    break;
  }
}

void PreprocessNode::publishTF(const PoseSample &s)
{
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = toStamp(s.t);
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
// Scan scheduling: stamps are exact multiples of the scan period in sim time
// ---------------------------------------------------------------------------
void PreprocessNode::scheduleScans()
{
  const double now = get_clock()->now().seconds();
  if (now < last_now_ - 1.0) {          // simulation restarted
    next_scan_k_ = -1;
    pending_.clear();
  }
  last_now_ = now;

  if (pose_buf_.empty()) {
    return;
  }
  if (now <= 0.0) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "No /clock received yet (sim time is 0).");
  }
  if (next_scan_k_ < 0) {
    next_scan_k_ = static_cast<int64_t>(std::ceil(pose_buf_.back().t / scan_period_s_));
  }
  while (static_cast<double>(next_scan_k_) * scan_period_s_ <= now) {
    pending_.push_back(
      {static_cast<double>(next_scan_k_) * scan_period_s_, std::chrono::steady_clock::now()});
    ++next_scan_k_;
  }
  while (pending_.size() > 50) {
    pending_.pop_front();
    ++n_dropped_;
  }
}

// Pose at time ts: interpolated between the two bracketing samples, or held (hold_pose_).
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
  if (it->t == ts || it == pose_buf_.begin()) {
    out = *it;
    return true;
  }
  if (hold_pose_) {
    out = *(it - 1);      // latest sample before the stamp
    return true;
  }

  const PoseSample & a = *(it - 1);
  const PoseSample & b = *it;
  const double gap = b.t - a.t;
  if (gap > max_interp_gap_s_) {
    out = a;              // Gazebo publishes only changed poses: robot stood still, then moved.
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
    double stamp;
    PoseSample pose;
  };
  std::vector<Ready> ready;

  {
    std::lock_guard<std::mutex> lock(mutex_);
    scheduleScans();
    const auto now = std::chrono::steady_clock::now();
    while (!pending_.empty()) {
      const double stamp = pending_.front().stamp;
      const bool have_newer = !pose_buf_.empty() && pose_buf_.back().t >= stamp;
      const bool timed_out =
        std::chrono::duration<double>(now - pending_.front().created).count() > max_wait_s_;
      if (!have_newer && !timed_out) {
        break;    // wait for a pose at or after the scan stamp
      }
      PoseSample ps;
      bool held = false;
      if (lookupPose(stamp, ps, held)) {
        if (held) {
          ++n_held_;
        }
        ready.push_back({stamp, ps});
      } else {
        ++n_dropped_;
      }
      pending_.pop_front();
    }
  }

  for (auto & r : ready) {
    const Eigen::Vector3f & pos = r.pose.p;
    const Eigen::Quaternionf & q = r.pose.q;

    const auto output = castScan(r.stamp, pos, q);

    const Eigen::Vector3f sensor = pos + q * lidar_offset_;
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = output.header.stamp;
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

// ---------------------------------------------------------------------------
// The virtual lidar: ray-box slab test against every active box, nearest hit wins.
// Output is already in the map (world) frame. Rays with no hit are simply absent.
// ---------------------------------------------------------------------------
sensor_msgs::msg::PointCloud2 PreprocessNode::castScan(
  double stamp, const Eigen::Vector3f & pos, const Eigen::Quaternionf & q)
{
  const Eigen::Matrix3f R = q.toRotationMatrix();
  const Eigen::Vector3f origin = pos + q * lidar_offset_;

  struct Active
  {
    const Box * b;
    Eigen::Vector3f o;      // ray origin in the box frame
  };
  std::vector<Active> active;
  active.reserve(boxes_.size());
  for (const auto & b : boxes_) {
    if (!remove_box_.empty() && b.name == remove_box_ && stamp >= remove_after_s_) {
      continue;
    }
    active.push_back({&b, b.Rt * (origin - b.c)});
  }

  const bool noisy = range_noise_std_ > 0.0;
  std::normal_distribution<float> noise(0.0f, static_cast<float>(range_noise_std_));
  constexpr float kInf = std::numeric_limits<float>::infinity();

  pcl::PointCloud<pcl::PointXYZ> out;
  out.reserve(ray_dirs_.size());

  for (const auto & dl : ray_dirs_) {
    const Eigen::Vector3f d = R * dl;
    float best = max_range_;
    bool hit = false;

    for (const auto & a : active) {
      const Eigen::Vector3f dd = a.b->Rt * d;
      float tmin = -kInf;
      float tmax = kInf;
      bool miss = false;
      for (int i = 0; i < 3; ++i) {
        if (std::abs(dd[i]) < 1e-9f) {
          if (std::abs(a.o[i]) > a.b->h[i]) {
            miss = true;
            break;
          }
        } else {
          float t1 = (-a.b->h[i] - a.o[i]) / dd[i];
          float t2 = (a.b->h[i] - a.o[i]) / dd[i];
          if (t1 > t2) {
            std::swap(t1, t2);
          }
          tmin = std::max(tmin, t1);
          tmax = std::min(tmax, t2);
          if (tmin > tmax) {
            miss = true;
            break;
          }
        }
      }
      if (miss || tmax < 0.0f) {
        continue;
      }
      if (tmin >= min_range_ && tmin < best) {    // origin inside a box gives tmin < 0: skipped
        best = tmin;
        hit = true;
      }
    }

    if (hit) {
      const float r = noisy ? best + noise(rng_) : best;
      const Eigen::Vector3f w = origin + d * r;
      out.emplace_back(w.x(), w.y(), w.z());
    }
  }
  out.width = static_cast<uint32_t>(out.size());
  out.height = 1;
  out.is_dense = true;

  sensor_msgs::msg::PointCloud2 result;
  pcl::toROSMsg(out, result);
  result.header.frame_id = "map";
  result.header.stamp = toStamp(stamp);
  return result;
}