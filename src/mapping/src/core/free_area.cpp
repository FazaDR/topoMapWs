#include "atc_core/free_area.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "atc_core/free_area_kdtree.hpp"

namespace atc_core
{

namespace
{

constexpr double kPiD = 3.14159265358979323846;

inline float wrap_angle(float a)
{
  double x = std::fmod(static_cast<double>(a) + kPiD, 2.0 * kPiD);
  if (x < 0.0) {
    x += 2.0 * kPiD;
  }
  return static_cast<float>(x - kPiD);
}

}  // namespace

float heading_from_quaternion(const Eigen::Quaternionf & q, UpAxis up)
{
  const Vec3 fwd = q * Vec3::UnitX();
  return up == UpAxis::Z ? std::atan2(fwd.y(), fwd.x()) : std::atan2(fwd.z(), fwd.x());
}

int Proximity::valid_count() const
{
  int c = 0;
  for (auto v : valid) {
    c += v ? 1 : 0;
  }
  return c;
}

FreeAreaDetector::FreeAreaDetector(const FreeAreaConfig & cfg)
: cfg_(cfg),
  fov_(static_cast<float>(cfg.fov_degrees * kPiD / 180.0))
{
  if (cfg_.up_axis == UpAxis::Z) {
    ax_a_ = 0; ax_b_ = 1; ax_up_ = 2;
  } else {
    ax_a_ = 0; ax_b_ = 2; ax_up_ = 1;
  }
}

void FreeAreaDetector::set_heading(float heading)
{
  heading_ = wrap_angle(heading);
}

Proximity FreeAreaDetector::proximity_points(
  const Cloud & cloud, const Vec3 & robot_position) const
{
  const int S = cfg_.sectors;
  const float nan = std::numeric_limits<float>::quiet_NaN();

  Proximity out;
  out.points.assign(S, Vec3::Constant(nan));
  out.valid.assign(S, 0);

  std::vector<float> best(S, std::numeric_limits<float>::infinity());

  const float half_fov = fov_ / 2.0f;
  const float width = fov_ / static_cast<float>(S);

  for (const auto & p : cloud) {
    if (std::abs(p[ax_up_] - robot_position[ax_up_]) > cfg_.horizontal_band) {
      continue;
    }
    const float da = p[ax_a_] - robot_position[ax_a_];
    const float db = p[ax_b_] - robot_position[ax_b_];
    const float rel = wrap_angle(std::atan2(db, da) - heading_);
    if (std::abs(rel) > half_fov) {
      continue;
    }
    const int s = std::clamp(
      static_cast<int>(std::floor((rel + half_fov) / width)), 0, S - 1);
    const float dist = std::hypot(da, db);
    if (dist < best[s]) {
      best[s] = dist;
      out.points[s] = p;
      out.valid[s] = 1;
    }
  }
  return out;
}

std::vector<uint8_t> FreeAreaDetector::nodes_in_free_area(
  const std::vector<Vec3> & node_positions, const Proximity & prox,
  const Vec3 & robot_position) const
{
  std::vector<uint8_t> out(node_positions.size(), 0);

  if (prox.valid_count() < 2) {
    return out;
  }

  const float ra = robot_position[ax_a_];
  const float rb = robot_position[ax_b_];

  struct Ray { float rx, ry, qx, qy; };
  std::vector<Ray> rays;
  for (int s = 0; s < cfg_.sectors; ++s) {
    if (!prox.valid[s]) {
      continue;
    }
    const float rx = prox.points[s][ax_a_];
    const float ry = prox.points[s][ax_b_];
    const float qx = rx - ra;
    const float qy = ry - rb;
    const float norm = std::hypot(qx, qy);
    if (norm > 1e-6f) {
      rays.push_back({rx, ry, qx / norm, qy / norm});
    }
  }
  if (rays.empty()) {
    return out;
  }

  const float half_fov = fov_ / 2.0f;
  const float width = fov_ / static_cast<float>(cfg_.sectors);

  for (size_t i = 0; i < node_positions.size(); ++i) {
    const float nx = node_positions[i][ax_a_];
    const float ny = node_positions[i][ax_b_];

    const float rel = wrap_angle(std::atan2(ny - rb, nx - ra) - heading_);
    if (std::abs(rel) > half_fov) {
      continue;
    }
    const int sector = std::clamp(
      static_cast<int>(std::floor((rel + half_fov) / width)), 0, cfg_.sectors - 1);
    if (!prox.valid[sector]) {
      continue;
    }

    bool inside = true;
    for (const auto & r : rays) {
      const float signed_d = r.qx * (r.rx - nx) + r.qy * (r.ry - ny);
      if (!(signed_d > cfg_.margin)) {
        inside = false;
        break;
      }
    }
    out[i] = inside ? 1 : 0;
  }
  return out;
}

std::vector<int> FreeAreaDetector::find_deletions(
  const std::vector<Vec3> & positions, const std::vector<int> & node_ids,
  const Cloud & cloud, const Vec3 & robot_position) const
{
  std::vector<int> result;
  if (cloud.empty() || positions.empty()) {
    return result;
  }

  const Proximity prox = proximity_points(cloud, robot_position);
  if (prox.valid_count() < 2) {
    return result;
  }

  const std::vector<uint8_t> in_free = nodes_in_free_area(positions, prox, robot_position);

  std::vector<size_t> cand;
  for (size_t i = 0; i < in_free.size(); ++i) {
    if (in_free[i]) {
      cand.push_back(i);
    }
  }
  if (cand.empty()) {
    return result;
  }

  // Scan-side KD-tree: delete candidates with no scan point nearby.
  ScanKdTree tree;
  tree.build(cloud);
  for (size_t i : cand) {
    if (!tree.has_point_within(positions[i], cfg_.distance)) {
      result.push_back(node_ids[i]);
    }
  }
  return result;
}

}  // namespace atc_core