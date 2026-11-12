#include "atc_core/atc_layer.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <numeric>

namespace atc_core
{

namespace
{

// numpy-style linear-interpolated percentile
double percentile(std::vector<float> v, double p)
{
  std::sort(v.begin(), v.end());
  const double pos = p / 100.0 * static_cast<double>(v.size() - 1);
  const size_t lo = static_cast<size_t>(std::floor(pos));
  const size_t hi = std::min(lo + 1, v.size() - 1);
  const double frac = pos - static_cast<double>(lo);
  return v[lo] + (static_cast<double>(v[hi]) - v[lo]) * frac;
}

}  // namespace

Cloud sample_points(const Cloud & points, int lambda_points, std::mt19937_64 & rng)
{
  const size_t n = points.size();
  std::vector<size_t> idx(n);
  std::iota(idx.begin(), idx.end(), 0);

  size_t take = n;
  if (n > static_cast<size_t>(lambda_points)) {
    take = static_cast<size_t>(lambda_points);
    // partial Fisher-Yates: choose `take` without replacement
    for (size_t i = 0; i < take; ++i) {
      std::uniform_int_distribution<size_t> dist(i, n - 1);
      std::swap(idx[i], idx[dist(rng)]);
    }
  } else {
    std::shuffle(idx.begin(), idx.end(), rng);
  }

  Cloud out;
  out.reserve(take);
  for (size_t i = 0; i < take; ++i) {
    out.push_back(points[idx[i]]);
  }
  return out;
}

ATCLayer::ATCLayer(const LayerConfig & cfg)
: cfg_(cfg)
{
  const float g = cfg_.gravity.norm();
  if (g > 1e-8f) {
    cfg_.gravity /= g;
  }
}

int ATCLayer::add_node(const Vec3 & point)
{
  const int id = map_.add_node(point);
  winner_count_.push_back(1);  // Eq. (3)
  return id;
}

void ATCLayer::remove_node(int node_id)
{
  if (!map_.has_node(node_id)) {
    return;
  }
  const int slot = map_.get_slot(node_id);
  const int last = map_.node_count() - 1;
  if (slot != last) {
    winner_count_[slot] = winner_count_[last];
  }
  winner_count_.pop_back();
  map_.remove_node(node_id);
}

Winners ATCLayer::winner_search(const Vec3 & point, const std::vector<int> * candidates) const
{
  Winners w;

  int best_slot = -1, second_slot = -1;
  float best_sq = std::numeric_limits<float>::infinity();
  float second_sq = std::numeric_limits<float>::infinity();

  auto consider = [&](int slot) {
      const float sq = (map_.positions[slot] - point).squaredNorm();
      if (sq < best_sq) {
        second_sq = best_sq;
        second_slot = best_slot;
        best_sq = sq;
        best_slot = slot;
      } else if (sq < second_sq) {
        second_sq = sq;
        second_slot = slot;
      }
    };

  if (candidates == nullptr) {
    const int n = map_.node_count();
    for (int s = 0; s < n; ++s) {
      consider(s);
    }
  } else {
    for (int id : *candidates) {
      consider(map_.get_slot(id));
    }
  }

  if (best_slot < 0) {
    return w;
  }
  w.s1 = map_.node_ids[best_slot];
  w.d1 = std::sqrt(best_sq);
  if (second_slot >= 0) {
    w.s2 = map_.node_ids[second_slot];
    w.d2 = std::sqrt(second_sq);
  }
  return w;
}

void ATCLayer::update_neighbors(const Vec3 & point, int winner)
{
  for (const auto & kv : map_.neighbors(winner)) {
    const int slot = map_.get_slot(kv.first);
    // Eq. (5)
    const float lr = 1.0f / (100.0f * static_cast<float>(winner_count_[slot]));
    map_.positions[slot] += lr * (point - map_.positions[slot]);
  }
}

void ATCLayer::update_winner(const Vec3 & point, int winner)
{
  const int slot = map_.get_slot(winner);
  winner_count_[slot] += 1;
  // Eq. (4)
  const float lr = 1.0f / (10.0f * static_cast<float>(winner_count_[slot]));
  map_.positions[slot] += lr * (point - map_.positions[slot]);
}

int ATCLayer::update_by_winners(const Vec3 & point, const Winners & w)
{
  // Case (a): add new node
  if (w.d1 >= cfg_.vigilance) {
    return add_node(point);
  }

  // Case (b): update winner
  update_winner(point, w.s1);

  const bool s2_ok = (w.s2 != kNoNode) && (w.d2 < cfg_.vigilance);

  if (s2_ok) {
    update_neighbors(point, w.s1);
  }

  // Age the edges that existed before the s1-s2 edge is added/reset.
  for (auto & kv : map_.graph[w.s1]) {
    kv.second += 1;
    map_.graph[kv.first][w.s1] += 1;
  }

  if (s2_ok) {
    if (map_.has_edge(w.s1, w.s2)) {
      map_.graph[w.s1][w.s2] = 1;
      map_.graph[w.s2][w.s1] = 1;
    } else {
      map_.add_edge(w.s1, w.s2);
    }
  }

  remove_old_edges(w.s1, compute_gmax(w.s1));

  if (cfg_.enable_attributes) {
    estimate_normal(w.s1);
    estimate_traversability(w.s1);
    detect_contour(w.s1);
  }

  return kNoNode;
}

void ATCLayer::remove_old_edges(int s1, double gmax)
{
  std::vector<int> to_remove;

  for (const auto & kv : map_.neighbors(s1)) {
    const int age = kv.second;
    if (static_cast<double>(age) > gmax) {
      ++deleted_edge_count_;
      deleted_edge_mean_ += (age - deleted_edge_mean_) / deleted_edge_count_;
      to_remove.push_back(kv.first);
    }
  }

  for (int nb : to_remove) {
    map_.remove_edge(s1, nb);
  }
}

double ATCLayer::compute_gthr(const std::vector<float> & gamma) const
{
  if (gamma.empty()) {
    return std::numeric_limits<double>::infinity();
  }
  const double q3 = percentile(gamma, 75.0);
  const double q1 = percentile(gamma, 25.0);
  return q3 + (q3 - q1);
}

double ATCLayer::compute_gmax(int s1) const
{
  const std::vector<float> gamma = map_.edge_ages_of(s1);
  if (gamma.empty()) {
    return std::numeric_limits<double>::infinity();
  }

  const double gthr = compute_gthr(gamma);
  if (deleted_edge_count_ == 0) {
    return gthr;
  }

  const double total = static_cast<double>(deleted_edge_count_) + gamma.size();
  const double w_del = deleted_edge_count_ / total;
  return deleted_edge_mean_ * w_del + gthr * (1.0 - w_del);
}

void ATCLayer::estimate_normal(int s1)
{
  const auto & nbrs = map_.neighbors(s1);
  if (nbrs.size() < 3) {
    return;
  }

  const int slot = map_.get_slot(s1);
  const Vec3 p = map_.positions[slot];

  Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
  for (const auto & kv : nbrs) {
    const Eigen::Vector3d d = (map_.positions[map_.get_slot(kv.first)] - p).cast<double>();
    cov += d * d.transpose();
  }

  // Eigenvalues ascending -> column 0 is the smallest-eigenvalue direction.
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);
  map_.normals[slot] = es.eigenvectors().col(0).cast<float>();
}

void ATCLayer::estimate_traversability(int s1)
{
  const int slot = map_.get_slot(s1);
  const Vec3 normal = map_.normals[slot];
  const float norm = normal.norm();
  if (norm < 1e-8f) {
    return;
  }

  const float cos_theta = std::clamp(normal.dot(cfg_.gravity) / norm, -1.0f, 1.0f);
  float angle = std::acos(cos_theta);

  // PCA normal sign is arbitrary: fold to acute angle from vertical.
  angle = std::min(angle, kPi - angle);

  map_.slope_angles[slot] = angle;
  map_.traversable[slot] = angle < cfg_.deg_max ? 1 : 0;
}

void ATCLayer::detect_contour(int s1, bool exclude_untraversable)
{
  const int slot = map_.get_slot(s1);

  std::vector<int> ids;
  for (const auto & kv : map_.neighbors(s1)) {
    ids.push_back(kv.first);
  }

  if (exclude_untraversable) {
    std::vector<int> confirmed;
    for (int nid : ids) {
      const int ns = map_.get_slot(nid);
      const bool has_normal = map_.normals[ns].norm() > 1e-8f;
      if (has_normal && map_.traversable[ns]) {
        confirmed.push_back(nid);
      }
    }
    ids.swap(confirmed);
  }

  if (ids.size() < 2) {
    map_.contour[slot] = 1;
    return;
  }

  const Vec3 p = map_.positions[slot];
  const Vec3 g = cfg_.gravity.normalized();

  Vec3 ref(1.f, 0.f, 0.f);
  if (std::abs(ref.dot(g)) > 0.9f) {
    ref = Vec3(0.f, 1.f, 0.f);
  }
  Vec3 u = g.cross(ref);
  u.normalize();
  const Vec3 v = g.cross(u);

  std::vector<float> angles;
  angles.reserve(ids.size());
  for (int nid : ids) {
    const Vec3 diff = map_.positions[map_.get_slot(nid)] - p;
    const Vec3 proj = diff - diff.dot(g) * g;
    angles.push_back(std::atan2(proj.dot(v), proj.dot(u)));
  }
  std::sort(angles.begin(), angles.end());

  float theta_max = (angles.front() + 2.0f * kPi) - angles.back();
  for (size_t i = 1; i < angles.size(); ++i) {
    theta_max = std::max(theta_max, angles[i] - angles[i - 1]);
  }

  map_.contour[slot] = theta_max > cfg_.theta_thr ? 1 : 0;
}

}  // namespace atc_core