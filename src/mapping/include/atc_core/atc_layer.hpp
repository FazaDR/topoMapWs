#pragma once

#include <limits>
#include <random>
#include <vector>

#include "atc_core/topological_map.hpp"

namespace atc_core
{

constexpr float kPi = 3.14159265358979323846f;

inline Vec3 gravity_for(UpAxis up)
{
  return up == UpAxis::Z ? Vec3(0.f, 0.f, -1.f) : Vec3(0.f, -1.f, 0.f);
}

struct LayerConfig
{
  float vigilance = 1.0f;
  int lambda_points = 5000;
  float deg_max = 30.0f * kPi / 180.0f;
  float theta_thr = kPi;
  Vec3 gravity = Vec3(0.f, 0.f, -1.f);
  bool enable_attributes = false;
};

struct Winners
{
  int s1 = kNoNode;
  int s2 = kNoNode;
  float d1 = std::numeric_limits<float>::infinity();
  float d2 = std::numeric_limits<float>::infinity();
};

// Random subsample of at most lambda points (shuffled order).
Cloud sample_points(const Cloud & points, int lambda_points, std::mt19937_64 & rng);

class ATCLayer
{
public:
  explicit ATCLayer(const LayerConfig & cfg = LayerConfig());

  TopologicalMap & map() { return map_; }
  const TopologicalMap & map() const { return map_; }
  float vigilance() const { return cfg_.vigilance; }
  bool attributes_enabled() const { return cfg_.enable_attributes; }

  int add_node(const Vec3 & point);
  void remove_node(int node_id);

  // candidates == nullptr -> search all nodes.
  Winners winner_search(const Vec3 & point, const std::vector<int> * candidates = nullptr) const;

  // Returns the id of a newly created node, or kNoNode.
  int update_by_winners(const Vec3 & point, const Winners & w);

private:
  void update_winner(const Vec3 & point, int winner);
  void update_neighbors(const Vec3 & point, int winner);
  void remove_old_edges(int s1, double gmax);
  double compute_gthr(const std::vector<float> & gamma) const;
  double compute_gmax(int s1) const;

  void estimate_normal(int s1);
  void estimate_traversability(int s1);
  void detect_contour(int s1, bool exclude_untraversable = true);

  LayerConfig cfg_;
  TopologicalMap map_;
  std::vector<int> winner_count_;   // aligned with map_ slots

  int deleted_edge_count_ = 0;
  double deleted_edge_mean_ = 0.0;
};

}  // namespace atc_core