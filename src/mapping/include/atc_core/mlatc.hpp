#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <random>
#include <vector>

#include "atc_core/atc_layer.hpp"
#include "atc_core/free_area.hpp"
#include "atc_core/hierarchy.hpp"

namespace atc_core
{

struct MLATCConfig
{
  float base_vigilance = 0.5f;
  float alpha = 4.0f;
  int lambda_points = 4000;
  uint64_t seed = 42;
  float deg_max = 30.0f * kPi / 180.0f;
  float theta_thr = kPi;
  UpAxis up_axis = UpAxis::Z;
  std::optional<FreeAreaConfig> free_area;
};

class MLATC
{
public:
  explicit MLATC(const MLATCConfig & cfg = MLATCConfig());

  const std::vector<std::unique_ptr<ATCLayer>> & layers() const { return layers_; }
  const Hierarchy & hierarchy() const { return hierarchy_; }
  int total_nodes() const;
  bool free_area_enabled() const { return free_.has_value(); }

  void process_frame(const Cloud & cloud, const Vec3 & robot_position, float robot_heading);
  void process_point(const Vec3 & point);

private:
  int add_layer();
  float search_vigilance(int layer_index) const;
  std::vector<std::vector<int>> hierarchical_nns(const Vec3 & point) const;
  int expand_top_layer();
  void remove_leaf(int node_id);

  MLATCConfig cfg_;
  std::mt19937_64 rng_;
  std::optional<FreeAreaDetector> free_;
  std::vector<std::unique_ptr<ATCLayer>> layers_;
  Hierarchy hierarchy_;
};

}  // namespace atc_core