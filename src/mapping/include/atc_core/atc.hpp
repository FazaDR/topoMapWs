#pragma once

#include <cstdint>
#include <optional>
#include <random>

#include "atc_core/atc_layer.hpp"
#include "atc_core/free_area.hpp"

namespace atc_core
{

struct ATCConfig
{
  float vigilance = 1.0f;
  int lambda_points = 5000;
  uint64_t seed = 42;
  bool random_init = true;
  float deg_max = 30.0f * kPi / 180.0f;
  float theta_thr = kPi;
  UpAxis up_axis = UpAxis::Z;
  std::optional<FreeAreaConfig> free_area;   // empty -> no free-area deletion
};

class ATC
{
public:
  explicit ATC(const ATCConfig & cfg = ATCConfig());

  const TopologicalMap & map() const { return layer_.map(); }
  int total_nodes() const { return layer_.map().node_count(); }
  bool free_area_enabled() const { return free_.has_value(); }

  // robot_position / robot_heading are only used when free-area is enabled.
  void process_frame(const Cloud & cloud, const Vec3 & robot_position, float robot_heading);

private:
  ATCConfig cfg_;
  ATCLayer layer_;
  std::mt19937_64 rng_;
  std::optional<FreeAreaDetector> free_;
};

}  // namespace atc_core