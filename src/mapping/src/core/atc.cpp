#include "atc_core/atc.hpp"

namespace atc_core
{

namespace
{
LayerConfig make_layer_config(const ATCConfig & c)
{
  LayerConfig l;
  l.vigilance = c.vigilance;
  l.lambda_points = c.lambda_points;
  l.deg_max = c.deg_max;
  l.theta_thr = c.theta_thr;
  l.gravity = gravity_for(c.up_axis);
  l.enable_attributes = true;
  return l;
}
}  // namespace

ATC::ATC(const ATCConfig & cfg)
: cfg_(cfg),
  layer_(make_layer_config(cfg)),
  rng_(cfg.seed)
{
  if (cfg_.free_area.has_value()) {
    FreeAreaConfig fa = *cfg_.free_area;
    fa.up_axis = cfg_.up_axis;   // single source of truth
    free_.emplace(fa);
  }
}

void ATC::process_frame(const Cloud & cloud, const Vec3 & robot_position, float robot_heading)
{
  if (free_) {
    free_->set_heading(robot_heading);
  }

  const Cloud sampled = sample_points(cloud, cfg_.lambda_points, rng_);

  if (layer_.map().node_count() == 0 && cfg_.random_init) {
    if (sampled.size() < 2) {
      return;
    }
    std::uniform_int_distribution<size_t> dist(0, sampled.size() - 1);
    const size_t i = dist(rng_);
    size_t j = dist(rng_);
    while (j == i) {
      j = dist(rng_);
    }
    layer_.add_node(sampled[i]);
    layer_.add_node(sampled[j]);
  }

  for (const auto & point : sampled) {
    const Winners w = layer_.winner_search(point);
    layer_.update_by_winners(point, w);
  }

  if (free_ && !cloud.empty()) {
    const auto & m = layer_.map();
    const std::vector<int> ids =
      free_->find_deletions(m.positions, m.node_ids, cloud, robot_position);
    for (int id : ids) {
      layer_.remove_node(id);
    }
  }
}

}  // namespace atc_core