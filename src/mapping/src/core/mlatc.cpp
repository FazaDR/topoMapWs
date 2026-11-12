#include "atc_core/mlatc.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <unordered_set>
#include <utility>

namespace atc_core
{

MLATC::MLATC(const MLATCConfig & cfg)
: cfg_(cfg),
  rng_(cfg.seed)
{
  if (cfg_.free_area.has_value()) {
    FreeAreaConfig fa = *cfg_.free_area;
    fa.up_axis = cfg_.up_axis;
    free_.emplace(fa);
  }
  add_layer();
}

int MLATC::add_layer()
{
  const int idx = static_cast<int>(layers_.size());

  LayerConfig lc;
  lc.vigilance = cfg_.base_vigilance * std::pow(cfg_.alpha, static_cast<float>(idx));
  lc.lambda_points = cfg_.lambda_points;
  lc.deg_max = cfg_.deg_max;
  lc.theta_thr = cfg_.theta_thr;
  lc.gravity = gravity_for(cfg_.up_axis);
  lc.enable_attributes = (idx == 0);

  layers_.push_back(std::make_unique<ATCLayer>(lc));
  return idx;
}

int MLATC::total_nodes() const
{
  int n = 0;
  for (const auto & l : layers_) {
    n += l->map().node_count();
  }
  return n;
}

float MLATC::search_vigilance(int layer_index) const
{
  float total = 0.0f;
  for (int i = 0; i <= layer_index; ++i) {
    total += layers_[i]->vigilance();
  }
  return total;
}

std::vector<std::vector<int>> MLATC::hierarchical_nns(const Vec3 & point) const
{
  const int L = static_cast<int>(layers_.size());
  std::vector<std::vector<int>> winner_sets(L);

  for (int li = L - 1; li >= 0; --li) {
    const TopologicalMap & map = layers_[li]->map();
    std::vector<std::pair<float, int>> scored;   // (distance, id)

    if (li == L - 1) {
      // Top layer: all nodes
      const int n = map.node_count();
      scored.reserve(n);
      for (int s = 0; s < n; ++s) {
        scored.emplace_back((point - map.positions[s]).norm(), map.node_ids[s]);
      }
    } else {
      // Lower layers: children of the upper winners, filtered by Eq. (18)
      const float rho = search_vigilance(li);

      std::unordered_set<int> cand;
      for (int pid : winner_sets[li + 1]) {
        const auto & kids = hierarchy_.get_children(li + 1, pid);
        cand.insert(kids.begin(), kids.end());
      }
      for (int id : cand) {
        const float d = (point - map.get_position(id)).norm();
        if (d <= rho) {
          scored.emplace_back(d, id);
        }
      }
    }

    std::sort(scored.begin(), scored.end());   // by distance, then id
    winner_sets[li].reserve(scored.size());
    for (const auto & s : scored) {
      winner_sets[li].push_back(s.second);
    }
  }
  return winner_sets;
}

int MLATC::expand_top_layer()
{
  for (;;) {
    const int old_idx = static_cast<int>(layers_.size()) - 1;
    const TopologicalMap & old_map = layers_[old_idx]->map();

    if (old_map.node_count() != 2) {
      throw std::runtime_error("Top layer expansion requires exactly two nodes.");
    }

    const int first_id = old_map.node_ids[0];
    const int second_id = old_map.node_ids[1];
    const Vec3 first_pos = old_map.get_position(first_id);
    const Vec3 second_pos = old_map.get_position(second_id);

    const int new_idx = add_layer();
    ATCLayer & nl = *layers_[new_idx];

    // First old top node becomes the root's child
    const int root_id = nl.add_node(first_pos);
    hierarchy_.add_relationship(new_idx, root_id, first_id);

    // Second old top node is fed as an input
    const Winners w = nl.winner_search(second_pos);
    const int new_node = nl.update_by_winners(second_pos, w);
    const int parent_id = (new_node == kNoNode) ? w.s1 : new_node;
    hierarchy_.add_relationship(new_idx, parent_id, second_id);

    if (nl.map().node_count() == 2) {
      continue;   // new top layer also has exactly two nodes -> expand again
    }
    return new_idx;
  }
}

void MLATC::remove_leaf(int node_id)
{
  layers_[0]->remove_node(node_id);

  int child = node_id;
  const int L = static_cast<int>(layers_.size());

  for (int l = 1; l < L; ++l) {
    bool parent_empty = false;
    const int parent = hierarchy_.detach_child(l, child, parent_empty);
    if (parent == kNoNode || !parent_empty) {
      break;
    }

    // Never empty the top layer (single-root invariant)
    if (l == L - 1 && layers_[l]->map().node_count() <= 1) {
      break;
    }

    layers_[l]->remove_node(parent);
    child = parent;
  }
}

void MLATC::process_point(const Vec3 & point)
{
  const std::vector<std::vector<int>> winner_sets = hierarchical_nns(point);

  int layer_index = 0;
  int child_id = kNoNode;

  while (layer_index < static_cast<int>(layers_.size())) {
    ATCLayer & layer = *layers_[layer_index];

    const Winners w = layer.winner_search(point, &winner_sets[layer_index]);
    const int new_node = layer.update_by_winners(point, w);
    const int winner_id = (new_node != kNoNode) ? new_node : w.s1;

    if (layer_index > 0 && child_id != kNoNode) {
      hierarchy_.add_relationship(layer_index, winner_id, child_id);
    }

    if (new_node == kNoNode) {
      break;
    }

    if (layer_index == static_cast<int>(layers_.size()) - 1 &&
      layer.map().node_count() == 2)
    {
      expand_top_layer();
      break;
    }

    child_id = new_node;
    ++layer_index;
  }
}

void MLATC::process_frame(const Cloud & cloud, const Vec3 & robot_position, float robot_heading)
{
  if (free_) {
    free_->set_heading(robot_heading);
  }

  for (const auto & p : sample_points(cloud, cfg_.lambda_points, rng_)) {
    process_point(p);
  }

  if (free_ && !cloud.empty()) {
    const auto & m = layers_[0]->map();
    const std::vector<int> ids =
      free_->find_deletions(m.positions, m.node_ids, cloud, robot_position);
    for (int id : ids) {
      remove_leaf(id);
    }
  }
}

}  // namespace atc_core