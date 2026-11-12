#pragma once

#include <array>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "atc_core/topological_map.hpp"

namespace atc_core
{

class Hierarchy
{
public:
  using IdSet = std::unordered_set<int>;

  // upper_layer is the layer index of the parent (>= 1).
  void add_relationship(int upper_layer, int parent_id, int child_id);

  const IdSet & get_children(int upper_layer, int parent_id) const;
  int get_parent(int upper_layer, int child_id) const;   // kNoNode if none
  bool has_parent(int upper_layer, int child_id) const;

  // Detach child from its parent. Returns parent id (kNoNode if none).
  // parent_now_empty is true when the parent has no children left
  // (its children entry is erased in that case).
  int detach_child(int upper_layer, int child_id, bool & parent_now_empty);

  // Flat rows {upper_layer, parent_id, child_id} for saving.
  std::vector<std::array<int, 3>> rows() const;

private:
  std::unordered_map<int, std::unordered_map<int, IdSet>> children_;
  std::unordered_map<int, std::unordered_map<int, int>> parent_;
};

}  // namespace atc_core