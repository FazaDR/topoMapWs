#include "atc_core/hierarchy.hpp"

namespace atc_core
{

void Hierarchy::add_relationship(int upper_layer, int parent_id, int child_id)
{
  children_[upper_layer][parent_id].insert(child_id);
  parent_[upper_layer][child_id] = parent_id;
}

const Hierarchy::IdSet & Hierarchy::get_children(int upper_layer, int parent_id) const
{
  static const IdSet empty;
  auto l = children_.find(upper_layer);
  if (l == children_.end()) {
    return empty;
  }
  auto p = l->second.find(parent_id);
  return p == l->second.end() ? empty : p->second;
}

int Hierarchy::get_parent(int upper_layer, int child_id) const
{
  auto l = parent_.find(upper_layer);
  if (l == parent_.end()) {
    return kNoNode;
  }
  auto c = l->second.find(child_id);
  return c == l->second.end() ? kNoNode : c->second;
}

bool Hierarchy::has_parent(int upper_layer, int child_id) const
{
  return get_parent(upper_layer, child_id) != kNoNode;
}

int Hierarchy::detach_child(int upper_layer, int child_id, bool & parent_now_empty)
{
  parent_now_empty = false;

  auto pl = parent_.find(upper_layer);
  if (pl == parent_.end()) {
    return kNoNode;
  }
  auto pc = pl->second.find(child_id);
  if (pc == pl->second.end()) {
    return kNoNode;
  }
  const int parent = pc->second;
  pl->second.erase(pc);

  auto & layer_children = children_[upper_layer];
  auto cs = layer_children.find(parent);
  if (cs != layer_children.end()) {
    cs->second.erase(child_id);
    if (cs->second.empty()) {
      layer_children.erase(cs);
      parent_now_empty = true;
    }
  } else {
    parent_now_empty = true;
  }
  return parent;
}

std::vector<std::array<int, 3>> Hierarchy::rows() const
{
  std::vector<std::array<int, 3>> out;
  for (const auto & l : children_) {
    for (const auto & p : l.second) {
      for (int c : p.second) {
        out.push_back({l.first, p.first, c});
      }
    }
  }
  return out;
}

}  // namespace atc_core