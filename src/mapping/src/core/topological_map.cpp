#include "atc_core/topological_map.hpp"

namespace atc_core
{

int TopologicalMap::add_node(const Vec3 & position)
{
  const int id = next_node_id++;
  const int slot = node_count();

  positions.push_back(position);
  node_ids.push_back(id);
  normals.push_back(Vec3::Zero());
  slope_angles.push_back(0.0f);
  traversable.push_back(0);
  contour.push_back(0);

  id_to_slot[id] = slot;
  return id;
}

void TopologicalMap::remove_node(int node_id)
{
  auto it = id_to_slot.find(node_id);
  if (it == id_to_slot.end()) {
    return;
  }
  const int slot = it->second;

  auto g = graph.find(node_id);
  if (g != graph.end()) {
    for (const auto & kv : g->second) {
      auto other = graph.find(kv.first);
      if (other != graph.end()) {
        other->second.erase(node_id);
      }
    }
    graph.erase(g);
  }

  const int last = node_count() - 1;
  if (slot != last) {
    const int moved_id = node_ids[last];
    positions[slot] = positions[last];
    node_ids[slot] = moved_id;
    normals[slot] = normals[last];
    slope_angles[slot] = slope_angles[last];
    traversable[slot] = traversable[last];
    contour[slot] = contour[last];
    id_to_slot[moved_id] = slot;
  }

  positions.pop_back();
  node_ids.pop_back();
  normals.pop_back();
  slope_angles.pop_back();
  traversable.pop_back();
  contour.pop_back();

  id_to_slot.erase(node_id);
}

bool TopologicalMap::has_edge(int a, int b) const
{
  auto it = graph.find(a);
  return it != graph.end() && it->second.count(b) > 0;
}

void TopologicalMap::add_edge(int a, int b)
{
  graph[a][b] = 1;
  graph[b][a] = 1;
}

void TopologicalMap::remove_edge(int a, int b)
{
  auto ia = graph.find(a);
  if (ia != graph.end()) {
    ia->second.erase(b);
  }
  auto ib = graph.find(b);
  if (ib != graph.end()) {
    ib->second.erase(a);
  }
}

std::vector<float> TopologicalMap::edge_ages_of(int node_id) const
{
  std::vector<float> ages;
  auto it = graph.find(node_id);
  if (it == graph.end()) {
    return ages;
  }
  ages.reserve(it->second.size());
  for (const auto & kv : it->second) {
    ages.push_back(static_cast<float>(kv.second));
  }
  return ages;
}

std::vector<std::pair<int, int>> TopologicalMap::edges() const
{
  std::vector<std::pair<int, int>> out;
  for (const auto & a : graph) {
    for (const auto & b : a.second) {
      if (a.first < b.first) {
        out.emplace_back(a.first, b.first);
      }
    }
  }
  return out;
}

int TopologicalMap::num_edges() const
{
  int count = 0;
  for (const auto & a : graph) {
    for (const auto & b : a.second) {
      if (a.first < b.first) {
        ++count;
      }
    }
  }
  return count;
}

}  // namespace atc_core