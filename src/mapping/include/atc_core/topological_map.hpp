#pragma once

#include <Eigen/Core>

#include <cstdint>
#include <unordered_map>
#include <utility>
#include <vector>

namespace atc_core
{

using Vec3 = Eigen::Vector3f;
using Cloud = std::vector<Vec3>;

constexpr int kNoNode = -1;

// Which world axis points up. Python dataset was Y-up, ROS/Gazebo is Z-up.
enum class UpAxis { Y, Z };

class TopologicalMap
{
public:
  using AdjMap = std::unordered_map<int, int>;  // neighbor id -> edge age

  // Dense node storage, all vectors have size node_count().
  std::vector<Vec3> positions;
  std::vector<Vec3> normals;
  std::vector<float> slope_angles;
  std::vector<uint8_t> traversable;
  std::vector<uint8_t> contour;
  std::vector<int> node_ids;                     // stable id per slot
  std::unordered_map<int, int> id_to_slot;       // stable id -> slot
  std::unordered_map<int, AdjMap> graph;         // graph[a][b] = age
  int next_node_id = 0;

  int node_count() const { return static_cast<int>(node_ids.size()); }

  int add_node(const Vec3 & position);
  void remove_node(int node_id);

  bool has_node(int node_id) const { return id_to_slot.count(node_id) > 0; }
  int get_slot(int node_id) const { return id_to_slot.at(node_id); }
  const Vec3 & get_position(int node_id) const { return positions[id_to_slot.at(node_id)]; }

  bool has_edge(int a, int b) const;
  void add_edge(int a, int b);
  void remove_edge(int a, int b);
  AdjMap & neighbors(int node_id) { return graph[node_id]; }

  std::vector<float> edge_ages_of(int node_id) const;
  std::vector<std::pair<int, int>> edges() const;   // each undirected edge once
  int num_edges() const;
};

}  // namespace atc_core