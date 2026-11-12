#include "mapping/mlatc.hpp"

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <limits>
#include <random>

namespace mlatc
{

ATCDT::ATCDT(float v, int lambda_points, bool attributes)
: vigilance(v), attributes_(attributes)
{
  (void)lambda_points;
}

int ATCDT::addNode(const Eigen::Vector3f & point)
{
  if (!point.allFinite()) return -1;
  const int id = static_cast<int>(nodes_.size());
  nodes_.push_back({point});
  return id;
}

void ATCDT::removeNode(int id)
{
  if (id < 0 || static_cast<size_t>(id) >= nodes_.size()) return;
  const int last = static_cast<int>(nodes_.size()) - 1;
  for (auto &[node, neighbors] : graph_)
    neighbors.erase(id);
  graph_.erase(id);

  if (id != last)
  {
    nodes_[static_cast<size_t>(id)] = std::move(nodes_[static_cast<size_t>(last)]);
    auto last_it = graph_.find(last);
    if (last_it != graph_.end())
    {
      auto last_neighbors = std::move(last_it->second);
      graph_.erase(last_it);
      for (auto &[neighbor, age] : last_neighbors)
      {
        graph_[neighbor].erase(last);
        graph_[neighbor][id] = age;
      }
      graph_[id] = std::move(last_neighbors);
    }
  }
  nodes_.pop_back();
}

std::vector<int> ATCDT::nodeIds() const
{
  std::vector<int> ids;
  for (int i = 0; i < nodeCount(); ++i) ids.push_back(i);
  return ids;
}

std::vector<std::pair<int, int>> ATCDT::edges() const
{
  std::vector<std::pair<int, int>> result;
  for (const auto & [first, neighbors] : graph_) {
    for (const auto & [second, age] : neighbors) {
      (void)age;
      if (first < second) result.emplace_back(first, second);
    }
  }
  return result;
}

const Node & ATCDT::node(int id) const { return nodes_.at(static_cast<size_t>(id)); }

std::pair<int, int> ATCDT::winners(
  const Eigen::Vector3f & point, const std::vector<int> * candidates,
  float & d1, float & d2) const
{
  int first = -1;
  int second = -1;
  d1 = std::numeric_limits<float>::infinity();
  d2 = d1;
  const auto ids = candidates == nullptr ? nodeIds() : *candidates;
  for (int id : ids) {
    const float d = (nodes_[static_cast<size_t>(id)].position - point).norm();
    if (d < d1) { d2 = d1; second = first; d1 = d; first = id; }
    else if (d < d2) { d2 = d; second = id; }
  }
  return {first, second};
}

void ATCDT::updateNode(const Eigen::Vector3f & point, int id)
{
  auto & winner = nodes_[static_cast<size_t>(id)];
  const float rate = 1.0F / (10.0F * ++winner.winner_count);
  winner.position += rate * (point - winner.position);
  for (const auto & [neighbor, age] : graph_[id]) {
    (void)age;
    auto & node = nodes_[static_cast<size_t>(neighbor)];
    const float neighbor_rate = 1.0F / (100.0F * node.winner_count);
    node.position += neighbor_rate * (point - node.position);
  }
}

void ATCDT::updateEdge(int first, int second)
{
  for (auto & [neighbor, age] : graph_[first]) {
    ++age;
    graph_[neighbor][first] = age;
  }
  graph_[first][second] = 1;
  graph_[second][first] = 1;
}

float ATCDT::gmax(int id) const
{
  std::vector<float> ages;
  for (const auto & [neighbor, age] : graph_.at(id)) { (void)neighbor; ages.push_back(age); }
  if (ages.empty()) return std::numeric_limits<float>::infinity();
  std::sort(ages.begin(), ages.end());
  const auto percentile = [&ages](float p) {
    const float index = p * static_cast<float>(ages.size() - 1);
    const size_t low = static_cast<size_t>(std::floor(index));
    const size_t high = static_cast<size_t>(std::ceil(index));
    return ages[low] + (ages[high] - ages[low]) * (index - low);
  };
  const float threshold = percentile(0.75F) + percentile(0.75F) - percentile(0.25F);
  if (deleted_count_ == 0) return threshold;
  const float weight = static_cast<float>(deleted_count_) /
    static_cast<float>(deleted_count_ + ages.size());
  return deleted_mean_ * weight + threshold * (1.0F - weight);
}

void ATCDT::removeOldEdges(int id, float limit)
{
  std::vector<int> remove;
  for (const auto & [neighbor, age] : graph_[id]) if (age > limit) {
    deleted_ages_.push_back(static_cast<float>(age));
    ++deleted_count_;
    deleted_mean_ += (static_cast<float>(age) - deleted_mean_) / deleted_count_;
    remove.push_back(neighbor);
  }
  for (int neighbor : remove) { graph_[id].erase(neighbor); graph_[neighbor].erase(id); }
}

void ATCDT::estimateAttributes(int id)
{
  if (!attributes_ || graph_[id].size() < 3) return;
  Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();
  const auto center = nodes_[static_cast<size_t>(id)].position;
  for (const auto & [neighbor, age] : graph_[id]) {
    (void)age;
    const auto diff = nodes_[static_cast<size_t>(neighbor)].position - center;
    covariance += diff * diff.transpose();
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);
  if (solver.info() == Eigen::Success) {
    auto & n = nodes_[static_cast<size_t>(id)];
    n.normal = solver.eigenvectors().col(0);
    n.slope = std::acos(std::clamp(std::abs(n.normal.normalized().y()), 0.0F, 1.0F));
    n.traversable = n.slope < 30.0F * static_cast<float>(M_PI) / 180.0F;
  }
}

int ATCDT::updateByWinners(const Eigen::Vector3f & point, const std::vector<int> & candidates, int * winner)
{
  float d1, d2;
  const auto result = winners(point, &candidates, d1, d2);
  if (winner) *winner = result.first;
  if (result.first < 0 || d1 >= vigilance) return addNode(point);
  updateNode(point, result.first);
  if (result.second >= 0 && d2 < vigilance) updateEdge(result.first, result.second);
  estimateAttributes(result.first);
  removeOldEdges(result.first, gmax(result.first));
  return -1;
}

void ATCDT::process(const Eigen::Vector3f & point) { updateByWinners(point, {}); }

void Hierarchy::add(int layer, int parent, int child) { children_[layer][parent].insert(child); }

void Hierarchy::removeNode(int layer, int node)
{
  auto & relations = children_[layer];
  relations.erase(node);
  auto child_relations = children_.find(layer + 1);
  if (child_relations != children_.end())
  {
    for (auto it = child_relations->second.begin(); it != child_relations->second.end();)
    {
      it->second.erase(node);
      if (it->second.empty())
        it = child_relations->second.erase(it);
      else
        ++it;
    }
  }
}

void Hierarchy::remapNode(int layer, int from, int to)
{
  if (from == to)
    return;

  auto parent_relations = children_.find(layer);
  if (parent_relations != children_.end())
  {
    auto parent = parent_relations->second.find(from);
    if (parent != parent_relations->second.end())
    {
      auto children = std::move(parent->second);
      parent_relations->second.erase(parent);
      parent_relations->second[to].insert(children.begin(), children.end());
    }
  }

  auto child_relations = children_.find(layer + 1);
  if (child_relations != children_.end())
  {
    for (auto &[parent, children] : child_relations->second)
    {
      if (children.erase(from) > 0)
        children.insert(to);
    }
  }
}

int Hierarchy::parent(int upper_layer, int child) const
{
  const auto layer = children_.find(upper_layer);
  if (layer == children_.end())
    return -1;

  for (const auto &[candidate, children] : layer->second)
  {
    if (children.count(child) != 0)
      return candidate;
  }
  return -1;
}

std::unordered_set<int> Hierarchy::children(int layer, int parent) const
{
  const auto l = children_.find(layer);
  if (l == children_.end()) return {};
  const auto p = l->second.find(parent);
  return p == l->second.end() ? std::unordered_set<int>{} : p->second;
}

std::vector<std::tuple<int, int, int>> Hierarchy::edges() const
{
  std::vector<std::tuple<int, int, int>> result;
  for (const auto & [layer, parents] : children_) {
    for (const auto & [parent, children] : parents) {
      for (const int child : children) result.emplace_back(layer, parent, child);
    }
  }
  return result;
}

MLATC::MLATC(float base, float alpha, int lambda_points)
: base_vigilance_(base), alpha_(alpha), lambda_points_(lambda_points) { addLayer(); }

void MLATC::removeLeaf(int node_id)
{
  if (layers_.empty() || node_id < 0 || node_id >= layers_[0].nodeCount())
    return;

  int child = node_id;
  for (size_t layer_index = 0; layer_index < layers_.size(); ++layer_index)
  {
    const int upper_layer = static_cast<int>(layer_index + 1);
    const int parent = upper_layer < static_cast<int>(layers_.size())
      ? hierarchy_.parent(upper_layer, child) : -1;

    hierarchy_.removeNode(static_cast<int>(layer_index), child);
    const int last = layers_[layer_index].nodeCount() - 1;
    if (last != child)
      hierarchy_.remapNode(static_cast<int>(layer_index), last, child);
    layers_[layer_index].removeNode(child);

    if (parent < 0 ||
        layer_index + 1 >= layers_.size() ||
        !hierarchy_.children(upper_layer, parent).empty() ||
        (layer_index + 1 == layers_.size() - 1 &&
         layers_[layer_index + 1].nodeCount() <= 1))
      break;

    child = parent;
  }
}

float MLATC::vigilance(size_t layer) const
{
  return base_vigilance_ * std::pow(alpha_, static_cast<float>(layer));
}

void MLATC::addLayer()
{
  const size_t index = layers_.size();
  layers_.emplace_back(
    base_vigilance_ * std::pow(alpha_, static_cast<float>(index)),
    lambda_points_, index == 0);
}

std::vector<std::vector<int>> MLATC::hierarchicalNns(const Eigen::Vector3f & point) const
{
  std::vector<std::vector<int>> result(layers_.size());
  for (int layer = static_cast<int>(layers_.size()) - 1; layer >= 0; --layer) {
    if (layer == static_cast<int>(layers_.size()) - 1) result[static_cast<size_t>(layer)] = layers_[static_cast<size_t>(layer)].nodeIds();
    else {
      const float search_radius = [&]() {
        float total = 0.0F;
        for (size_t i = 0; i <= static_cast<size_t>(layer); ++i) total += vigilance(i);
        return total;
      }();
      for (int parent : result[static_cast<size_t>(layer + 1)]) {
        const auto children = hierarchy_.children(layer + 1, parent);
        for (int child : children) {
          if ((layers_[static_cast<size_t>(layer)].node(child).position - point).norm() <= search_radius)
            result[static_cast<size_t>(layer)].push_back(child);
        }
      }
    }
  }
  (void)point;
  return result;
}

void MLATC::expandTopLayer()
{
  if (layers_.back().nodeCount() != 2) return;
  const size_t old_index = layers_.size() - 1;
  const auto first = layers_[old_index].node(0).position;
  const auto second = layers_[old_index].node(1).position;
  addLayer();
  auto & layer = layers_.back();
  const int parent = layer.addNode(first);
  hierarchy_.add(static_cast<int>(layers_.size() - 1), parent, 0);
  layer.updateByWinners(second, layer.nodeIds());
  const int second_parent = layer.nodeCount() > 1 ? 1 : parent;
  hierarchy_.add(static_cast<int>(layers_.size() - 1), second_parent, 1);
}

void MLATC::processPoint(const Eigen::Vector3f & point)
{
  if (!point.allFinite()) return;
  auto candidates = hierarchicalNns(point);
  int child = -1;
  for (size_t layer = 0; layer < layers_.size(); ++layer) {
    int s1 = -1;
    const int id = layers_[layer].updateByWinners(point, candidates[layer], &s1);
    const int winner = id >= 0 ? id : s1;
    if (winner < 0) break;
    if (child >= 0) hierarchy_.add(static_cast<int>(layer), winner, child);
    if (id < 0) break;
    child = id;
    if (layer + 1 == layers_.size() && layers_[layer].nodeCount() == 2) { expandTopLayer(); break; }
  }
}

void MLATC::processFrame(const std::vector<Eigen::Vector3f> & points)
{
  if (points.empty()) return;
  const size_t stride = points.size() > static_cast<size_t>(lambda_points_)
    ? (points.size() + static_cast<size_t>(lambda_points_) - 1) /
      static_cast<size_t>(lambda_points_) : 1;
  for (size_t i = 0; i < points.size(); i += stride) {
    if (points[i].allFinite()) processPoint(points[i]);
  }
}

}  // namespace mlatc
