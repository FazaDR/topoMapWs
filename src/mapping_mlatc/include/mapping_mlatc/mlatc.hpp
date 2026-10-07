#pragma once

#include <Eigen/Core>
#include <cstddef>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace mlatc
{

struct Node
{
  Eigen::Vector3f position;
  Eigen::Vector3f normal = Eigen::Vector3f::Zero();
  float slope = 0.0F;
  bool traversable = false;
  bool contour = false;
  int winner_count = 1;
};

class ATCDT
{
public:
  ATCDT(float vigilance, int lambda_points, bool attributes);
  std::vector<int> nodeIds() const;
  const Node & node(int id) const;
  std::vector<std::pair<int, int>> edges() const;
  void process(const Eigen::Vector3f & point);
  int addNode(const Eigen::Vector3f & point);
  int updateByWinners(const Eigen::Vector3f & point, const std::vector<int> & candidates);
  int nodeCount() const { return static_cast<int>(nodes_.size()); }
  float vigilance;

private:
  std::vector<Node> nodes_;
  std::unordered_map<int, std::unordered_map<int, int>> graph_;
  std::vector<float> deleted_ages_;
  int deleted_count_ = 0;
  float deleted_mean_ = 0.0F;
  bool attributes_;
  int next_id_ = 0;
  std::pair<int, int> winners(const Eigen::Vector3f &, const std::vector<int> * candidates,
                              float & d1, float & d2) const;
  void updateNode(const Eigen::Vector3f &, int);
  void updateEdge(int, int);
  float gmax(int) const;
  void removeOldEdges(int, float);
  void estimateAttributes(int);
};

class Hierarchy
{
public:
  void add(int upper_layer, int parent, int child);
  std::unordered_set<int> children(int upper_layer, int parent) const;

private:
  std::unordered_map<int, std::unordered_map<int, std::unordered_set<int>>> children_;
};

class MLATC
{
public:
  MLATC(float base_vigilance = 0.5F, float alpha = 4.0F, int lambda_points = 4000);
  void processFrame(const std::vector<Eigen::Vector3f> & points);
  const std::vector<ATCDT> & layers() const { return layers_; }
  float vigilance(size_t layer) const;

private:
  float base_vigilance_;
  float alpha_;
  int lambda_points_;
  std::vector<ATCDT> layers_;
  Hierarchy hierarchy_;
  void addLayer();
  void processPoint(const Eigen::Vector3f & point);
  std::vector<std::vector<int>> hierarchicalNns(const Eigen::Vector3f & point) const;
  void expandTopLayer();
};

}  // namespace mlatc
