#pragma once

#include <memory>

#include "atc_core/topological_map.hpp"

namespace atc_core
{

// Scan-side KD-tree (replacement for scipy cKDTree). PCL is hidden in the .cpp.
class ScanKdTree
{
public:
  ScanKdTree();
  ~ScanKdTree();
  ScanKdTree(const ScanKdTree &) = delete;
  ScanKdTree & operator=(const ScanKdTree &) = delete;

  void build(const Cloud & cloud);

  // true if any scan point lies within `radius` of q
  bool has_point_within(const Vec3 & q, float radius) const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace atc_core