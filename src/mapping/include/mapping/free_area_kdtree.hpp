#pragma once

#include <vector>

#include <Eigen/Core>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace mapping
{

class FreeAreaKdTree
{
public:
  explicit FreeAreaKdTree(const std::vector<Eigen::Vector3f> &points)
  {
    cloud_.reserve(points.size());
    for (const auto &point : points)
      cloud_.emplace_back(point.x(), point.y(), point.z());
    tree_.setInputCloud(cloud_.makeShared());
  }

  bool hasPointWithin(const Eigen::Vector3f &point, float radius) const
  {
    pcl::PointXYZ query(point.x(), point.y(), point.z());
    std::vector<int> indices;
    std::vector<float> squared_distances;
    return tree_.radiusSearch(query, radius, indices, squared_distances, 1) > 0;
  }

private:
  pcl::PointCloud<pcl::PointXYZ> cloud_;
  pcl::KdTreeFLANN<pcl::PointXYZ> tree_;
};

}  // namespace mapping
