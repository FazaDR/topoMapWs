#include "atc_core/free_area_kdtree.hpp"

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cmath>
#include <vector>

namespace atc_core
{

struct ScanKdTree::Impl
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud{new pcl::PointCloud<pcl::PointXYZ>};
  pcl::KdTreeFLANN<pcl::PointXYZ> tree;
  bool empty = true;
};

ScanKdTree::ScanKdTree()
: impl_(new Impl) {}

ScanKdTree::~ScanKdTree() = default;

void ScanKdTree::build(const Cloud & cloud)
{
  impl_->cloud->clear();
  impl_->cloud->reserve(cloud.size());
  for (const auto & p : cloud) {
    if (std::isfinite(p.x()) && std::isfinite(p.y()) && std::isfinite(p.z())) {
      impl_->cloud->push_back(pcl::PointXYZ(p.x(), p.y(), p.z()));
    }
  }
  impl_->cloud->width = static_cast<uint32_t>(impl_->cloud->size());
  impl_->cloud->height = 1;
  impl_->cloud->is_dense = true;

  impl_->empty = impl_->cloud->empty();
  if (!impl_->empty) {
    impl_->tree.setInputCloud(impl_->cloud);
  }
}

bool ScanKdTree::has_point_within(const Vec3 & q, float radius) const
{
  if (impl_->empty) {
    return false;
  }
  std::vector<int> idx(1);
  std::vector<float> sqd(1);
  const pcl::PointXYZ pt(q.x(), q.y(), q.z());
  if (impl_->tree.nearestKSearch(pt, 1, idx, sqd) > 0) {
    return sqd[0] <= radius * radius;
  }
  return false;
}

}  // namespace atc_core