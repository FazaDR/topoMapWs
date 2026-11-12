#pragma once

#include <Eigen/Geometry>

#include <cstdint>
#include <vector>

#include "atc_core/topological_map.hpp"

namespace atc_core
{

struct FreeAreaConfig
{
  int sectors = 24;
  float horizontal_band = 0.15f;
  float distance = 0.5f;
  float fov_degrees = 180.0f;
  float margin = 0.1f;
  UpAxis up_axis = UpAxis::Z;
};

// Yaw of the robot's +X axis projected on the horizontal plane.
// Same convention as the detector: atan2(horizontal_b, horizontal_a).
float heading_from_quaternion(const Eigen::Quaternionf & q, UpAxis up);

struct Proximity
{
  std::vector<Vec3> points;      // size = sectors
  std::vector<uint8_t> valid;    // size = sectors
  int valid_count() const;
};

class FreeAreaDetector
{
public:
  explicit FreeAreaDetector(const FreeAreaConfig & cfg = FreeAreaConfig());

  // Actual robot heading (from pose). Call before find_deletions().
  void set_heading(float heading);
  float heading() const { return heading_; }

  Proximity proximity_points(const Cloud & cloud, const Vec3 & robot_position) const;

  std::vector<uint8_t> nodes_in_free_area(
    const std::vector<Vec3> & node_positions, const Proximity & prox,
    const Vec3 & robot_position) const;

  // Ids of nodes to delete. positions / node_ids are the dense map arrays.
  std::vector<int> find_deletions(
    const std::vector<Vec3> & positions, const std::vector<int> & node_ids,
    const Cloud & cloud, const Vec3 & robot_position) const;

private:
  FreeAreaConfig cfg_;
  float fov_;
  float heading_ = 0.0f;
  int ax_a_, ax_b_, ax_up_;   // horizontal plane axes and vertical axis indices
};

}  // namespace atc_core