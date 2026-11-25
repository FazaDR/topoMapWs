#pragma once

#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

namespace controller
{

struct Waypoint
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

// Ordered waypoint list plus the index of the one currently being pursued.
// Not thread-safe: the owner locks.
class WaypointPlan
{
public:
  void reset(std::vector<Waypoint> wps)
  {
    wps_ = std::move(wps);
    index_ = 0;
  }

  bool empty() const {return wps_.empty();}
  bool done() const {return index_ >= wps_.size();}
  std::size_t index() const {return index_;}
  std::size_t size() const {return wps_.size();}

  const Waypoint & current() const {return wps_[index_];}

  // The leg start. For the first waypoint this is the local origin (0, 0, 0).
  Waypoint previous() const {return index_ == 0 ? Waypoint{} : wps_[index_ - 1];}

  void advance()
  {
    if (!done()) {
      ++index_;
    }
  }

  // Length of the polyline from the local origin through all waypoints.
  double totalLength() const
  {
    double len = 0.0;
    Waypoint p{};
    for (const auto & w : wps_) {
      const double dx = w.x - p.x, dy = w.y - p.y, dz = w.z - p.z;
      len += std::sqrt(dx * dx + dy * dy + dz * dz);
      p = w;
    }
    return len;
  }

private:
  std::vector<Waypoint> wps_;
  std::size_t index_{0};
};

}  // namespace controller
