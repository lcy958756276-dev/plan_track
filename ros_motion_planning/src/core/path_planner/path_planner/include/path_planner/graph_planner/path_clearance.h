#ifndef RMP_PATH_PLANNER_GRAPH_PLANNER_PATH_CLEARANCE_H
#define RMP_PATH_PLANNER_GRAPH_PLANNER_PATH_CLEARANCE_H

#include <algorithm>
#include <cmath>
#include <vector>

#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>

#include "common/geometry/point.h"

namespace rmp::path_planner {
class PathClearance {
public:
  struct Segment {
    double min_m;
    double cost_m = 0.0;
    double below_target_m = 0.0;
    double nonfree_m = 0.0;
    bool free_interior = true;
  };

  PathClearance(const costmap_2d::Costmap2D& map, double target_m, double weight)
    : map_(map), target_m_(target_m), weight_(weight) {
    const int width = map_.getSizeInCellsX();
    const int height = map_.getSizeInCellsY();
    const auto* costs = map_.getCharMap();
    const double cell = map_.getResolution();
    distances_.resize(static_cast<size_t>(width) * height);

    // Approximate grid distance, capped at the preferred clearance after the
    // half-cell adjustment from occupied cell centers toward their boundary.
    // Treat unknown cells as boundary sources for the preference only.
    for (size_t id = 0; id < distances_.size(); ++id) {
      distances_[id] = costs[id] == costmap_2d::FREE_SPACE ? target_m_ + 0.5 * cell : 0.0;
    }
    const double diagonal = cell * std::sqrt(2.0);
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        const int id = y * width + x;
        double& distance = distances_[id];
        if (x > 0) distance = std::min(distance, distances_[id - 1] + cell);
        if (y == 0) continue;
        distance = std::min(distance, distances_[id - width] + cell);
        if (x > 0) distance = std::min(distance, distances_[id - width - 1] + diagonal);
        if (x + 1 < width) distance = std::min(distance, distances_[id - width + 1] + diagonal);
      }
    }
    for (int y = height - 1; y >= 0; --y) {
      for (int x = width - 1; x >= 0; --x) {
        const int id = y * width + x;
        double& distance = distances_[id];
        if (x + 1 < width) distance = std::min(distance, distances_[id + 1] + cell);
        if (y + 1 == height) continue;
        distance = std::min(distance, distances_[id + width] + cell);
        if (x > 0) distance = std::min(distance, distances_[id + width - 1] + diagonal);
        if (x + 1 < width) distance = std::min(distance, distances_[id + width + 1] + diagonal);
      }
    }
  }

  bool enabled() const { return weight_ > 0.0 && target_m_ > 0.0; }

  double multiplier(double distance) const {
    if (!enabled()) return 1.0;
    const double deficit = std::max(0.0, 1.0 - distance / target_m_);
    return 1.0 + weight_ * deficit * deficit;
  }

  double distance(int id) const {
    return std::max(0.0, distances_[id] - 0.5 * map_.getResolution());
  }

  Segment measure(const common::geometry::Point3d& a,
                  const common::geometry::Point3d& b,
                  double required_min_m = -1.0) const {
    const double dx = b.x() - a.x();
    const double dy = b.y() - a.y();
    const double length = std::hypot(dx, dy);
    const double step = 0.25 * map_.getResolution();
    const int samples = std::max(2, static_cast<int>(std::ceil(length / step)));
    const double ds = length / samples;
    Segment result{target_m_};
    for (int i = 0; i <= samples; ++i) {
      const double t = static_cast<double>(i) / samples;
      unsigned int mx, my;
      const bool in_map = map_.worldToMap(a.x() + t * dx, a.y() + t * dy, mx, my);
      const bool free = in_map && map_.getCost(mx, my) == costmap_2d::FREE_SPACE;
      const double clearance = free ? distance(my * map_.getSizeInCellsX() + mx) : 0.0;
      const bool endpoint = i == 0 || i == samples;
      const double sample_length = endpoint ? 0.5 * ds : ds;
      result.min_m = std::min(result.min_m, clearance);
      result.cost_m += sample_length * multiplier(clearance);
      if (clearance + 1e-9 < target_m_) result.below_target_m += sample_length;
      if (!free) result.nonfree_m += sample_length;
      if (!in_map || (!endpoint && !free)) result.free_interior = false;
      if (required_min_m >= 0.0 && (!result.free_interior || result.min_m < required_min_m)) {
        return result;
      }
    }
    return result;
  }

private:
  const costmap_2d::Costmap2D& map_;
  double target_m_;
  double weight_;
  std::vector<double> distances_;
};
}  // namespace rmp::path_planner

#endif
