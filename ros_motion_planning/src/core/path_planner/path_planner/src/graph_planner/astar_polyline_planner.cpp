/**
 * *********************************************************
 *
 * @file: astar_polyline_planner.cpp
 * @brief: A* planner followed by strict polyline shortcutting
 *
 * ********************************************************
 */
#include <algorithm>
#include <cmath>

#include <costmap_2d/cost_values.h>

#include "common/util/log.h"
#include "path_planner/graph_planner/astar_polyline_planner.h"

using namespace rmp::common::geometry;

namespace rmp::path_planner {
AStarPolylinePathPlanner::AStarPolylinePathPlanner(
    costmap_2d::Costmap2DROS* costmap_ros)
  : PathPlanner(costmap_ros), base_planner_(costmap_ros) {
}

bool AStarPolylinePathPlanner::plan(const Point3d& start, const Point3d& goal,
                                    Points3d* path, Points3d* expand) {
  Points3d base_path;
  if (!base_planner_.plan(start, goal, &base_path, expand)) {
    return false;
  }

  Points3d polyline = _polylineShortcut(base_path);
  _assignSegmentHeadings(&polyline);
  *path = polyline;

  R_INFO << "AStarPolyline: base poses=" << base_path.size()
         << ", polyline poses=" << path->size();
  return !path->empty();
}

Points3d AStarPolylinePathPlanner::_polylineShortcut(const Points3d& path) const {
  if (path.size() < 3) {
    return path;
  }

  Points3d result;
  size_t i = 0;
  result.push_back(path.front());

  while (i < path.size() - 1) {
    size_t best = i + 1;

    for (size_t j = path.size() - 1; j > i + 1; --j) {
      if (_lineFreeOfInflation(path[i], path[j])) {
        best = j;
        break;
      }
    }

    result.push_back(path[best]);
    i = best;
  }

  return result;
}

bool AStarPolylinePathPlanner::_lineFreeOfInflation(const Point3d& a,
                                                    const Point3d& b) const {
  const double dx = b.x() - a.x();
  const double dy = b.y() - a.y();
  const double length = std::hypot(dx, dy);
  const double step = std::max(costmap_->getResolution() * 0.5, 0.01);
  const int samples = std::max(2, static_cast<int>(std::ceil(length / step)));

  for (int i = 1; i < samples; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(samples);
    const double wx = a.x() + t * dx;
    const double wy = a.y() + t * dy;
    if (!_worldPointFree(wx, wy)) {
      return false;
    }
  }

  return true;
}

bool AStarPolylinePathPlanner::_worldPointFree(double wx, double wy) const {
  unsigned int mx = 0;
  unsigned int my = 0;
  if (!costmap_->worldToMap(wx, wy, mx, my)) {
    return false;
  }

  const unsigned char cost = costmap_->getCost(mx, my);
  return cost == costmap_2d::FREE_SPACE;
}

void AStarPolylinePathPlanner::_assignSegmentHeadings(Points3d* path) const {
  if (path == nullptr || path->empty()) {
    return;
  }

  for (size_t i = 0; i + 1 < path->size(); ++i) {
    const double dx = (*path)[i + 1].x() - (*path)[i].x();
    const double dy = (*path)[i + 1].y() - (*path)[i].y();
    (*path)[i].setTheta(std::atan2(dy, dx));
  }

  if (path->size() >= 2) {
    path->back().setTheta((*path)[path->size() - 2].theta());
  }
}
}  // namespace rmp::path_planner
