/**
 * *********************************************************
 *
 * @file: astar_polyline_planner.cpp
 * @brief: A* planner followed by strict polyline shortcutting
 *
 * ********************************************************
 */
#include <algorithm>
#include <chrono>
#include <cmath>

#include <costmap_2d/cost_values.h>

#include "common/util/log.h"
#include "path_planner/graph_planner/astar_polyline_planner.h"
#include "path_planner/graph_planner/path_clearance.h"

using namespace rmp::common::geometry;

namespace rmp::path_planner {
AStarPolylinePathPlanner::AStarPolylinePathPlanner(
    costmap_2d::Costmap2DROS* costmap_ros, double clearance_target_m, double clearance_weight)
  : PathPlanner(costmap_ros), base_planner_(costmap_ros),
    clearance_target_m_(std::isfinite(clearance_target_m) ? std::max(0.0, clearance_target_m) : 0.15),
    clearance_weight_(std::isfinite(clearance_weight) ? std::max(0.0, clearance_weight) : 4.0) {
}

bool AStarPolylinePathPlanner::plan(const Point3d& start, const Point3d& goal,
                                    Points3d* path, Points3d* expand) {
  Points3d base_path;
  const auto begin = std::chrono::steady_clock::now();
  const PathClearance clearance(*costmap_, clearance_target_m_, clearance_weight_);
  const double stamp = ros::Time::now().toSec();
  if (!base_planner_.plan(start, goal, &base_path, expand, clearance.enabled() ? &clearance : nullptr) ||
      base_path.empty()) {
    ROS_WARN("[ASTAR_CLEARANCE] t=%.3f result=no_path start=(%.3f,%.3f) goal=(%.3f,%.3f) "
             "target_m=%.3f weight=%.2f expanded=%zu",
             stamp, start.x(), start.y(), goal.x(), goal.y(),
             clearance_target_m_, clearance_weight_, expand->size());
    return false;
  }

  Points3d polyline = _polylineShortcut(base_path, clearance);
  _assignSegmentHeadings(&polyline);

  Points3d dense_path;
  const double polyline_length = _pathLength(polyline);
  double sample_step = std::max(costmap_->getResolution(), 0.05);
  if (base_path.size() > 1 && polyline_length > 1e-6) {
    sample_step = polyline_length / (base_path.size() - 1);
  }
  if (clearance.enabled()) {
    // Keep every vertex so a downstream consumer cannot cut across a bend
    // by connecting uniform samples from different segments.
    dense_path.push_back(polyline.front());
    for (size_t i = 1; i < polyline.size(); ++i) {
      const auto& a = polyline[i - 1];
      const auto& b = polyline[i];
      const double dx = b.x() - a.x();
      const double dy = b.y() - a.y();
      const int count = std::max(1, static_cast<int>(std::ceil(std::hypot(dx, dy) / sample_step)));
      for (int j = 1; j < count; ++j) {
        const double t = static_cast<double>(j) / count;
        dense_path.emplace_back(a.x() + t * dx, a.y() + t * dy);
      }
      dense_path.push_back(b);
    }
    // PathPlannerNode replaces the final pose with the exact goal.
    if (std::hypot(dense_path.back().x() - goal.x(), dense_path.back().y() - goal.y()) > 1e-6) {
      dense_path.push_back(goal);
    }
    _assignSegmentHeadings(&dense_path);
  } else if (!PathPlanner::resample(polyline, &dense_path, sample_step)) {
    dense_path = polyline;
  }
  unsigned int max_cost = 0;
  double total_cost = 0.0;
  std::size_t inflated_poses = 0;
  for (const auto& point : base_path) {
    unsigned int mx = 0;
    unsigned int my = 0;
    if (!costmap_->worldToMap(point.x(), point.y(), mx, my)) {
      continue;
    }
    const unsigned int cost = costmap_->getCost(mx, my);
    max_cost = std::max(max_cost, cost);
    total_cost += cost;
    if (cost != costmap_2d::FREE_SPACE) {
      ++inflated_poses;
    }
  }
  R_INFO << "AStarPolyline: base poses=" << base_path.size()
         << ", polyline vertices=" << polyline.size()
         << ", dense poses=" << dense_path.size()
         << ", path_cost(max/mean)=" << max_cost << "/"
         << (base_path.empty() ? 0.0 : total_cost / base_path.size())
         << ", inflated_poses=" << inflated_poses << "/" << base_path.size();

  const auto summarize = [&clearance](const Points3d& points) {
    auto stats = clearance.measure(points.front(), points.front());
    for (size_t i = 1; i < points.size(); ++i) {
      const auto segment = clearance.measure(points[i - 1], points[i]);
      stats.min_m = std::min(stats.min_m, segment.min_m);
      stats.below_target_m += segment.below_target_m;
      stats.nonfree_m += segment.nonfree_m;
    }
    return stats;
  };
  const auto base = summarize(base_path);
  // Match the goal position published by PathPlannerNode in baseline mode.
  const Point3d last_pose = dense_path.back();
  dense_path.back() = goal;
  const auto output = summarize(dense_path);
  const double elapsed_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - begin).count();
  ROS_INFO("[ASTAR_CLEARANCE] t=%.3f result=ok start=(%.3f,%.3f) goal=(%.3f,%.3f) "
           "target_m=%.3f weight=%.2f cell_m=%.3f expanded=%zu elapsed_ms=%.1f "
           "base_min_m=%.3f output_min_m=%.3f base_below_target_m=%.3f output_below_target_m=%.3f "
           "output_nonfree_m=%.3f base_length_m=%.3f output_length_m=%.3f vertices=%zu poses=%zu",
           stamp, start.x(), start.y(), goal.x(), goal.y(),
           clearance_target_m_, clearance_weight_, costmap_->getResolution(), expand->size(), elapsed_ms,
           base.min_m, output.min_m, base.below_target_m, output.below_target_m,
           output.nonfree_m, _pathLength(base_path), _pathLength(dense_path), polyline.size(), dense_path.size());
  dense_path.back() = last_pose;
  path->swap(dense_path);
  return true;
}

Points3d AStarPolylinePathPlanner::_polylineShortcut(const Points3d& path,
                                                     const PathClearance& clearance) const {
  if (path.size() < 3) {
    return path;
  }

  Points3d result;
  size_t i = 0;
  result.push_back(path.front());

  std::vector<double> prefix_cost(path.size(), 0.0);
  std::vector<double> segment_min(path.size());
  std::vector<double> span_min(path.size());
  if (clearance.enabled()) {
    for (size_t j = 1; j < path.size(); ++j) {
      const auto segment = clearance.measure(path[j - 1], path[j]);
      prefix_cost[j] = prefix_cost[j - 1] + segment.cost_m;
      segment_min[j] = segment.min_m;
    }
  }

  while (i < path.size() - 1) {
    size_t best = i + 1;
    if (clearance.enabled()) {
      span_min[i + 1] = segment_min[i + 1];
      for (size_t j = i + 2; j < path.size(); ++j) {
        span_min[j] = std::min(span_min[j - 1], segment_min[j]);
      }
    }

    for (size_t j = path.size() - 1; j > i + 1; --j) {
      if (clearance.enabled()) {
        const auto candidate = clearance.measure(path[i], path[j], span_min[j]);
        if (!candidate.free_interior || candidate.min_m < span_min[j] ||
            candidate.cost_m > prefix_cost[j] - prefix_cost[i] + 1e-6) {
          continue;
        }
      } else if (!_lineFreeOfInflation(path[i], path[j])) {
        continue;
      }
      best = j;
      break;
    }

    result.push_back(path[best]);
    i = best;
  }

  return result;
}

double AStarPolylinePathPlanner::_pathLength(const Points3d& path) const {
  double length = 0.0;
  for (size_t i = 1; i < path.size(); ++i) {
    length += std::hypot(path[i].x() - path[i - 1].x(),
                         path[i].y() - path[i - 1].y());
  }
  return length;
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
