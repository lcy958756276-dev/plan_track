/**
 * *********************************************************
 *
 * @file: astar_polyline_planner.cpp
 * @brief: A* planner with clearance-aware shortcutting and corner rounding
 *
 * ********************************************************
 */
#include <algorithm>
#include <chrono>
#include <cmath>

#include <costmap_2d/cost_values.h>

#include "common/geometry/curve/bezier_curve.h"
#include "common/util/log.h"
#include "path_planner/graph_planner/astar_polyline_planner.h"
#include "path_planner/graph_planner/path_clearance.h"

using namespace rmp::common::geometry;

namespace rmp::path_planner {
namespace {
struct RoundedPath {
  Points3d points;
  size_t attempted_corners = 0;
  size_t rounded_corners = 0;
  double curved_length_m = 0.0;
  double max_unrounded_turn_rad = 0.0;
};

RoundedPath roundCorners(const Points3d& path, const PathClearance& clearance,
                         double max_trim_m, double cell_m, double sample_step) {
  RoundedPath result;
  result.points.push_back(path.front());
  BezierCurve bezier;

  for (size_t i = 1; i + 1 < path.size(); ++i) {
    const auto& a = path[i - 1];
    const auto& b = path[i];
    const auto& c = path[i + 1];
    const double in_x = b.x() - a.x(), in_y = b.y() - a.y();
    const double out_x = c.x() - b.x(), out_y = c.y() - b.y();
    const double in_length = std::hypot(in_x, in_y);
    const double out_length = std::hypot(out_x, out_y);
    const double turn = std::abs(std::atan2(in_x * out_y - in_y * out_x,
                                            in_x * out_x + in_y * out_y));
    if (turn < 1e-4) {
      result.points.push_back(b);
      continue;
    }
    ++result.attempted_corners;

    // Leave an untouched part of each straight segment even when both ends round.
    double trim = std::min({max_trim_m, 0.45 * in_length, 0.45 * out_length});
    const double min_trim = 0.25 * cell_m;
    bool accepted = false;
    for (; turn < 3.05 && trim >= min_trim; trim *= 0.5) {
      const double ux = in_x / in_length, uy = in_y / in_length;
      const double vx = out_x / out_length, vy = out_y / out_length;
      const Point3d entry(b.x() - trim * ux, b.y() - trim * uy, std::atan2(uy, ux));
      const Point3d exit(b.x() + trim * vx, b.y() + trim * vy, std::atan2(vy, vx));
      const auto original_in = clearance.measure(entry, b);
      const auto original_out = clearance.measure(b, exit);
      const double minimum = std::min(original_in.min_m, original_out.min_m);
      const double cost_limit = original_in.cost_m + original_out.cost_m;

      // Equally spaced control triples on each straight yield zero endpoint
      // acceleration and curvature. Only the corner receives a quintic curve.
      const double handle = trim / 3.0;
      const Points2d controls{
          {entry.x(), entry.y()},
          {b.x() - 2.0 * handle * ux, b.y() - 2.0 * handle * uy},
          {b.x() - handle * ux, b.y() - handle * uy},
          {b.x() + handle * vx, b.y() + handle * vy},
          {b.x() + 2.0 * handle * vx, b.y() + 2.0 * handle * vy},
          {exit.x(), exit.y()}};
      Points2d tangents;
      for (size_t j = 1; j < controls.size(); ++j) {
        tangents.emplace_back(controls[j].x() - controls[j - 1].x(),
                              controls[j].y() - controls[j - 1].y());
      }
      const double curve_step = std::min(0.5 * cell_m, sample_step);
      const int samples = std::max(8, static_cast<int>(std::ceil(10.0 * handle / curve_step)));
      Points3d arc{entry};
      double cost = 0.0, length = 0.0;
      for (int j = 1; j <= samples; ++j) {
        const double t = static_cast<double>(j) / samples;
        const auto p = bezier.bezier(t, controls);
        const auto tangent = bezier.bezier(t, tangents);
        const Point3d next = j == samples ? exit : Point3d(p.x(), p.y(),
                                                         std::atan2(tangent.y(), tangent.x()));
        const auto segment = clearance.measure(arc.back(), next, minimum);
        cost += segment.cost_m;
        if (!segment.free_interior || segment.nonfree_m > 0.0 || segment.min_m < minimum ||
            cost > cost_limit + 1e-6) {
          break;
        }
        length += std::hypot(next.x() - arc.back().x(), next.y() - arc.back().y());
        arc.push_back(next);
      }
      if (arc.size() == static_cast<size_t>(samples + 1)) {
        result.points.insert(result.points.end(), arc.begin(), arc.end());
        ++result.rounded_corners;
        result.curved_length_m += length;
        accepted = true;
        break;
      }
    }
    if (!accepted) {
      result.points.push_back(b);
      result.max_unrounded_turn_rad = std::max(result.max_unrounded_turn_rad, turn);
    }
  }

  if (path.size() > 1) result.points.push_back(path.back());
  return result;
}

double maxTurnDegrees(const Points3d& path) {
  double maximum = 0.0;
  for (size_t i = 1; i + 1 < path.size(); ++i) {
    const double ax = path[i].x() - path[i - 1].x(), ay = path[i].y() - path[i - 1].y();
    const double bx = path[i + 1].x() - path[i].x(), by = path[i + 1].y() - path[i].y();
    maximum = std::max(maximum, std::abs(std::atan2(ax * by - ay * bx, ax * bx + ay * by)));
  }
  return maximum * 180.0 / std::acos(-1.0);
}
}  // namespace

AStarPolylinePathPlanner::AStarPolylinePathPlanner(
    costmap_2d::Costmap2DROS* costmap_ros, double clearance_target_m, double clearance_weight,
    double corner_trim_m)
  : PathPlanner(costmap_ros), base_planner_(costmap_ros),
    clearance_target_m_(std::isfinite(clearance_target_m) ? std::max(0.0, clearance_target_m) : 0.15),
    clearance_weight_(std::isfinite(clearance_weight) ? std::max(0.0, clearance_weight) : 4.0),
    corner_trim_m_(std::isfinite(corner_trim_m) ? std::max(0.0, corner_trim_m) : 0.20) {
}

bool AStarPolylinePathPlanner::plan(const Point3d& start, const Point3d& goal,
                                    Points3d* path, Points3d* expand) {
  Points3d base_path;
  const auto begin = std::chrono::steady_clock::now();
  const PathClearance clearance(*costmap_, clearance_target_m_, clearance_weight_);
  const bool clearance_enabled = clearance.enabled();
  const double stamp = ros::Time::now().toSec();
  if (!base_planner_.plan(start, goal, &base_path, expand, clearance_enabled ? &clearance : nullptr,
                         !clearance_enabled) ||
      base_path.empty()) {
    ROS_WARN("[ASTAR_CLEARANCE] t=%.3f result=no_path start=(%.3f,%.3f) goal=(%.3f,%.3f) "
             "target_m=%.3f weight=%.2f expanded=%zu",
             stamp, start.x(), start.y(), goal.x(), goal.y(),
             clearance_target_m_, clearance_weight_, expand->size());
    return false;
  }

  Points3d shortcut_input = base_path;
  if (clearance_enabled) {
    // Include exact endpoints during shortcutting so a grid-center connector
    // cannot introduce a final turn after smoothing has finished.
    if (base_path.size() == 1) {
      shortcut_input = {goal};
      if (std::hypot(start.x() - goal.x(), start.y() - goal.y()) > 1e-6) {
        shortcut_input.insert(shortcut_input.begin(), start);
      }
    } else {
      if (std::hypot(start.x() - base_path.front().x(), start.y() - base_path.front().y()) > 1e-6) {
        shortcut_input.insert(shortcut_input.begin(), start);
      } else {
        shortcut_input.front() = start;
      }
      if (std::hypot(goal.x() - base_path.back().x(), goal.y() - base_path.back().y()) > 1e-6) {
        shortcut_input.push_back(goal);
      } else {
        shortcut_input.back() = goal;
      }
    }
  }
  Points3d polyline = _polylineShortcut(shortcut_input, clearance);
  _assignSegmentHeadings(&polyline);

  const double polyline_length = _pathLength(polyline);
  double sample_step = costmap_->getResolution();
  if (!clearance_enabled && base_path.size() > 1 && polyline_length > 1e-6) {
    sample_step = polyline_length / (base_path.size() - 1);
  }
  const bool smooth_corners = clearance_enabled && corner_trim_m_ > 0.0;
  const auto rounded = smooth_corners
      ? roundCorners(polyline, clearance, corner_trim_m_, costmap_->getResolution(), sample_step)
      : RoundedPath{};
  const auto& reference = smooth_corners ? rounded.points : polyline;

  Points3d dense_path;
  if (clearance_enabled) {
    // Keep every vertex so a downstream consumer cannot cut across a bend
    // by connecting uniform samples from different segments.
    dense_path.push_back(reference.front());
    for (size_t i = 1; i < reference.size(); ++i) {
      const auto& a = reference[i - 1];
      const auto& b = reference[i];
      const double dx = b.x() - a.x();
      const double dy = b.y() - a.y();
      const int count = std::max(1, static_cast<int>(std::ceil(std::hypot(dx, dy) / sample_step)));
      for (int j = 1; j < count; ++j) {
        const double t = static_cast<double>(j) / count;
        dense_path.emplace_back(a.x() + t * dx, a.y() + t * dy, std::atan2(dy, dx));
      }
      dense_path.push_back(b);
    }
  } else {
    if (!PathPlanner::resample(polyline, &dense_path, sample_step)) {
      dense_path = polyline;
    }
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
           "output_nonfree_m=%.3f base_length_m=%.3f output_length_m=%.3f vertices=%zu poses=%zu "
           "corner_trim_m=%.3f rounded=%zu/%zu polyline_max_turn_deg=%.1f rounded_max_turn_deg=%.1f "
           "curved_length_m=%.3f max_unrounded_turn_deg=%.1f",
           stamp, start.x(), start.y(), goal.x(), goal.y(),
           clearance_target_m_, clearance_weight_, costmap_->getResolution(), expand->size(), elapsed_ms,
           base.min_m, output.min_m, base.below_target_m, output.below_target_m,
           output.nonfree_m, _pathLength(base_path), _pathLength(dense_path), polyline.size(), dense_path.size(),
           smooth_corners ? corner_trim_m_ : 0.0, rounded.rounded_corners, rounded.attempted_corners,
           maxTurnDegrees(polyline), maxTurnDegrees(reference), rounded.curved_length_m,
           rounded.max_unrounded_turn_rad * 180.0 / std::acos(-1.0));
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
  const double cell_m = costmap_->getResolution();
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
        // Allow sub-cell clearance differences and quadrature noise to favor
        // a straight shortcut. Never lower an already narrow bottleneck.
        const double minimum = std::min(span_min[j],
            std::max(0.5 * cell_m, span_min[j] - 0.25 * cell_m));
        const auto candidate = clearance.measure(path[i], path[j], minimum);
        if (!candidate.free_interior || candidate.min_m < minimum ||
            candidate.cost_m > prefix_cost[j] - prefix_cost[i] + 0.5 * cell_m) {
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
