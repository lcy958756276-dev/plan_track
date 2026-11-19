/**
 * *********************************************************
 *
 * @file: astar_prove_planner.cpp
 * @brief: Contains the A* with path smoothing planner class
 * @author: Yang Haodong
 * @date: 2023-12-12
 * @version: 1.2
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include <queue>
#include <vector>
#include <unordered_set>
#include <cmath>
#include <algorithm>

#include <costmap_2d/cost_values.h>

#include "common/structure/node.h"
#include "path_planner/graph_planner/astar_prove_planner.h"
#include "path_planner/graph_planner/path_clearance.h"

using namespace rmp::common::geometry;

namespace rmp::path_planner {
std::vector<AStarProvePathPlanner::Node> AStarProvePathPlanner::motions_ = {
  { 0, 1, 1.0 },           { 1, 0, 1.0 },
  { 0, -1, 1.0 },          { -1, 0, 1.0 },
  { 1, 1, std::sqrt(2) },  { 1, -1, std::sqrt(2) },
  { -1, 1, std::sqrt(2) }, { -1, -1, std::sqrt(2) },
};

/**
 * @brief Construct a new AStarProve object
 * @param costmap   the environment for path planning
 */
AStarProvePathPlanner::AStarProvePathPlanner(costmap_2d::Costmap2DROS* costmap_ros)
  : PathPlanner(costmap_ros) {};

/**
 * @brief A* implementation with path smoothing
 * @param start          start node
 * @param goal           goal node
 * @param path           The resulting smoothed path in (x, y, theta)
 * @param expand         containing the node been search during the process
 * @return true if path found, else false
 */
bool AStarProvePathPlanner::plan(const Point3d& start, const Point3d& goal, Points3d* path,
                                  Points3d* expand) {
  return plan(start, goal, path, expand, nullptr);
}

bool AStarProvePathPlanner::plan(const Point3d& start, const Point3d& goal, Points3d* path,
                                Points3d* expand, const PathClearance* clearance) {
  double m_start_x, m_start_y, m_goal_x, m_goal_y;
  if ((!validityCheck(start.x(), start.y(), m_start_x, m_start_y)) ||
      (!validityCheck(goal.x(), goal.y(), m_goal_x, m_goal_y))) {
    return false;
  }

  Node start_node(m_start_x, m_start_y);
  Node goal_node(m_goal_x, m_goal_y);
  start_node.set_id(grid2Index(start_node.x(), start_node.y()));
  goal_node.set_id(grid2Index(goal_node.x(), goal_node.y()));

  // clear vector
  path->clear();
  expand->clear();

  // open list and closed list
  std::priority_queue<Node, std::vector<Node>, Node::compare_cost> open_list;
  std::unordered_map<int, Node> closed_list;
  std::size_t rejected_boundary_or_cost = 0;
  std::size_t rejected_footprint = 0;

  open_list.push(start_node);

  // main process
  while (!open_list.empty()) {
    // pop current node from open list
    auto current = open_list.top();
    open_list.pop();

    // current node does not exist in closed list
    if (closed_list.find(current.id()) != closed_list.end())
      continue;

    closed_list.insert(std::make_pair(current.id(), current));
    expand->emplace_back(current.x(), current.y());

    // goal found
    if (current == goal_node) {
      const auto& backtrace =
          _convertClosedListToPath<Node>(closed_list, start_node, goal_node);

      Points3d raw_path;
      for (auto iter = backtrace.rbegin(); iter != backtrace.rend(); ++iter) {
        // convert to world frame
        double wx, wy;
        costmap_->mapToWorld(iter->x(), iter->y(), wx, wy);
        raw_path.emplace_back(wx, wy);
      }

      // smooth path using inertial smoothing (from matlab.txt)
      // p_new = alpha * (p_prev + p_next) / 2 + (1 - alpha) * p_curr
      // with collision check against costmap
      Points3d smooth_path = raw_path;
      const double alpha = 0.6;       // 平滑权重
      const int iterations = 5;       // 迭代次数

      for (int iter = 0; iter < iterations; ++iter) {
        for (size_t i = 1; i < smooth_path.size() - 1; ++i) {
          double px = smooth_path[i-1].x(), py = smooth_path[i-1].y();
          double cx = smooth_path[i].x(),   cy = smooth_path[i].y();
          double nx = smooth_path[i+1].x(), ny = smooth_path[i+1].y();

          // inertial smoothing
          double new_x = alpha * (px + nx) / 2.0 + (1.0 - alpha) * cx;
          double new_y = alpha * (py + ny) / 2.0 + (1.0 - alpha) * cy;

          // collision check: 与 A* 逻辑一致，过滤膨胀层内的高代价格子
          double mx, my;
          if (world2Map(new_x, new_y, mx, my)) {
            int idx = grid2Index((int)mx, (int)my);
            if (idx >= 0 && idx < map_size_ &&
                costmap_->getCharMap()[idx] <
                    costmap_2d::LETHAL_OBSTACLE * config_.obstacle_inflation_factor()) {
              if (clearance) {
                const Point3d candidate(new_x, new_y);
                const auto old_in = clearance->measure(smooth_path[i-1], smooth_path[i]);
                const auto old_out = clearance->measure(smooth_path[i], smooth_path[i+1]);
                const auto new_in = clearance->measure(smooth_path[i-1], candidate);
                const auto new_out = clearance->measure(candidate, smooth_path[i+1]);
                if (costmap_->getCharMap()[idx] != costmap_2d::FREE_SPACE ||
                    !new_in.free_interior || !new_out.free_interior ||
                    std::min(new_in.min_m, new_out.min_m) < std::min(old_in.min_m, old_out.min_m) ||
                    new_in.cost_m + new_out.cost_m > old_in.cost_m + old_out.cost_m + 1e-6) {
                  continue;
                }
              }
              smooth_path[i].setX(new_x);
              smooth_path[i].setY(new_y);
            }
          }
        }
      }

      // shortcut optimization: remove detours by straight-line shortcuts (from matlab.txt Stage 3)
      // shortcutOptimize(smooth_path);  // 暂时注释

      *path = smooth_path;

      return true;
    }

    // explore neighbor of current node
    for (const auto& motion : motions_) {
      // explore a new node
      auto node_new = current + motion;
      if (node_new.x() < 0 || node_new.y() < 0 ||
          node_new.x() >= static_cast<int>(costmap_->getSizeInCellsX()) ||
          node_new.y() >= static_cast<int>(costmap_->getSizeInCellsY())) {
        ++rejected_boundary_or_cost;
        continue;
      }
      node_new.set_id(grid2Index(node_new.x(), node_new.y()));

      // node_new in closed list
      if (closed_list.find(node_new.id()) != closed_list.end()) {
        continue;
      }

      node_new.set_pid(current.id());

      // next node hit the boundary or obstacle
      // prevent planning failed when the current within inflation
      if ((node_new.id() < 0) || (node_new.id() >= map_size_)) {
        ++rejected_boundary_or_cost;
        continue;
      }
      const auto node_cost = costmap_->getCharMap()[node_new.id()];
      if (node_cost != costmap_2d::NO_INFORMATION &&
          node_cost >= costmap_2d::LETHAL_OBSTACLE * config_.obstacle_inflation_factor() &&
          node_cost >= costmap_->getCharMap()[current.id()]) {
        ++rejected_boundary_or_cost;
        continue;
      }

      // Collision clearance belongs to the configured global costmap.  Do not
      // apply a second, hard-coded footprint test here: it can contradict the
      // costmap footprint/inflation settings and disconnect otherwise free cells.

      const double multiplier = clearance ? clearance->multiplier(clearance->distance(node_new.id())) : 1.0;
      node_new.set_g(current.g() + motion.g() * multiplier);
      node_new.set_h(
          std::hypot(node_new.x() - goal_node.x(), node_new.y() - goal_node.y()));

      open_list.push(node_new);
    }
  }

  R_WARN << "AStarProve: no path; expanded=" << closed_list.size()
         << ", rejected_boundary_or_cost=" << rejected_boundary_or_cost
         << ", rejected_footprint=" << rejected_footprint
         << ", start_grid=(" << m_start_x << ", " << m_start_y << ")"
         << ", goal_grid=(" << m_goal_x << ", " << m_goal_y << ")";
  return false;
}
// ════════════════════════════════════════════════════════════
// Collision check for a single world point (costmap-based)
// ════════════════════════════════════════════════════════════
bool AStarProvePathPlanner::pointInCollision(double px, double py) {
  double mx, my;
  if (!world2Map(px, py, mx, my))
    return true;  // outside map → collision
  int ix = static_cast<int>(mx);
  int iy = static_cast<int>(my);
  if (ix < 0 || ix >= nx_ || iy < 0 || iy >= ny_)
    return true;
  unsigned int idx = iy * nx_ + ix;
  // Unknown cells are traversable in the static-map debug mode; lethal cells are not.
  const auto cost = costmap_->getCharMap()[idx];
  return cost != costmap_2d::NO_INFORMATION &&
         cost >= costmap_2d::LETHAL_OBSTACLE * 0.8;
}

bool AStarProvePathPlanner::footprintFree(double px, double py, double yaw) const {
  constexpr double kHalfLength = 0.30 + 0.02;
  constexpr double kHalfWidth = 0.15 + 0.02;
  const double step = std::max(0.5 * costmap_->getResolution(), 0.01);
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);

  for (double local_x = -kHalfLength; local_x <= kHalfLength + 1e-9;
       local_x += step) {
    for (double local_y = -kHalfWidth; local_y <= kHalfWidth + 1e-9;
         local_y += step) {
      const double wx = px + local_x * cos_yaw - local_y * sin_yaw;
      const double wy = py + local_x * sin_yaw + local_y * cos_yaw;
      unsigned int mx = 0;
      unsigned int my = 0;
      if (!costmap_->worldToMap(wx, wy, mx, my)) {
        return false;
      }
      const unsigned char cost = costmap_->getCost(mx, my);
      if (cost != costmap_2d::NO_INFORMATION &&
          cost >= costmap_2d::LETHAL_OBSTACLE) {
        return false;
      }
    }
  }
  return true;
}

// ════════════════════════════════════════════════════════════
// Shortcut optimization (Stage 3 from matlab.txt)
// For each non-adjacent pair (i, j), check straight-line
// feasibility with adaptive sampling, cut out intermediate
// waypoints if safe.
// ════════════════════════════════════════════════════════════
void AStarProvePathPlanner::shortcutOptimize(Points3d& path) {
  if (path.size() < 3) return;

  size_t i = 0;
  while (i < path.size() - 2) {
    size_t j = i + 2;
    bool changed = false;

    while (j < path.size()) {
      double seg_len = std::hypot(path[j].x() - path[i].x(),
                                   path[j].y() - path[i].y());
      // adaptive sampling density (match matlab: max(ceil(seg_len/0.3), 12))
      int N = std::max(static_cast<int>(std::ceil(seg_len / 0.3)), 12);

      bool collide = false;
      for (int t = 1; t <= N; ++t) {
        double frac = static_cast<double>(t) / N;
        double cx = path[i].x() + frac * (path[j].x() - path[i].x());
        double cy = path[i].y() + frac * (path[j].y() - path[i].y());
        if (pointInCollision(cx, cy)) {
          collide = true;
          break;
        }
      }

      if (!collide) {
        // straight line is safe → cut out intermediate points
        path.erase(path.begin() + i + 1, path.begin() + j);
        changed = true;
        j = i + 2;  // re-scan from the same i
      } else {
        ++j;
      }
    }

    if (!changed) ++i;
  }
}

}  // namespace rmp
