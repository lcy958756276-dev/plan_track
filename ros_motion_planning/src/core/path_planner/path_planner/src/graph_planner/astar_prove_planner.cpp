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
      const double safe_dist = 0.3;   // 安全距离（米）

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
      node_new.set_g(current.g() + motion.g());
      node_new.set_id(grid2Index(node_new.x(), node_new.y()));

      // node_new in closed list
      if (closed_list.find(node_new.id()) != closed_list.end()) {
        continue;
      }

      node_new.set_pid(current.id());

      // next node hit the boundary or obstacle
      // prevent planning failed when the current within inflation
      if ((node_new.id() < 0) || (node_new.id() >= map_size_) ||
          (costmap_->getCharMap()[node_new.id()] >=
               costmap_2d::LETHAL_OBSTACLE * config_.obstacle_inflation_factor() &&
           costmap_->getCharMap()[node_new.id()] >=
               costmap_->getCharMap()[current.id()])) {
        continue;
      }

      node_new.set_h(
          std::hypot(node_new.x() - goal_node.x(), node_new.y() - goal_node.y()));

      open_list.push(node_new);
    }
  }

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
  // LETHAL_OBSTACLE = 253; threshold consistent with A* expansion logic
  return costmap_->getCharMap()[idx] >= costmap_2d::LETHAL_OBSTACLE * 0.8;
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
