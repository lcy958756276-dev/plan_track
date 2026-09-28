/**
 * *********************************************************
 *
 * @file: proved_apf_planner.cpp
 * @brief: Improved APF global planner with escape controller
 *         and path smoothing. Adapted from MATLAB simulation.
 *
 * @author: Adapted from MATLAB simulation
 * @date: 2026-06-25
 * @version: 1.0
 *
 * ********************************************************
 */
#include <cmath>
#include <limits>
#include <vector>
#include <algorithm>

#include <costmap_2d/cost_values.h>

#include "common/math/math_helper.h"
#include "path_planner/sample_planner/proved_apf_planner.h"

namespace rmp::path_planner {

ProvedAPFPathPlanner::ProvedAPFPathPlanner(costmap_2d::Costmap2DROS* costmap_ros)
  : PathPlanner(costmap_ros)
  , k_att_(2.0)
  , k_rep_(80.0)
  , d0_(1.0)
  , step_max_(0.3)
  , step_min_(0.05)
  , safe_dist_(0.3)
  , max_iter_(5000)
  , goal_tolerance_(0.1)
  , stall_window_size_(6)
  , stall_threshold_(0.05)
  , escape_scan_range_(1.5)
  , escape_samples_(24)
  , smooth_alpha_(0.6)
  , smooth_beta_(0.3)
  , loop_threshold_(1.5)
  , shortcut_resolution_(0.3) {
}

bool ProvedAPFPathPlanner::plan(const common::geometry::Point3d& start,
                                 const common::geometry::Point3d& goal,
                                 common::geometry::Points3d* path,
                                 common::geometry::Points3d* expand) {
  path->clear();
  expand->clear();

  double sx = start.x(), sy = start.y();
  double gx = goal.x(), gy = goal.y();

  // ── Check start/goal validity ──
  {
    double mx, my;
    if (!validityCheck(sx, sy, mx, my) || !validityCheck(gx, gy, mx, my)) {
      R_WARN << "Start or goal is outside the costmap.";
      return false;
    }
  }

  // ── State ──
  double px = sx, py = sy;
  common::geometry::Points3d raw_path;
  raw_path.emplace_back(px, py, 0.0);

  bool escape_mode = false;
  double escape_ox = 0.0, escape_oy = 0.0;

  // ── Stagnation detection ──
  std::vector<double> progress_hist;

  // ── Main loop ──
  for (int iter = 0; iter < max_iter_; ++iter) {
    // Check if reached goal
    double dist_to_goal = std::hypot(gx - px, gy - py);
    if (dist_to_goal < goal_tolerance_) {
      raw_path.emplace_back(gx, gy, 0.0);
      break;
    }

    // ── 1. Compute APF force ──
    auto [fx, fy] = computeForce(px, py, gx, gy);
    double f_norm = std::hypot(fx, fy);

    // ── 2. Stagnation detection ──
    progress_hist.push_back(dist_to_goal);
    bool stalled = false;
    if (progress_hist.size() > static_cast<size_t>(stall_window_size_)) {
      double delta = std::abs(progress_hist.back() -
          progress_hist[progress_hist.size() - 1 - stall_window_size_]);
      if (delta < stall_threshold_) {
        stalled = true;
      }
    }

    // ── 3. Enter escape mode if stuck ──
    if (!escape_mode && (stalled || f_norm < 0.01)) {
      escape_mode = true;
      escape_ox = px;
      escape_oy = py;
    }

    // ── 4. Choose direction ──
    double dx, dy;
    if (escape_mode) {
      double best_theta = escapeController(px, py, raw_path,
                                            gx, gy, escape_ox, escape_oy, fx, fy);
      dx = std::cos(best_theta);
      dy = std::sin(best_theta);
    } else {
      if (f_norm > 1e-9) {
        dx = fx / f_norm;
        dy = fy / f_norm;
      } else {
        double theta = std::atan2(gy - py, gx - px);
        dx = std::cos(theta);
        dy = std::sin(theta);
      }
    }

    // ── 5. Select safe step ──
    double step = selectSafeStep(px, py, dx, dy);
    if (step < step_min_) {
      continue;  // can't move, try next iteration
    }

    // ── 6. Update position ──
    px += step * dx;
    py += step * dy;
    raw_path.emplace_back(px, py, 0.0);

    // ── 7. Check escape exit condition ──
    if (escape_mode) {
      auto [fnx, fny] = computeForce(px, py, gx, gy);
      double back_x = escape_ox - px;
      double back_y = escape_oy - py;
      double dot = fnx * back_x + fny * back_y;
      if (dot < 0.0) {
        escape_mode = false;
      }
    }
  }

  // ── Check if path was found ──
  if (raw_path.size() < 2) {
    return false;
  }

  // ── 8. Path smoothing (3 stages) ──
  smoothPath(raw_path);

  // ── Output ──
  *path = raw_path;

  return true;
}

// ════════════════════════════════════════════════════════════
// APF Force Calculation
// ════════════════════════════════════════════════════════════
std::pair<double, double> ProvedAPFPathPlanner::computeForce(
    double px, double py, double gx, double gy) {
  // Attractive force
  double fax = k_att_ * (gx - px);
  double fay = k_att_ * (gy - py);

  // Repulsive force
  double dir_x, dir_y;
  double d = nearestObstacleInfo(px, py, dir_x, dir_y);

  double frx = 0.0, fry = 0.0;
  if (d < d0_ && d > 1e-6) {
    double mag = k_rep_ * (1.0 / d - 1.0 / d0_) * (1.0 / (d * d));
    frx = mag * dir_x;
    fry = mag * dir_y;
  }

  return {fax + frx, fay + fry};
}

// ════════════════════════════════════════════════════════════
// Nearest obstacle search using costmap
// ════════════════════════════════════════════════════════════
double ProvedAPFPathPlanner::nearestObstacleInfo(
    double px, double py, double& dir_x, double& dir_y) {
  // Search range in costmap cells
  int range_cells = static_cast<int>(d0_ / costmap_->getResolution()) + 1;
  double mxd, myd;
  if (!world2Map(px, py, mxd, myd)) {
    dir_x = 0.0; dir_y = 0.0;
    return d0_ + 1.0;  // outside map, no repulsion
  }
  int mx = static_cast<int>(mxd);
  int my = static_cast<int>(myd);

  double min_dist = d0_ + 1.0;
  double nearest_x = px, nearest_y = py;
  bool found = false;

  // Search in a square window around the robot
  for (int dy = -range_cells; dy <= range_cells; ++dy) {
    for (int dx = -range_cells; dx <= range_cells; ++dx) {
      int cx = mx + dx;
      int cy = my + dy;
      if (cx < 0 || cx >= nx_ || cy < 0 || cy >= ny_)
        continue;

      unsigned int idx = cy * nx_ + cx;
      // LETHAL_OBSTACLE = 253, anything near lethal is an obstacle
      if (costmap_->getCharMap()[idx] >= costmap_2d::LETHAL_OBSTACLE * 0.5) {
        double wx, wy;
        map2World(cx, cy, wx, wy);
        double d = std::hypot(wx - px, wy - py);
        if (d < min_dist) {
          min_dist = d;
          nearest_x = wx;
          nearest_y = wy;
          found = true;
        }
      }
    }
  }

  if (found) {
    double dx = px - nearest_x;
    double dy = py - nearest_y;
    double n = std::hypot(dx, dy);
    if (n > 1e-9) {
      dir_x = dx / n;
      dir_y = dy / n;
    } else {
      dir_x = 0.0; dir_y = 0.0;
    }
    return min_dist;
  }

  dir_x = 0.0; dir_y = 0.0;
  return d0_ + 1.0;
}

// ════════════════════════════════════════════════════════════
// Escape Controller
// ════════════════════════════════════════════════════════════
double ProvedAPFPathPlanner::escapeController(
    double px, double py, const common::geometry::Points3d& path,
    double gx, double gy, double escape_ox, double escape_oy,
    double fx, double fy) {
  double theta_goal = std::atan2(gy - py, gx - px);

  // Previous direction
  double theta_prev = theta_goal;
  if (path.size() > 1) {
    auto& p1 = path[path.size() - 1];
    auto& p0 = path[path.size() - 2];
    theta_prev = std::atan2(p1.y() - p0.y(), p1.x() - p0.x());
  }

  // Force direction
  double theta_force = std::atan2(fy, fx);

  double best_theta = 0.0;
  double best_score = -std::numeric_limits<double>::infinity();

  double angle_step = 2.0 * M_PI / escape_samples_;

  for (int i = 0; i < escape_samples_; ++i) {
    double theta = i * angle_step;

    if (!isFree(px, py, theta, escape_scan_range_))
      continue;

    double to_goal = std::cos(theta - theta_goal);
    double continuity = std::cos(theta - theta_prev);
    double anti_stuck = std::cos(theta - std::atan2(escape_oy - py, escape_ox - px));
    double anti_force = std::cos(theta - theta_force);

    double dist_to_stuck = std::hypot(px - escape_ox, py - escape_oy);

    double J;
    if (dist_to_stuck < 2.0) {
      J = 1.8 * to_goal + 0.8 * continuity - 1.5 * anti_stuck - 1.0 * anti_force;
    } else {
      J = 2.0 * to_goal + 1.2 * continuity - 0.8 * anti_force;
    }

    if (J > best_score) {
      best_score = J;
      best_theta = theta;
    }
  }

  return best_theta;
}

// ════════════════════════════════════════════════════════════
// Check if a direction is obstacle-free for a given range
// ════════════════════════════════════════════════════════════
bool ProvedAPFPathPlanner::isFree(double px, double py, double theta, double range) {
  int num_samples = 8;
  for (int i = 1; i <= num_samples; ++i) {
    double t = (static_cast<double>(i) / num_samples) * range;
    double cx = px + t * std::cos(theta);
    double cy = py + t * std::sin(theta);

    if (pointInCollision(cx, cy))
      return false;
  }
  return true;
}

// ════════════════════════════════════════════════════════════
// Select safe step size (try max first, reduce if collision)
// ════════════════════════════════════════════════════════════
double ProvedAPFPathPlanner::selectSafeStep(double px, double py,
                                             double dx, double dy) {
  // Try step sizes: max, mid, min
  for (double s : {step_max_, (step_max_ + step_min_) / 2.0, step_min_}) {
    double nx = px + s * dx;
    double ny = py + s * dy;
    if (!checkSegmentCollision(px, py, nx, ny)) {
      return s;
    }
  }
  return 0.0;
}

// ════════════════════════════════════════════════════════════
// Check collision along a line segment
// ════════════════════════════════════════════════════════════
bool ProvedAPFPathPlanner::checkSegmentCollision(double x1, double y1,
                                                   double x2, double y2) {
  double seg_len = std::hypot(x2 - x1, y2 - y1);
  int N = std::max(static_cast<int>(std::ceil(seg_len / 0.1)), 5);

  for (int i = 1; i <= N; ++i) {
    double t = static_cast<double>(i) / N;
    double cx = x1 + t * (x2 - x1);
    double cy = y1 + t * (y2 - y1);
    if (pointInCollision(cx, cy))
      return true;
  }
  return false;
}

// ════════════════════════════════════════════════════════════
// Check if a single point is in collision
// ════════════════════════════════════════════════════════════
bool ProvedAPFPathPlanner::pointInCollision(double px, double py) {
  double mx, my;
  if (!world2Map(px, py, mx, my)) {
    return true;  // outside map is considered collision
  }
  int ix = static_cast<int>(mx);
  int iy = static_cast<int>(my);
  if (ix < 0 || ix >= nx_ || iy < 0 || iy >= ny_)
    return true;

  unsigned int idx = iy * nx_ + ix;
  // LETHAL_OBSTACLE = 253. Use threshold to be safe.
  return costmap_->getCharMap()[idx] >= costmap_2d::LETHAL_OBSTACLE * 0.8;
}

// ════════════════════════════════════════════════════════════
// 3-Stage Path Smoothing
// ════════════════════════════════════════════════════════════
void ProvedAPFPathPlanner::smoothPath(common::geometry::Points3d& path) {
  if (path.size() < 3) return;

  removeLoops(path);
  inertialSmooth(path);
  shortcutOptimize(path);
}

// ════════════════════════════════════════════════════════════
// Stage 1: Loop removal — detect and cut shortcuts
// ════════════════════════════════════════════════════════════
void ProvedAPFPathPlanner::removeLoops(common::geometry::Points3d& path) {
  size_t i = 0;
  while (i < path.size() - 2) {
    size_t j = i + 10;  // skip ahead to avoid false positives
    bool changed = false;

    while (j < path.size()) {
      double dx = path[i].x() - path[j].x();
      double dy = path[i].y() - path[j].y();
      double dist = std::hypot(dx, dy);

      if (dist < loop_threshold_) {
        // Check if straight line is collision-free
        if (!checkSegmentCollision(path[i].x(), path[i].y(),
                                    path[j].x(), path[j].y())) {
          // Cut out intermediate points
          path.erase(path.begin() + i + 1, path.begin() + j);
          changed = true;
          j = i + 10;
          continue;
        }
      }
      ++j;
    }

    if (!changed) ++i;
  }
}

// ════════════════════════════════════════════════════════════
// Stage 2: Inertial smoothing + anti-fold
// ════════════════════════════════════════════════════════════
void ProvedAPFPathPlanner::inertialSmooth(common::geometry::Points3d& path) {
  size_t n = path.size();
  if (n < 3) return;

  // Multiple passes for better results
  for (int pass = 0; pass < 3; ++pass) {
    for (size_t i = 1; i < n - 1; ++i) {
      double px_prev = path[i - 1].x(), py_prev = path[i - 1].y();
      double px_curr = path[i].x(), py_curr = path[i].y();
      double px_next = path[i + 1].x(), py_next = path[i + 1].y();

      // Inertial smoothing
      double nx = smooth_alpha_ * (px_prev + px_next) / 2.0 +
                  (1.0 - smooth_alpha_) * px_curr;
      double ny = smooth_alpha_ * (py_prev + py_next) / 2.0 +
                  (1.0 - smooth_alpha_) * py_curr;

      // Anti-fold: prevent sharp backward turns
      double v1x = px_curr - px_prev;
      double v1y = py_curr - py_prev;
      double v2x = px_next - px_curr;
      double v2y = py_next - py_curr;
      double n1 = std::hypot(v1x, v1y);
      double n2 = std::hypot(v2x, v2y);

      if (n1 > 1e-6 && n2 > 1e-6) {
        double cos_angle = (v1x * v2x + v1y * v2y) / (n1 * n2);
        if (cos_angle < 0.1) {
          // Sharp angle detected, blend with original
          nx = 0.5 * px_curr + 0.5 * (px_prev + px_next) / 2.0;
          ny = 0.5 * py_curr + 0.5 * (py_prev + py_next) / 2.0;
        }
      }

      // Collision protection
      if (!pointInCollision(nx, ny)) {
        path[i].setX(nx);
        path[i].setY(ny);
      }
    }
  }
}

// ════════════════════════════════════════════════════════════
// Stage 3: Shortcut optimization (remove detours)
// ════════════════════════════════════════════════════════════
void ProvedAPFPathPlanner::shortcutOptimize(common::geometry::Points3d& path) {
  size_t i = 0;
  while (i < path.size() - 2) {
    size_t j = i + 2;
    bool changed = false;

    while (j < path.size()) {
      double seg_len = std::hypot(path[j].x() - path[i].x(),
                                   path[j].y() - path[i].y());
      int N = std::max(static_cast<int>(std::ceil(seg_len / shortcut_resolution_)), 12);

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
        path.erase(path.begin() + i + 1, path.begin() + j);
        changed = true;
        j = i + 2;
      } else {
        ++j;
      }
    }

    if (!changed) ++i;
  }
}

}  // namespace rmp
