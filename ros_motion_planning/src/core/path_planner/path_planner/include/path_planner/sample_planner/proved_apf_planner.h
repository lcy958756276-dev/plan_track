/**
 * *********************************************************
 *
 * @file: proved_apf_planner.h
 * @brief: Contains the improved APF global planner with
 *         continuous escape controller and path smoothing
 * @author: Adapted from MATLAB simulation
 * @date: 2026-06-25
 * @version: 1.0
 *
 * ********************************************************
 */
#ifndef RMP_PATH_PLANNER_SAMPLE_PLANNER_PROVED_APF_PLANNER_H
#define RMP_PATH_PLANNER_SAMPLE_PLANNER_PROVED_APF_PLANNER_H

#include "path_planner/path_planner.h"

namespace rmp::path_planner {

/**
 * @brief Improved APF global planner with escape controller and path smoothing
 *
 * Core algorithm:
 * 1. APF force calculation (attractive + repulsive)
 * 2. Escape mechanism for local minima
 * 3. Adaptive step selection
 * 4. Post-processing: loop removal + inertial smoothing + shortcut optimization
 */
class ProvedAPFPathPlanner : public PathPlanner {
public:
  ProvedAPFPathPlanner(costmap_2d::Costmap2DROS* costmap_ros);

  /**
   * @brief APF-based global planning
   * @param start  start pose (x, y, theta)
   * @param goal   goal pose (x, y, theta)
   * @param path   output path
   * @param expand output expansion nodes (not used for APF, kept for interface)
   * @return true if path found
   */
  bool plan(const common::geometry::Point3d& start,
            const common::geometry::Point3d& goal,
            common::geometry::Points3d* path,
            common::geometry::Points3d* expand) override;

private:
  // ── APF parameters ──
  double k_att_;            // attractive gain
  double k_rep_;            // repulsive gain
  double d0_;               // obstacle influence range (m)
  double step_max_;         // max step per iteration (m)
  double step_min_;         // min step per iteration (m)
  double safe_dist_;        // minimum safe distance from obstacles (m)
  int max_iter_;            // maximum iterations
  double goal_tolerance_;   // goal reach tolerance (m)

  // ── Stagnation detection ──
  int stall_window_size_;   // window for stagnation detection
  double stall_threshold_;  // stagnation distance threshold

  // ── Escape controller parameters ──
  double escape_scan_range_;// scan range for escape direction (m)
  int escape_samples_;      // number of angles to sample in escape

  // ── Smoothing parameters ──
  double smooth_alpha_;     // inertial smoothing weight
  double smooth_beta_;      // inertia weight for smoothing
  double loop_threshold_;   // loop detection threshold (m)
  double shortcut_resolution_; // sampling resolution for shortcut check (m)

  /**
   * @brief Compute APF force (attractive + repulsive)
   * @param pos current position
   * @param goal goal position
   * @return combined force vector (fx, fy)
   */
  std::pair<double, double> computeForce(double px, double py,
                                          double gx, double gy);

  /**
   * @brief Find nearest obstacle distance and direction
   * @param px current x
   * @param py current y
   * @param[out] dir_x direction away from obstacle (x)
   * @param[out] dir_y direction away from obstacle (y)
   * @return distance to nearest obstacle
   */
  double nearestObstacleInfo(double px, double py,
                             double& dir_x, double& dir_y);

  /**
   * @brief Escape controller for local minima
   */
  double escapeController(double px, double py,
                          const common::geometry::Points3d& path,
                          double gx, double gy,
                          double escape_ox, double escape_oy,
                          double fx, double fy);

  /**
   * @brief Check if a direction is obstacle-free for given range
   */
  bool isFree(double px, double py, double theta, double range);

  /**
   * @brief Select safe step size
   */
  double selectSafeStep(double px, double py, double dx, double dy);

  /**
   * @brief Check collision along a line segment
   */
  bool checkSegmentCollision(double x1, double y1, double x2, double y2);

  /**
   * @brief 3-stage path smoothing
   */
  void smoothPath(common::geometry::Points3d& path);

  /**
   * @brief Stage 1: loop removal (shortcut detection)
   */
  void removeLoops(common::geometry::Points3d& path);

  /**
   * @brief Stage 2: inertial smoothing + anti-fold
   */
  void inertialSmooth(common::geometry::Points3d& path);

  /**
   * @brief Stage 3: shortcut optimization
   */
  void shortcutOptimize(common::geometry::Points3d& path);

  /**
   * @brief Check if a single point is in collision
   */
  bool pointInCollision(double px, double py);
};

}  // namespace rmp

#endif
