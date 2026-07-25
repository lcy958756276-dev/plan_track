/**
 * *********************************************************
 *
 * @file: astar_polyline_planner.h
 * @brief: A* planner followed by strict polyline shortcutting
 *
 * ********************************************************
 */
#ifndef RMP_PATH_PLANNER_GRAPH_PLANNER_ASTAR_POLYLINE_H
#define RMP_PATH_PLANNER_GRAPH_PLANNER_ASTAR_POLYLINE_H

#include "path_planner/graph_planner/astar_prove_planner.h"
#include "path_planner/path_planner.h"

namespace rmp::path_planner {
/**
 * @brief A* based planner that reduces the global path to safe straight segments.
 */
class AStarPolylinePathPlanner : public PathPlanner {
public:
  explicit AStarPolylinePathPlanner(costmap_2d::Costmap2DROS* costmap_ros);

  bool plan(const common::geometry::Point3d& start,
            const common::geometry::Point3d& goal,
            common::geometry::Points3d* path,
            common::geometry::Points3d* expand) override;

private:
  bool _lineFreeOfInflation(const common::geometry::Point3d& a,
                            const common::geometry::Point3d& b) const;
  bool _worldPointFree(double wx, double wy) const;
  common::geometry::Points3d _polylineShortcut(
      const common::geometry::Points3d& path) const;
  void _assignSegmentHeadings(common::geometry::Points3d* path) const;

private:
  AStarProvePathPlanner base_planner_;
};
}  // namespace rmp::path_planner

#endif
