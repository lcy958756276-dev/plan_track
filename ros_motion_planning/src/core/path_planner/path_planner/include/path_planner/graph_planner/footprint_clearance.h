#ifndef RMP_PATH_PLANNER_GRAPH_PLANNER_FOOTPRINT_CLEARANCE_H
#define RMP_PATH_PLANNER_GRAPH_PLANNER_FOOTPRINT_CLEARANCE_H

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/Point.h>

#include "common/geometry/point.h"

namespace rmp::path_planner {
class FootprintClearance {
public:
  using Point3d = common::geometry::Point3d;

  struct Segment {
    double min_margin_m;
    double penalty_m = 0.0;
    double below_target_m = 0.0;
    Point3d tight_pose;
  };

  FootprintClearance(const costmap_2d::Costmap2D& map,
                     const std::vector<geometry_msgs::Point>& footprint,
                     double target_m, double weight)
    : target_m_(target_m), weight_(weight), cell_m_(map.getResolution()),
      origin_x_(map.getOriginX()), origin_y_(map.getOriginY()),
      width_(map.getSizeInCellsX()), height_(map.getSizeInCellsY()) {
    for (const auto& point : footprint) {
      half_length_ = std::max(half_length_, std::abs(point.x));
      half_width_ = std::max(half_width_, std::abs(point.y));
    }
    if (!available()) return;

    occupied_rows_.resize(height_);
    for (int y = 0; y < height_; ++y) {
      for (int x = 0; x < width_; ++x) {
        // Use occupied cells only. Inflation is already handled by the center
        // path cost, so it must not be added to the vehicle footprint again.
        if (map.getCost(x, y) >= costmap_2d::LETHAL_OBSTACLE) {
          occupied_rows_[y].push_back(x);
        }
      }
    }
  }

  bool available() const {
    return target_m_ > 0.0 && half_length_ > 0.0 && half_width_ > 0.0;
  }

  bool enabled() const { return available() && weight_ > 0.0; }

  double targetMargin() const { return target_m_; }

  double penalty(double signed_margin_m) const {
    if (!enabled()) return 0.0;
    const double deficit = std::min(2.0, std::max(0.0, 1.0 - signed_margin_m / target_m_));
    return weight_ * deficit * deficit;
  }

  // Minimum uniform padding at which the oriented rectangle touches an
  // obstacle cell. Negative values indicate overlap at the unpadded size.
  double margin(double x, double y, double yaw) const {
    if (!available()) return target_m_;
    const double c = std::cos(yaw), s = std::sin(yaw);
    const double ac = std::abs(c), as = std::abs(s), axes = ac + as;
    const double extent_x = half_length_ * ac + half_width_ * as;
    const double extent_y = half_length_ * as + half_width_ * ac;
    const double padded_x = extent_x + target_m_ * axes;
    const double padded_y = extent_y + target_m_ * axes;
    const double half_cell = 0.5 * cell_m_;
    double result = std::min({target_m_,
        (std::min(x - origin_x_, origin_x_ + width_ * cell_m_ - x) - extent_x) / axes,
        (std::min(y - origin_y_, origin_y_ + height_ * cell_m_ - y) - extent_y) / axes});

    const int min_x = std::max(0, static_cast<int>(std::floor((x - padded_x - origin_x_) / cell_m_)));
    const int max_x = std::min(width_ - 1, static_cast<int>(std::floor((x + padded_x - origin_x_) / cell_m_)));
    const int min_y = std::max(0, static_cast<int>(std::floor((y - padded_y - origin_y_) / cell_m_)));
    const int max_y = std::min(height_ - 1, static_cast<int>(std::floor((y + padded_y - origin_y_) / cell_m_)));
    for (int row = min_y; row <= max_y; ++row) {
      const double dy = origin_y_ + (row + 0.5) * cell_m_ - y;
      const auto& occupied = occupied_rows_[row];
      for (auto it = std::lower_bound(occupied.begin(), occupied.end(), min_x);
           it != occupied.end() && *it <= max_x; ++it) {
        const double dx = origin_x_ + (*it + 0.5) * cell_m_ - x;
        // The four separating axes are the two vehicle axes and the two
        // costmap axes. All four intervals must overlap to touch the cell.
        const double touch_margin = std::max({
            std::abs(dx * c + dy * s) - half_length_ - half_cell * axes,
            std::abs(-dx * s + dy * c) - half_width_ - half_cell * axes,
            (std::abs(dx) - extent_x - half_cell) / axes,
            (std::abs(dy) - extent_y - half_cell) / axes});
        result = std::min(result, touch_margin);
      }
    }
    return result;
  }

  Segment measureStraight(const Point3d& a, const Point3d& b,
                          double required_min_m = -std::numeric_limits<double>::infinity()) const {
    const double yaw = std::atan2(b.y() - a.y(), b.x() - a.x());
    return measure(a, b, yaw, yaw, required_min_m);
  }

  Segment measure(const Point3d& a, const Point3d& b, double yaw0, double yaw1,
                  double required_min_m = -std::numeric_limits<double>::infinity()) const {
    Segment result{target_m_};
    if (!available()) return result;
    const double dx = b.x() - a.x(), dy = b.y() - a.y();
    const double length = std::hypot(dx, dy);
    const double turn = std::remainder(yaw1 - yaw0, 2.0 * std::acos(-1.0));
    const double radius = std::hypot(half_length_ + target_m_, half_width_ + target_m_);
    const int samples = std::max(1, static_cast<int>(
        std::ceil((length + radius * std::abs(turn)) / (0.5 * cell_m_))));
    for (int i = 0; i <= samples; ++i) {
      const double t = static_cast<double>(i) / samples;
      const Point3d pose(a.x() + t * dx, a.y() + t * dy, yaw0 + t * turn);
      const double gap = margin(pose.x(), pose.y(), pose.theta());
      const double ds = length / samples * (i == 0 || i == samples ? 0.5 : 1.0);
      if (gap < result.min_margin_m) {
        result.min_margin_m = gap;
        result.tight_pose = pose;
      }
      if (gap < required_min_m) break;
      result.penalty_m += ds * penalty(gap);
      if (gap + 1e-6 < target_m_) result.below_target_m += ds;
    }
    return result;
  }

private:
  double target_m_, weight_, cell_m_, origin_x_, origin_y_;
  int width_, height_;
  double half_length_ = 0.0, half_width_ = 0.0;
  std::vector<std::vector<int>> occupied_rows_;
};
}  // namespace rmp::path_planner

#endif
