#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <utility>

#include <costmap_2d/cost_values.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "controller/double_nmpc_controller.h"

PLUGINLIB_EXPORT_CLASS(rmp::controller::DoubleNMPCController, nav_core::BaseLocalPlanner)

namespace rmp::controller {

DoubleNMPCController::DoubleNMPCController() = default;

DoubleNMPCController::DoubleNMPCController(std::string name, tf2_ros::Buffer* tf,
                                           costmap_2d::Costmap2DROS* costmap_ros)
    : DoubleNMPCController() {
  initialize(std::move(name), tf, costmap_ros);
}

void DoubleNMPCController::initialize(std::string name, tf2_ros::Buffer* tf,
                                      costmap_2d::Costmap2DROS* costmap_ros) {
  if (initialized_) {
    ROS_WARN("DoubleNMPCController has already been initialized");
    return;
  }

  tf_ = tf;
  costmap_ros_ = costmap_ros;
  ros::NodeHandle nh("~/" + name);
  nh.param("control_period", control_period_, control_period_);
  nh.param("command_period", command_period_, command_period_);
  nh.param("planner_period", planner_period_, planner_period_);
  nh.param("planner_update_period", planner_update_period_, planner_update_period_);
  nh.param("tracking_horizon_steps", tracking_horizon_steps_, tracking_horizon_steps_);
  nh.param("planner_horizon_steps", planner_horizon_steps_, planner_horizon_steps_);
  nh.param("max_linear_velocity", max_linear_velocity_, max_linear_velocity_);
  nh.param("max_angular_velocity", max_angular_velocity_, config_.max_angular_velocity());
  nh.param("max_linear_acceleration", max_linear_acceleration_, max_linear_acceleration_);
  nh.param("max_linear_deceleration", max_linear_deceleration_, max_linear_deceleration_);
  nh.param("max_angular_acceleration", max_angular_acceleration_, max_angular_acceleration_);
  nh.param("robot_radius", robot_radius_, robot_radius_);
  nh.param("goal_tolerance", goal_tolerance_, goal_tolerance_);
  nh.param("planner_lookahead", planner_lookahead_, planner_lookahead_);
  nh.param("tracker_lookahead", tracker_lookahead_, tracker_lookahead_);
  nh.param("max_lateral_acceleration", max_lateral_acceleration_,
           max_lateral_acceleration_);
  nh.param("min_curve_speed", min_curve_speed_, min_curve_speed_);
  nh.param("nominal_margin", nominal_margin_, nominal_margin_);
  nh.param("min_margin", min_margin_, min_margin_);
  nh.param("max_margin", max_margin_, max_margin_);
  nh.param("obstacle_relevance_distance", obstacle_relevance_distance_,
           obstacle_relevance_distance_);
  nh.param("risk_ewma_alpha", risk_ewma_alpha_, risk_ewma_alpha_);
  nh.param("turn_relax_floor", turn_relax_floor_, turn_relax_floor_);
  nh.param("margin_rise_per_cycle", margin_rise_per_cycle_, margin_rise_per_cycle_);
  nh.param("margin_fall_per_cycle", margin_fall_per_cycle_, margin_fall_per_cycle_);

  min_margin_ = std::max(0.0, min_margin_);
  planner_update_period_ = std::max(1e-3, planner_update_period_);
  turn_relax_floor_ = clamp(turn_relax_floor_, 0.0, 1.0);
  nominal_margin_ = clamp(nominal_margin_, min_margin_, max_margin_);
  max_margin_ = std::max(nominal_margin_, max_margin_);
  tracking_margin_ = nominal_margin_;
  risk_pub_ = nh.advertise<std_msgs::Float64>("tracking_risk", 1);
  margin_pub_ = nh.advertise<std_msgs::Float64>("dynamic_safe_margin", 1);
  clearance_pub_ = nh.advertise<std_msgs::Float64>("predicted_min_clearance", 1);
  initialized_ = true;

  ROS_INFO_STREAM("DoubleNMPCController initialized: prediction=" << control_period_
                  << "s x " << tracking_horizon_steps_ << ", command="
                  << command_period_ << "s, planner="
                  << planner_period_ << "s x " << planner_horizon_steps_
                  << " (" << planner_period_ * planner_horizon_steps_
                  << "s horizon, updated every " << planner_update_period_
                  << "s), max_angular_velocity=" << max_angular_velocity_
                  << "rad/s, margin=[" << min_margin_ << ", "
                  << max_margin_ << "] m. Set ~" << name
                  << "/max_linear_velocity explicitly before high-speed operation.");
}

bool DoubleNMPCController::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) {
  if (!initialized_ || plan.empty()) {
    ROS_WARN("DoubleNMPCController received an empty plan");
    return false;
  }
  global_plan_ = plan;
  planner_reference_ = plan.front();
  last_planner_update_ = ros::Time(0);
  goal_reached_ = false;
  tracking_risk_ = 0.0;
  tracking_margin_ = nominal_margin_;
  planner_command_ = Control{};
  previous_command_ = Control{};
  timing_window_start_ = ros::WallTime(0);
  timing_cycle_count_ = 0;
  timing_planner_count_ = 0;
  timing_overrun_count_ = 0;
  timing_total_ms_ = 0.0;
  timing_max_ms_ = 0.0;
  timing_planner_total_ms_ = 0.0;
  timing_planner_max_ms_ = 0.0;
  return true;
}

bool DoubleNMPCController::isGoalReached() {
  return initialized_ && goal_reached_;
}

bool DoubleNMPCController::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
  const ros::WallTime cycle_started = ros::WallTime::now();
  cmd_vel = geometry_msgs::Twist();
  if (!initialized_ || global_plan_.empty() || costmap_ros_ == nullptr || tf_ == nullptr) {
    ROS_ERROR_THROTTLE(1.0, "DoubleNMPCController is not ready to compute commands");
    return false;
  }

  geometry_msgs::PoseStamped robot_pose;
  if (!costmap_ros_->getRobotPose(robot_pose)) {
    ROS_WARN_THROTTLE(1.0, "DoubleNMPCController cannot obtain robot pose");
    return false;
  }
  if (robot_pose.header.frame_id != config_.map_frame()) {
    try {
      tf_->transform(robot_pose, robot_pose, config_.map_frame());
    } catch (const tf2::TransformException& exception) {
      ROS_WARN_THROTTLE(1.0, "DoubleNMPCController pose transform failed: %s",
                        exception.what());
      return false;
    }
  }

  nav_msgs::Odometry odom;
  odom_helper_->getOdom(odom);
  Control current{odom.twist.twist.linear.x, odom.twist.twist.angular.z};
  const auto& goal = global_plan_.back();
  const double goal_distance = std::hypot(robot_pose.pose.position.x - goal.pose.position.x,
                                          robot_pose.pose.position.y - goal.pose.position.y);
  // For navigation, entering the positional goal region completes the plan. The
  // final pose orientation is not a task requirement, so never command an
  // in-place rotation after the vehicle has arrived.
  if (goal_distance <= goal_tolerance_) {
    goal_reached_ = true;
    previous_command_ = Control{};
    return true;
  }

  const double path_curvature = pathCurvatureAhead(robot_pose);
  const double curve_speed_limit = std::fabs(path_curvature) < 1e-3
      ? max_linear_velocity_
      : clamp(std::sqrt(std::max(0.01, max_lateral_acceleration_) /
                         std::fabs(path_curvature)),
              min_curve_speed_, max_linear_velocity_);

  const ros::Time now = ros::Time::now();
  double planner_elapsed_ms = 0.0;
  if (last_planner_update_.isZero() ||
      (now - last_planner_update_).toSec() >= planner_update_period_) {
    planner_reference_ = lookAheadPose(robot_pose, planner_lookahead_);
    // The long layer is genuinely 6 x 0.48 = 2.88 s. It is refreshed independently
    // at planner_update_period_ and becomes a hard speed cap for the tracking layer.
    const ros::WallTime planner_started = ros::WallTime::now();
    planner_command_ = chooseControl(robot_pose, current, planner_reference_,
                                     planner_horizon_steps_, planner_period_,
                                     curve_speed_limit, tracking_margin_, true);
    planner_elapsed_ms = (ros::WallTime::now() - planner_started).toSec() * 1000.0;
    last_planner_update_ = now;
  }

  const auto tracker_reference = lookAheadPose(robot_pose, tracker_lookahead_);
  const double target_heading = std::atan2(
      tracker_reference.pose.position.y - robot_pose.pose.position.y,
      tracker_reference.pose.position.x - robot_pose.pose.position.x);
  const double heading_error = normalizeAngle(target_heading - tf2::getYaw(robot_pose.pose.orientation));
  const double tracking_speed_scale = clamp(1.0 - std::fabs(heading_error) / 1.2, 0.15, 1.0);
  const double clearance = obstacleClearance(robot_pose.pose.position.x,
                                             robot_pose.pose.position.y);

  // The tracking optimizer may refine steering, but cannot exceed the low-rate plan's
  // speed. This prevents a short-horizon tracker from re-accelerating before a bend.
  const double tracking_speed_cap = std::min(curve_speed_limit, planner_command_.v);
  const Control command = chooseControl(robot_pose, current, tracker_reference,
                                        tracking_horizon_steps_, control_period_,
                                        tracking_speed_cap, tracking_margin_, false);
  const Control bounded = rateLimit(command);
  const double moving_angular_limit = std::min(
      max_angular_velocity_,
      std::max(0.0, max_lateral_acceleration_) / std::max(bounded.v, 0.05));
  const Control correction{bounded.v - planner_command_.v,
                           bounded.w - planner_command_.w};
  const TrackingPrediction prediction = evaluateTrackingPrediction(
      robot_pose, bounded, tracking_horizon_steps_, control_period_);
  const double predicted_min_clearance = predictedMinimumClearance(
      robot_pose, bounded, tracking_horizon_steps_, control_period_);
  updateTrackingMargin(prediction, correction, clearance);

  std_msgs::Float64 clearance_message;
  clearance_message.data = predicted_min_clearance;
  clearance_pub_.publish(clearance_message);

  if (!rolloutIsSafe(robot_pose.pose.position.x, robot_pose.pose.position.y,
                     tf2::getYaw(robot_pose.pose.orientation), bounded,
                     tracking_horizon_steps_, control_period_, tracking_margin_)) {
    ROS_WARN_THROTTLE(0.5, "DoubleNMPCController safety rollout rejected command; stopping");
    previous_command_ = Control{};
    return false;
  }

  cmd_vel.linear.x = bounded.v;
  cmd_vel.angular.z = bounded.w;
  previous_command_ = bounded;
  ROS_INFO_THROTTLE(0.5,
                    "DoubleNMPC: curvature=%.3f  curve_cap=%.3f  plan=(%.3f, %.3f) "
                    "track_cap=%.3f  cmd=(%.3f, %.3f)  moving_w_cap=%.3f  heading=%.3f  speed_scale=%.2f "
                    "pred_err=(%.3f, %.3f)  turn=%.2f  risk=%.2f  margin=%.3f  "
                    "min_clearance=%.3f",
                    path_curvature, curve_speed_limit, planner_command_.v,
                    planner_command_.w, tracking_speed_cap, bounded.v, bounded.w,
                    moving_angular_limit, heading_error, tracking_speed_scale,
                    prediction.max_position_error, prediction.max_heading_error,
                    prediction.turn_activity,
                    tracking_risk_, tracking_margin_, predicted_min_clearance);

  const double cycle_elapsed_ms = (ros::WallTime::now() - cycle_started).toSec() * 1000.0;
  const double budget_ms = command_period_ * 1000.0;
  if (timing_window_start_.isZero()) {
    timing_window_start_ = cycle_started;
  }
  ++timing_cycle_count_;
  timing_total_ms_ += cycle_elapsed_ms;
  timing_max_ms_ = std::max(timing_max_ms_, cycle_elapsed_ms);
  if (planner_elapsed_ms > 0.0) {
    ++timing_planner_count_;
    timing_planner_total_ms_ += planner_elapsed_ms;
    timing_planner_max_ms_ = std::max(timing_planner_max_ms_, planner_elapsed_ms);
  }
  if (cycle_elapsed_ms > budget_ms) {
    ++timing_overrun_count_;
  }
  const double timing_window_s = (ros::WallTime::now() - timing_window_start_).toSec();
  if (timing_window_s >= 1.0) {
    const double average_ms = timing_total_ms_ / std::max(1u, timing_cycle_count_);
    const double planner_average_ms = timing_planner_count_ == 0
        ? 0.0 : timing_planner_total_ms_ / timing_planner_count_;
    ROS_INFO("DoubleNMPC timing: calls=%u window=%.2fs rate=%.1fHz "
             "cycle_ms(avg/max)=%.2f/%.2f budget=%.1f over_budget=%u "
             "planner_calls=%u planner_ms(avg/max)=%.2f/%.2f",
             timing_cycle_count_, timing_window_s, timing_cycle_count_ / timing_window_s,
             average_ms, timing_max_ms_, budget_ms, timing_overrun_count_,
             timing_planner_count_, planner_average_ms, timing_planner_max_ms_);
    timing_window_start_ = ros::WallTime::now();
    timing_cycle_count_ = 0;
    timing_planner_count_ = 0;
    timing_overrun_count_ = 0;
    timing_total_ms_ = 0.0;
    timing_max_ms_ = 0.0;
    timing_planner_total_ms_ = 0.0;
    timing_planner_max_ms_ = 0.0;
  }
  return true;
}

geometry_msgs::PoseStamped DoubleNMPCController::lookAheadPose(
    const geometry_msgs::PoseStamped& pose, double distance) const {
  geometry_msgs::PoseStamped result = global_plan_.back();
  if (global_plan_.size() == 1) {
    return result;
  }

  const double x = pose.pose.position.x;
  const double y = pose.pose.position.y;
  std::size_t nearest = 0;
  double nearest_distance = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < global_plan_.size(); ++i) {
    const double d = std::hypot(x - global_plan_[i].pose.position.x,
                                y - global_plan_[i].pose.position.y);
    if (d < nearest_distance) {
      nearest_distance = d;
      nearest = i;
    }
  }

  double traveled = 0.0;
  for (std::size_t i = nearest; i + 1 < global_plan_.size(); ++i) {
    const auto& a = global_plan_[i].pose.position;
    const auto& b = global_plan_[i + 1].pose.position;
    const double segment = std::hypot(b.x - a.x, b.y - a.y);
    if (traveled + segment >= distance && segment > 1e-6) {
      const double ratio = (distance - traveled) / segment;
      result = global_plan_[i];
      result.pose.position.x = a.x + ratio * (b.x - a.x);
      result.pose.position.y = a.y + ratio * (b.y - a.y);
      const double yaw = std::atan2(b.y - a.y, b.x - a.x);
      result.pose.orientation = tf2::toMsg(tf2::Quaternion(0.0, 0.0,
          std::sin(yaw / 2.0), std::cos(yaw / 2.0)));
      return result;
    }
    traveled += segment;
  }
  return result;
}

double DoubleNMPCController::pathCurvatureAhead(
    const geometry_msgs::PoseStamped& pose) const {
  // Three arc-length samples expose a future bend before the tracker itself reaches it.
  const double near_distance = std::max(0.15, planner_lookahead_ * 0.25);
  const double middle_distance = std::max(near_distance + 0.10, planner_lookahead_ * 0.60);
  const auto p0 = lookAheadPose(pose, near_distance).pose.position;
  const auto p1 = lookAheadPose(pose, middle_distance).pose.position;
  const auto p2 = lookAheadPose(pose, planner_lookahead_).pose.position;
  const double a = std::hypot(p1.x - p0.x, p1.y - p0.y);
  const double b = std::hypot(p2.x - p1.x, p2.y - p1.y);
  const double c = std::hypot(p2.x - p0.x, p2.y - p0.y);
  if (a < 1e-4 || b < 1e-4 || c < 1e-4) {
    return 0.0;
  }
  const double cross = (p1.x - p0.x) * (p2.y - p0.y) -
                       (p1.y - p0.y) * (p2.x - p0.x);
  return 2.0 * cross / (a * b * c);
}

DoubleNMPCController::Control DoubleNMPCController::chooseControl(
    const geometry_msgs::PoseStamped& pose, const Control& current,
    const geometry_msgs::PoseStamped& reference, int horizon_steps, double step_period,
    double speed_cap, double margin, bool planner_layer) const {
  const double x = pose.pose.position.x;
  const double y = pose.pose.position.y;
  const double yaw = tf2::getYaw(pose.pose.orientation);
  const double desired_heading = std::atan2(reference.pose.position.y - y,
                                            reference.pose.position.x - x);
  const double heading_error = normalizeAngle(desired_heading - yaw);
  const double nominal_w = clamp(heading_error / std::max(step_period, 1e-3),
                                 -max_angular_velocity_, max_angular_velocity_);
  const double heading_speed_scale = clamp(1.0 - std::fabs(heading_error) / 1.2, 0.15, 1.0);
  const double nominal_v = std::max(0.0, speed_cap) * heading_speed_scale;

  const int speed_samples = planner_layer ? 5 : 7;
  const int turn_samples = planner_layer ? 7 : 9;
  double best_cost = std::numeric_limits<double>::infinity();
  Control best{0.0, 0.0};
  for (int i = 0; i < speed_samples; ++i) {
    const double v = nominal_v * static_cast<double>(i) / (speed_samples - 1);
    // There is no separate low navigation angular-speed ceiling.  The only
    // angular-velocity bound is the platform maximum; at speed, the lateral
    // acceleration model additionally keeps a candidate dynamically feasible.
    const double candidate_w_limit = std::min(
        max_angular_velocity_,
        std::max(0.0, max_lateral_acceleration_) / std::max(v, 0.05));
    for (int j = 0; j < turn_samples; ++j) {
      const double spread = -1.0 + 2.0 * static_cast<double>(j) / (turn_samples - 1);
      const Control candidate{v, clamp(nominal_w + spread * 0.65 * candidate_w_limit,
                                       -candidate_w_limit, candidate_w_limit)};
      if (!rolloutIsSafe(x, y, yaw, candidate, horizon_steps, step_period, margin)) {
        continue;
      }
      double px = x;
      double py = y;
      double pyaw = yaw;
      for (int step = 0; step < horizon_steps; ++step) {
        px += candidate.v * std::cos(pyaw) * step_period;
        py += candidate.v * std::sin(pyaw) * step_period;
        pyaw = normalizeAngle(pyaw + candidate.w * step_period);
      }
      const double position_cost = std::hypot(px - reference.pose.position.x,
                                              py - reference.pose.position.y);
      const double heading_cost = std::fabs(normalizeAngle(
          tf2::getYaw(reference.pose.orientation) - pyaw));
      const double effort_cost = 0.12 * std::fabs(candidate.v - current.v) +
                                 0.05 * std::fabs(candidate.w - current.w);
      const double cost = 8.0 * position_cost + 1.8 * heading_cost + effort_cost;
      if (cost < best_cost) {
        best_cost = cost;
        best = candidate;
      }
    }
  }
  return best;
}

bool DoubleNMPCController::rolloutIsSafe(double x, double y, double yaw,
                                          const Control& control, int steps,
                                          double step_period, double margin) const {
  for (int i = 0; i <= steps; ++i) {
    if (!poseIsSafe(x, y, margin)) {
      return false;
    }
    x += control.v * std::cos(yaw) * step_period;
    y += control.v * std::sin(yaw) * step_period;
    yaw = normalizeAngle(yaw + control.w * step_period);
  }
  return true;
}

bool DoubleNMPCController::poseIsSafe(double x, double y, double margin) const {
  auto* costmap = costmap_ros_->getCostmap();
  if (costmap == nullptr) {
    return false;
  }
  const double radius = robot_radius_ + margin;
  const std::array<std::pair<double, double>, 9> points = {{
      {0.0, 0.0}, {radius, 0.0}, {-radius, 0.0}, {0.0, radius}, {0.0, -radius},
      {0.707 * radius, 0.707 * radius}, {0.707 * radius, -0.707 * radius},
      {-0.707 * radius, 0.707 * radius}, {-0.707 * radius, -0.707 * radius}}};
  for (const auto& point : points) {
    unsigned int mx = 0;
    unsigned int my = 0;
    if (!costmap->worldToMap(x + point.first, y + point.second, mx, my)) {
      return false;
    }
    const unsigned char cost = costmap->getCost(mx, my);
    if (cost == costmap_2d::NO_INFORMATION || cost >= costmap_2d::LETHAL_OBSTACLE) {
      return false;
    }
  }
  return true;
}

double DoubleNMPCController::obstacleClearance(double x, double y) const {
  auto* costmap = costmap_ros_->getCostmap();
  if (costmap == nullptr) {
    return 0.0;
  }
  const double resolution = std::max(costmap->getResolution(), 0.01);
  for (double radius = resolution; radius <= obstacle_relevance_distance_; radius += resolution) {
    for (int i = 0; i < 16; ++i) {
      const double angle = 2.0 * M_PI * static_cast<double>(i) / 16.0;
      unsigned int mx = 0;
      unsigned int my = 0;
      if (!costmap->worldToMap(x + radius * std::cos(angle), y + radius * std::sin(angle),
                               mx, my)) {
        return 0.0;
      }
      const unsigned char cost = costmap->getCost(mx, my);
      if (cost == costmap_2d::NO_INFORMATION || cost >= costmap_2d::LETHAL_OBSTACLE) {
        return radius;
      }
    }
  }
  return obstacle_relevance_distance_;
}

DoubleNMPCController::TrackingPrediction DoubleNMPCController::evaluateTrackingPrediction(
    const geometry_msgs::PoseStamped& pose, const Control& control, int steps,
    double step_period) const {
  TrackingPrediction prediction;
  if (global_plan_.empty()) {
    return prediction;
  }

  double x = pose.pose.position.x;
  double y = pose.pose.position.y;
  double yaw = tf2::getYaw(pose.pose.orientation);
  const double initial_yaw = yaw;
  for (int step = 0; step <= steps; ++step) {
    std::size_t nearest = 0;
    double nearest_distance = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < global_plan_.size(); ++i) {
      const auto& point = global_plan_[i].pose.position;
      const double distance = std::hypot(x - point.x, y - point.y);
      if (distance < nearest_distance) {
        nearest_distance = distance;
        nearest = i;
      }
    }

    const std::size_t next = std::min(nearest + 1, global_plan_.size() - 1);
    const std::size_t previous = nearest == 0 ? 0 : nearest - 1;
    const auto& from = global_plan_[next == nearest ? previous : nearest].pose.position;
    const auto& to = global_plan_[next].pose.position;
    const double reference_yaw = std::hypot(to.x - from.x, to.y - from.y) > 1e-6
        ? std::atan2(to.y - from.y, to.x - from.x)
        : tf2::getYaw(global_plan_[nearest].pose.orientation);
    const double heading_error = std::fabs(normalizeAngle(yaw - reference_yaw));

    if (step == 0) {
      prediction.position_error = nearest_distance;
      prediction.heading_error = heading_error;
    }
    prediction.max_position_error = std::max(prediction.max_position_error, nearest_distance);
    prediction.max_heading_error = std::max(prediction.max_heading_error, heading_error);
    prediction.turn_activity = std::max(
        prediction.turn_activity,
        clamp(std::fabs(normalizeAngle(yaw - initial_yaw)) / 0.35, 0.0, 1.0));

    x += control.v * std::cos(yaw) * step_period;
    y += control.v * std::sin(yaw) * step_period;
    yaw = normalizeAngle(yaw + control.w * step_period);
  }
  prediction.turn_activity = std::max(
      prediction.turn_activity, clamp(std::fabs(control.w) / 1.0, 0.0, 1.0));
  return prediction;
}

double DoubleNMPCController::predictedMinimumClearance(
    const geometry_msgs::PoseStamped& pose, const Control& control, int steps,
    double step_period) const {
  double x = pose.pose.position.x;
  double y = pose.pose.position.y;
  double yaw = tf2::getYaw(pose.pose.orientation);
  double minimum_clearance = obstacle_relevance_distance_;
  for (int step = 0; step <= steps; ++step) {
    minimum_clearance = std::min(minimum_clearance, obstacleClearance(x, y));
    x += control.v * std::cos(yaw) * step_period;
    y += control.v * std::sin(yaw) * step_period;
    yaw = normalizeAngle(yaw + control.w * step_period);
  }
  return minimum_clearance;
}

void DoubleNMPCController::updateTrackingMargin(const TrackingPrediction& prediction,
                                                 const Control& correction,
                                                 double clearance) {
  // This is a genuine tracking envelope: compare each predicted state against
  // the nearest path tangent instead of treating look-ahead distance as error.
  const double pos_error = std::max(prediction.position_error,
                                    prediction.max_position_error);
  const double head_error = std::max(prediction.heading_error,
                                     prediction.max_heading_error);
  const double pos_risk = clamp((pos_error - 0.015) / 0.05, 0.0, 1.0);
  const double head_risk = clamp((head_error - 0.0035) / 0.044, 0.0, 1.0);
  const double input_risk = clamp(std::max(std::fabs(correction.v) / 0.10,
                                            std::fabs(correction.w) / 0.40), 0.0, 1.0);
  const double raw_risk = 0.22 * pos_risk + 0.48 * head_risk + 0.30 * input_risk;
  tracking_risk_ = clamp((1.0 - risk_ewma_alpha_) * tracking_risk_ +
                             risk_ewma_alpha_ * raw_risk,
                         0.0, 1.0);

  // Larger clearance margins matter only around obstacles. In open space they stay at
  // nominal/relaxed values, preserving the user's requested 0.05 m absolute lower bound.
  const double relevance = clamp((obstacle_relevance_distance_ - clearance) /
                                     std::max(obstacle_relevance_distance_ - robot_radius_, 1e-3),
                                 0.0, 1.0);
  double desired_margin = nominal_margin_;
  if (tracking_risk_ < 0.10) {
    // A curve is not a failure, but it should not fully relax to the
    // straight-line minimum until the vehicle exits the turn.
    const double relax_gate = std::max(turn_relax_floor_,
                                       1.0 - prediction.turn_activity);
    desired_margin = nominal_margin_ - (nominal_margin_ - min_margin_) * relax_gate;
  } else if (tracking_risk_ > 0.20 && relevance > 0.0) {
    const double intensity = clamp((tracking_risk_ - 0.20) / 0.80, 0.0, 1.0);
    desired_margin = nominal_margin_ + (max_margin_ - nominal_margin_) * intensity * relevance;
  }
  const double delta = desired_margin - tracking_margin_;
  tracking_margin_ += clamp(delta, -margin_fall_per_cycle_, margin_rise_per_cycle_);
  tracking_margin_ = clamp(tracking_margin_, min_margin_, max_margin_);

  std_msgs::Float64 risk_message;
  risk_message.data = tracking_risk_;
  risk_pub_.publish(risk_message);
  std_msgs::Float64 margin_message;
  margin_message.data = tracking_margin_;
  margin_pub_.publish(margin_message);
}

DoubleNMPCController::Control DoubleNMPCController::rateLimit(const Control& desired) const {
  // The encoder feedback is deliberately kept for state/cost evaluation, but it is
  // sampled slower than the 20 Hz command loop and trails the actuator. Limit changes
  // from the last sent command so a stale feedback sample cannot freeze acceleration.
  const double max_dv_up = max_linear_acceleration_ * command_period_;
  const double max_dv_down = max_linear_deceleration_ * command_period_;
  const double max_dw = max_angular_acceleration_ * command_period_;
  return Control{clamp(desired.v, std::max(0.0, previous_command_.v - max_dv_down),
                       std::min(max_linear_velocity_, previous_command_.v + max_dv_up)),
                 clamp(desired.w, std::max(-max_angular_velocity_, previous_command_.w - max_dw),
                       std::min(max_angular_velocity_, previous_command_.w + max_dw))};
}

double DoubleNMPCController::normalizeAngle(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

double DoubleNMPCController::clamp(double value, double low, double high) {
  return std::max(low, std::min(value, high));
}

}  // namespace rmp::controller
