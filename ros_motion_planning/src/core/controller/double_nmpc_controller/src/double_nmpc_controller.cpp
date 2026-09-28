#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <exception>
#include <limits>
#include <mutex>
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

DoubleNMPCController::~DoubleNMPCController() {
  if (long_plan_running_) {
    long_plan_future_.wait();
  }
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
  nh.param("terminal_approach_distance", terminal_approach_distance_,
           terminal_approach_distance_);
  nh.param("terminal_max_linear_velocity", terminal_max_linear_velocity_,
           terminal_max_linear_velocity_);
  nh.param("terminal_heading_threshold", terminal_heading_threshold_,
           terminal_heading_threshold_);
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
  nh.param("turn_risk_ewma_alpha", turn_risk_ewma_alpha_, turn_risk_ewma_alpha_);
  nh.param("position_risk_deadzone", position_risk_deadzone_, position_risk_deadzone_);
  nh.param("position_risk_reference", position_risk_reference_, position_risk_reference_);
  nh.param("heading_risk_deadzone", heading_risk_deadzone_, heading_risk_deadzone_);
  nh.param("heading_risk_reference", heading_risk_reference_, heading_risk_reference_);
  nh.param("input_risk_deadzone", input_risk_deadzone_, input_risk_deadzone_);
  nh.param("equivalent_body_error_reference", equivalent_body_error_reference_,
           equivalent_body_error_reference_);
  nh.param("risk_rise_per_cycle", risk_rise_per_cycle_, risk_rise_per_cycle_);
  nh.param("risk_fall_per_cycle", risk_fall_per_cycle_, risk_fall_per_cycle_);
  nh.param("margin_relax_risk", margin_relax_risk_, margin_relax_risk_);
  nh.param("margin_tighten_risk", margin_tighten_risk_, margin_tighten_risk_);
  nh.param("pressure_floor", pressure_floor_, pressure_floor_);
  nh.param("safety_check_period", safety_check_period_, safety_check_period_);
  nh.param("turn_relax_floor", turn_relax_floor_, turn_relax_floor_);
  nh.param("margin_rise_per_cycle", margin_rise_per_cycle_, margin_rise_per_cycle_);
  nh.param("margin_fall_per_cycle", margin_fall_per_cycle_, margin_fall_per_cycle_);

  min_margin_ = std::max(0.0, min_margin_);
  planner_update_period_ = std::max(1e-3, planner_update_period_);
  terminal_approach_distance_ = std::max(goal_tolerance_, terminal_approach_distance_);
  terminal_max_linear_velocity_ = clamp(terminal_max_linear_velocity_, 0.01,
                                        max_linear_velocity_);
  terminal_heading_threshold_ = clamp(terminal_heading_threshold_, 0.01, M_PI);
  risk_ewma_alpha_ = clamp(risk_ewma_alpha_, 0.0, 1.0);
  turn_risk_ewma_alpha_ = clamp(turn_risk_ewma_alpha_, 0.0, 1.0);
  position_risk_reference_ = std::max(1e-4, position_risk_reference_);
  heading_risk_reference_ = std::max(1e-4, heading_risk_reference_);
  equivalent_body_error_reference_ = std::max(1e-4, equivalent_body_error_reference_);
  margin_relax_risk_ = clamp(margin_relax_risk_, 0.0, 1.0);
  margin_tighten_risk_ = clamp(margin_tighten_risk_, margin_relax_risk_, 1.0);
  pressure_floor_ = clamp(pressure_floor_, 0.0, 1.0);
  safety_check_period_ = std::max(0.01, safety_check_period_);
  turn_relax_floor_ = clamp(turn_relax_floor_, 0.0, 1.0);
  nominal_margin_ = clamp(nominal_margin_, min_margin_, max_margin_);
  max_margin_ = std::max(nominal_margin_, max_margin_);
  tracking_margin_ = nominal_margin_;
  planner_margin_snapshot_ = nominal_margin_;
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
  // The worker reads the current global plan while building a snapshot. A new
  // move_base plan is rare, so wait here rather than racing the vector update.
  if (long_plan_running_) {
    ROS_INFO("DoubleNMPC long plan: waiting for in-flight worker before replacing global plan");
    long_plan_future_.wait();
    long_plan_running_ = false;
  }
  global_plan_ = plan;
  global_plan_length_ = 0.0;
  for (std::size_t i = 1; i < global_plan_.size(); ++i) {
    const auto& previous = global_plan_[i - 1].pose.position;
    const auto& current = global_plan_[i].pose.position;
    global_plan_length_ += std::hypot(current.x - previous.x, current.y - previous.y);
  }
  path_progress_ = 0.0;
  planner_reference_ = plan.front();
  last_planner_update_ = ros::Time(0);
  next_planner_activation_ = ros::Time(0);
  ++plan_version_;
  has_pending_long_plan_ = false;
  has_active_long_plan_ = false;
  pending_long_plan_ = LongPlan{};
  active_long_plan_ = LongPlan{};
  goal_reached_ = false;
  tracking_risk_ = 0.0;
  tracking_margin_ = nominal_margin_;
  planner_margin_snapshot_ = nominal_margin_;
  position_error_ewma_ = 0.0;
  heading_error_ewma_ = 0.0;
  input_correction_ewma_ = 0.0;
  turn_activity_ewma_ = 0.0;
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
  ROS_INFO("DoubleNMPC long plan: reset for global plan version=%llu, points=%zu, length=%.3fm",
           static_cast<unsigned long long>(plan_version_), global_plan_.size(), global_plan_length_);
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

  // A nearest vertex is not a valid path coordinate: it jumps between samples and
  // can move backward after a small odometry wobble. Project onto segments and only
  // allow a tiny backward window for a newly replanned path.
  const PathProjection path_projection = projectOntoPath(
      robot_pose, std::max(0.0, path_progress_ - 0.05));
  path_progress_ = std::max(path_progress_, path_projection.arc_length);
  const double recovery_scale = clamp(1.0 - path_projection.lateral_error / 0.50,
                                      0.35, 1.0);
  const double planner_lookahead = std::max(0.25, planner_lookahead_ * recovery_scale);
  const double tracker_lookahead = std::max(0.12, tracker_lookahead_ * recovery_scale);
  const bool terminal_approach = goal_distance <= terminal_approach_distance_;

  const double path_curvature = pathCurvatureAhead(robot_pose);
  const double curve_speed_limit = std::fabs(path_curvature) < 1e-3
      ? max_linear_velocity_
      : clamp(std::sqrt(std::max(0.01, max_lateral_acceleration_) /
                         std::fabs(path_curvature)),
              min_curve_speed_, max_linear_velocity_);

  const ros::Time now = ros::Time::now();
  collectLongPlanResult(now);
  if (!terminal_approach && !long_plan_running_ &&
      (last_planner_update_.isZero() ||
       (now - last_planner_update_).toSec() >= planner_update_period_)) {
    // The feedback margin is frozen into the request. The worker produces an
    // immutable 6-stage plan, so later 20 Hz feedback cannot mutate a plan in use.
    planner_margin_snapshot_ = tracking_margin_;
    LongPlanRequest request;
    request.id = next_long_plan_id_++;
    request.plan_version = plan_version_;
    request.launched_at = now;
    request.start_pose = robot_pose;
    request.start_control = current;
    request.start_arc_length = path_progress_;
    request.curve_speed_limit = curve_speed_limit;
    request.margin = planner_margin_snapshot_;
    launchLongPlan(request);
    last_planner_update_ = now;
  }
  commitPendingLongPlan(now);

  Control cached_plan_command;
  geometry_msgs::PoseStamped tracker_reference;
  unsigned int active_plan_stage = 0;
  double active_plan_age = 0.0;
  const bool active_plan_available = sampleActiveLongPlan(
      now, cached_plan_command, tracker_reference, active_plan_stage, active_plan_age);
  if (active_plan_available) {
    planner_command_ = cached_plan_command;
    planner_reference_ = tracker_reference;
  } else {
    planner_command_ = Control{};
    tracker_reference = poseAtPathArcLength(path_progress_ + tracker_lookahead);
    if (!terminal_approach) {
      const bool expired = has_active_long_plan_;
      ROS_WARN_THROTTLE(0.5,
                        "DoubleNMPC long plan stale: active=%d worker=%d pending=%d; holding zero command",
                        expired, long_plan_running_, has_pending_long_plan_);
    }
  }
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
  const bool planner_requests_in_place_turn =
      planner_command_.v <= 1e-4 && std::fabs(planner_command_.w) > 1e-3;
  // If the long layer has selected a safe in-place turn, preserve that decision.
  // Letting the short layer re-optimize with a zero linear-speed cap produced tiny,
  // alternating angular commands that the wheel deadband turned into endpoint jitter.
  Control command;
  if (terminal_approach) {
    const double terminal_heading = std::atan2(
        goal.pose.position.y - robot_pose.pose.position.y,
        goal.pose.position.x - robot_pose.pose.position.x);
    const double terminal_heading_error = normalizeAngle(
        terminal_heading - tf2::getYaw(robot_pose.pose.orientation));
    const double terminal_w = clamp(1.8 * terminal_heading_error, -0.60, 0.60);
    if (std::fabs(terminal_heading_error) > terminal_heading_threshold_) {
      command = Control{0.0, terminal_w};
    } else {
      const double remaining = std::max(0.0, goal_distance - goal_tolerance_);
      command = Control{std::min(terminal_max_linear_velocity_,
                                  std::max(0.035, 0.65 * remaining)), terminal_w};
    }
    planner_command_ = command;
  } else {
    command = planner_requests_in_place_turn
        ? Control{0.0, planner_command_.w}
        : chooseControl(robot_pose, current, tracker_reference,
                        tracking_horizon_steps_, control_period_,
                        tracking_speed_cap, tracking_margin_, false, path_progress_);
  }
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
                    "DoubleNMPC: terminal=%d goal_dist=%.3f path_s=%.3f/%.3f cte=%.3f lookahead=(%.2f,%.2f) "
                    "curvature=%.3f  curve_cap=%.3f  plan{id=%llu stage=%u age=%.2f ready=%d} margin=%.3f plan=(%.3f, %.3f) "
                    "track_cap=%.3f  cmd=(%.3f, %.3f)  moving_w_cap=%.3f  heading=%.3f  speed_scale=%.2f "
                    "pred_err=(%.3f, %.3f)  turn=%.2f  risk=%.2f  margin=%.3f  "
                    "min_clearance=%.3f",
                    terminal_approach, goal_distance, path_progress_, global_plan_length_, path_projection.lateral_error,
                    planner_lookahead, tracker_lookahead, path_curvature, curve_speed_limit,
                    static_cast<unsigned long long>(has_active_long_plan_ ? active_long_plan_.id : 0),
                    active_plan_stage, active_plan_age, active_plan_available,
                    planner_margin_snapshot_, planner_command_.v,
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
  const PathProjection projection = projectOntoPath(
      pose, std::max(0.0, path_progress_ - 0.05));
  return poseAtPathArcLength(std::max(path_progress_, projection.arc_length) + distance);
}

DoubleNMPCController::LongPlan DoubleNMPCController::buildLongPlan(
    const LongPlanRequest& request) const {
  const ros::WallTime solve_started = ros::WallTime::now();
  LongPlan plan;
  plan.id = request.id;
  plan.plan_version = request.plan_version;
  plan.launched_at = request.launched_at;
  plan.start_arc_length = request.start_arc_length;
  plan.margin = request.margin;

  geometry_msgs::PoseStamped predicted_pose = request.start_pose;
  Control predicted_control = request.start_control;
  plan.states.push_back(predicted_pose);
  double predicted_arc = request.start_arc_length;
  for (int stage = 0; stage < planner_horizon_steps_; ++stage) {
    const double reference_distance = planner_lookahead_ +
        static_cast<double>(stage) * std::max(0.12, request.curve_speed_limit * planner_period_);
    const geometry_msgs::PoseStamped reference = poseAtPathArcLength(
        std::min(global_plan_length_, request.start_arc_length + reference_distance));
    const Control control = chooseControl(
        predicted_pose, predicted_control, reference,
        std::max(1, planner_horizon_steps_ - stage), planner_period_,
        request.curve_speed_limit, request.margin, true, predicted_arc);
    plan.controls.push_back(control);

    const double yaw = tf2::getYaw(predicted_pose.pose.orientation);
    predicted_pose.pose.position.x += control.v * std::cos(yaw) * planner_period_;
    predicted_pose.pose.position.y += control.v * std::sin(yaw) * planner_period_;
    const double next_yaw = normalizeAngle(yaw + control.w * planner_period_);
    predicted_pose.pose.orientation = tf2::toMsg(tf2::Quaternion(
        0.0, 0.0, std::sin(next_yaw / 2.0), std::cos(next_yaw / 2.0)));
    predicted_arc = std::min(global_plan_length_, predicted_arc + control.v * planner_period_);
    predicted_control = control;
    plan.states.push_back(predicted_pose);
  }
  plan.solve_time_ms = (ros::WallTime::now() - solve_started).toSec() * 1000.0;
  return plan;
}

void DoubleNMPCController::launchLongPlan(const LongPlanRequest& request) {
  long_plan_running_ = true;
  ROS_DEBUG("DoubleNMPC long plan launch: id=%llu version=%llu s=%.3f margin=%.3f cap=%.3f",
            static_cast<unsigned long long>(request.id),
            static_cast<unsigned long long>(request.plan_version), request.start_arc_length,
            request.margin, request.curve_speed_limit);
  long_plan_future_ = std::async(std::launch::async,
      [this, request]() { return buildLongPlan(request); });
}

void DoubleNMPCController::collectLongPlanResult(const ros::Time& now) {
  if (!long_plan_running_ ||
      long_plan_future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
    return;
  }
  long_plan_running_ = false;
  LongPlan completed;
  try {
    completed = long_plan_future_.get();
  } catch (const std::exception& exception) {
    ROS_ERROR("DoubleNMPC long plan worker failed: %s", exception.what());
    return;
  }
  completed.ready_at = now;
  ++timing_planner_count_;
  timing_planner_total_ms_ += completed.solve_time_ms;
  timing_planner_max_ms_ = std::max(timing_planner_max_ms_, completed.solve_time_ms);
  if (completed.plan_version != plan_version_ || completed.controls.empty()) {
    ROS_WARN("DoubleNMPC long plan discard: id=%llu version=%llu current_version=%llu stages=%zu",
             static_cast<unsigned long long>(completed.id),
             static_cast<unsigned long long>(completed.plan_version),
             static_cast<unsigned long long>(plan_version_), completed.controls.size());
    return;
  }
  pending_long_plan_ = std::move(completed);
  has_pending_long_plan_ = true;
  ROS_DEBUG("DoubleNMPC long plan ready: id=%llu stages=%zu solve=%.2fms; waiting for commit boundary",
            static_cast<unsigned long long>(pending_long_plan_.id), pending_long_plan_.controls.size(),
            pending_long_plan_.solve_time_ms);
}

void DoubleNMPCController::commitPendingLongPlan(const ros::Time& now) {
  if (!has_pending_long_plan_) {
    return;
  }
  if (has_active_long_plan_ && now < next_planner_activation_) {
    return;
  }
  pending_long_plan_.activated_at = now;
  active_long_plan_ = std::move(pending_long_plan_);
  has_pending_long_plan_ = false;
  has_active_long_plan_ = true;
  next_planner_activation_ = now + ros::Duration(planner_period_);
  ROS_INFO("DoubleNMPC long plan commit: id=%llu age=%.3fs solve=%.2fms s=%.3f stages=%zu margin=%.3f "
           "u0=(%.3f,%.3f) next_boundary=%.3f",
           static_cast<unsigned long long>(active_long_plan_.id),
           (now - active_long_plan_.launched_at).toSec(), active_long_plan_.solve_time_ms,
           active_long_plan_.start_arc_length,
           active_long_plan_.controls.size(), active_long_plan_.margin,
           active_long_plan_.controls.front().v, active_long_plan_.controls.front().w,
           next_planner_activation_.toSec());
}

bool DoubleNMPCController::sampleActiveLongPlan(const ros::Time& now, Control& control,
                                                 geometry_msgs::PoseStamped& reference,
                                                 unsigned int& stage, double& age) const {
  if (!has_active_long_plan_ || active_long_plan_.controls.empty() ||
      active_long_plan_.states.empty()) {
    return false;
  }
  age = std::max(0.0, (now - active_long_plan_.activated_at).toSec());
  stage = static_cast<unsigned int>(age / planner_period_);
  if (stage >= active_long_plan_.controls.size()) {
    return false;
  }
  control = active_long_plan_.controls[stage];
  reference = active_long_plan_.states[stage];
  const double stage_elapsed = age - static_cast<double>(stage) * planner_period_;
  const double yaw = tf2::getYaw(reference.pose.orientation);
  reference.pose.position.x += control.v * std::cos(yaw) * stage_elapsed;
  reference.pose.position.y += control.v * std::sin(yaw) * stage_elapsed;
  const double reference_yaw = normalizeAngle(yaw + control.w * stage_elapsed);
  reference.pose.orientation = tf2::toMsg(tf2::Quaternion(
      0.0, 0.0, std::sin(reference_yaw / 2.0), std::cos(reference_yaw / 2.0)));
  return true;
}

DoubleNMPCController::PathProjection DoubleNMPCController::projectOntoPath(
    const geometry_msgs::PoseStamped& pose, double minimum_arc_length) const {
  PathProjection result;
  if (global_plan_.empty()) {
    return result;
  }
  result.pose = global_plan_.front();
  if (global_plan_.size() == 1) {
    result.lateral_error = std::hypot(pose.pose.position.x - result.pose.pose.position.x,
                                      pose.pose.position.y - result.pose.pose.position.y);
    return result;
  }

  double accumulated = 0.0;
  double best_distance = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i + 1 < global_plan_.size(); ++i) {
    const auto& a = global_plan_[i].pose.position;
    const auto& b = global_plan_[i + 1].pose.position;
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double length_squared = dx * dx + dy * dy;
    const double length = std::sqrt(length_squared);
    if (length < 1e-6) {
      continue;
    }
    const double segment_end = accumulated + length;
    if (segment_end + 1e-6 >= minimum_arc_length) {
      const double raw_ratio = ((pose.pose.position.x - a.x) * dx +
                                (pose.pose.position.y - a.y) * dy) / length_squared;
      const double min_ratio = clamp((minimum_arc_length - accumulated) / length, 0.0, 1.0);
      const double ratio = clamp(raw_ratio, min_ratio, 1.0);
      const double px = a.x + ratio * dx;
      const double py = a.y + ratio * dy;
      const double error = std::hypot(pose.pose.position.x - px, pose.pose.position.y - py);
      if (error < best_distance) {
        best_distance = error;
        result.pose = global_plan_[i];
        result.pose.pose.position.x = px;
        result.pose.pose.position.y = py;
        const double yaw = std::atan2(dy, dx);
        result.pose.pose.orientation = tf2::toMsg(tf2::Quaternion(
            0.0, 0.0, std::sin(yaw / 2.0), std::cos(yaw / 2.0)));
        result.arc_length = accumulated + ratio * length;
        result.lateral_error = error;
      }
    }
    accumulated = segment_end;
  }
  if (!std::isfinite(best_distance)) {
    result.pose = global_plan_.back();
    result.arc_length = global_plan_length_;
    result.lateral_error = std::hypot(pose.pose.position.x - result.pose.pose.position.x,
                                      pose.pose.position.y - result.pose.pose.position.y);
  }
  return result;
}

geometry_msgs::PoseStamped DoubleNMPCController::poseAtPathArcLength(double arc_length) const {
  geometry_msgs::PoseStamped result = global_plan_.back();
  if (global_plan_.size() < 2) {
    return result;
  }
  arc_length = clamp(arc_length, 0.0, global_plan_length_);
  double accumulated = 0.0;
  for (std::size_t i = 0; i + 1 < global_plan_.size(); ++i) {
    const auto& a = global_plan_[i].pose.position;
    const auto& b = global_plan_[i + 1].pose.position;
    const double length = std::hypot(b.x - a.x, b.y - a.y);
    if (length < 1e-6) {
      continue;
    }
    if (accumulated + length >= arc_length) {
      const double ratio = (arc_length - accumulated) / length;
      result = global_plan_[i];
      result.pose.position.x = a.x + ratio * (b.x - a.x);
      result.pose.position.y = a.y + ratio * (b.y - a.y);
      const double yaw = std::atan2(b.y - a.y, b.x - a.x);
      result.pose.orientation = tf2::toMsg(tf2::Quaternion(
          0.0, 0.0, std::sin(yaw / 2.0), std::cos(yaw / 2.0)));
      return result;
    }
    accumulated += length;
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
    double speed_cap, double margin, bool planner_layer, double path_progress) const {
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
      double accumulated_cross_track_error = 0.0;
      for (int step = 0; step < horizon_steps; ++step) {
        px += candidate.v * std::cos(pyaw) * step_period;
        py += candidate.v * std::sin(pyaw) * step_period;
        pyaw = normalizeAngle(pyaw + candidate.w * step_period);
        geometry_msgs::PoseStamped predicted_pose = pose;
        predicted_pose.pose.position.x = px;
        predicted_pose.pose.position.y = py;
        predicted_pose.pose.orientation = tf2::toMsg(tf2::Quaternion(
            0.0, 0.0, std::sin(pyaw / 2.0), std::cos(pyaw / 2.0)));
        accumulated_cross_track_error += projectOntoPath(
            predicted_pose, std::max(0.0, path_progress - 0.05)).lateral_error;
      }
      const double position_cost = std::hypot(px - reference.pose.position.x,
                                              py - reference.pose.position.y);
      const double heading_cost = std::fabs(normalizeAngle(
          tf2::getYaw(reference.pose.orientation) - pyaw));
      const double cross_track_cost = accumulated_cross_track_error /
                                      std::max(1, horizon_steps);
      const double effort_cost = 0.12 * std::fabs(candidate.v - current.v) +
                                 0.05 * std::fabs(candidate.w - current.w);
      // Endpoint-only tracking lets a vehicle carry lateral error until the last
      // segment. Penalizing the full predicted trajectory makes returning to the
      // global route beneficial as soon as odometry sees a deviation.
      const double cost = 8.0 * position_cost + 4.0 * cross_track_cost +
                          1.8 * heading_cost + effort_cost;
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
  const int substeps = std::max(1, static_cast<int>(std::ceil(
      step_period / safety_check_period_)));
  const double subperiod = step_period / static_cast<double>(substeps);
  for (int step = 0; step < steps; ++step) {
    for (int substep = 0; substep < substeps; ++substep) {
      if (!poseIsSafe(x, y, margin)) {
        return false;
      }
      x += control.v * std::cos(yaw) * subperiod;
      y += control.v * std::sin(yaw) * subperiod;
      yaw = normalizeAngle(yaw + control.w * subperiod);
    }
  }
  return poseIsSafe(x, y, margin);
}

bool DoubleNMPCController::poseIsSafe(double x, double y, double margin) const {
  auto* costmap = costmap_ros_->getCostmap();
  if (costmap == nullptr) {
    return false;
  }
  std::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*costmap->getMutex());
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
  std::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*costmap->getMutex());
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
  const double position_error = std::max(prediction.position_error,
                                         prediction.max_position_error);
  const double heading_error = std::max(prediction.heading_error,
                                        prediction.max_heading_error);
  const double input_correction = clamp(
      std::max(std::fabs(correction.v) / 0.10,
               std::fabs(correction.w) / 0.40), 0.0, 2.0);

  // Keep feedback components separate until the final combination. This exposes
  // whether a tightening was caused by lateral tracking, heading, control effort,
  // or an active turn, rather than hiding everything in one filtered scalar.
  position_error_ewma_ = (1.0 - risk_ewma_alpha_) * position_error_ewma_ +
                         risk_ewma_alpha_ * position_error;
  heading_error_ewma_ = (1.0 - risk_ewma_alpha_) * heading_error_ewma_ +
                        risk_ewma_alpha_ * heading_error;
  input_correction_ewma_ = (1.0 - risk_ewma_alpha_) * input_correction_ewma_ +
                           risk_ewma_alpha_ * input_correction;
  turn_activity_ewma_ = (1.0 - turn_risk_ewma_alpha_) * turn_activity_ewma_ +
                        turn_risk_ewma_alpha_ * prediction.turn_activity;

  const double position_risk = clamp(
      (position_error_ewma_ - position_risk_deadzone_) / position_risk_reference_,
      0.0, 1.0);
  const double heading_risk = clamp(
      (heading_error_ewma_ - heading_risk_deadzone_) / heading_risk_reference_,
      0.0, 1.0);
  const double input_risk = clamp(
      (input_correction_ewma_ - input_risk_deadzone_) /
          std::max(1e-4, 1.0 - input_risk_deadzone_),
      0.0, 1.0);
  const double turn_risk = clamp(turn_activity_ewma_, 0.0, 1.0);

  const double raw_risk = clamp(0.16 * position_risk + 0.34 * heading_risk +
                                    0.24 * input_risk +
                                    0.30 * turn_risk *
                                        (0.55 * heading_risk + 0.45 * input_risk),
                                0.0, 1.0);
  tracking_risk_ = clamp(tracking_risk_ + clamp(raw_risk - tracking_risk_,
                                                  -risk_fall_per_cycle_,
                                                  risk_rise_per_cycle_),
                         0.0, 1.0);

  // This is a circular-body envelope, not the MATLAB rectangular certificate.
  // It gives the margin controller a conservative heading-dependent displacement.
  const double equivalent_body_error = position_error_ewma_ +
      robot_radius_ * std::sin(std::min(0.5 * M_PI, heading_error_ewma_));
  const double body_pressure = clamp(
      equivalent_body_error / equivalent_body_error_reference_, 0.0, 1.0);
  const double joint_pressure = std::max({turn_risk, input_risk, body_pressure});

  // Larger clearance margins matter only around obstacles. In open space they stay at
  // nominal/relaxed values, preserving the user's requested 0.05 m absolute lower bound.
  const double relevance = clamp((obstacle_relevance_distance_ - clearance) /
                                     std::max(obstacle_relevance_distance_ - robot_radius_, 1e-3),
                                 0.0, 1.0);
  double desired_margin = nominal_margin_;
  if (tracking_risk_ < margin_relax_risk_) {
    // A curve is not a failure, but it should not fully relax to the
    // straight-line minimum until the vehicle exits the turn.
    const double relax_gate = std::max(turn_relax_floor_,
                                       1.0 - turn_risk);
    desired_margin = nominal_margin_ - (nominal_margin_ - min_margin_) * relax_gate;
  } else if (tracking_risk_ > margin_tighten_risk_ && relevance > 0.0) {
    const double intensity = clamp(
        (tracking_risk_ - margin_tighten_risk_) /
            std::max(1e-4, 1.0 - margin_tighten_risk_),
        0.0, 1.0);
    const double pressure = pressure_floor_ + (1.0 - pressure_floor_) * joint_pressure;
    desired_margin = nominal_margin_ +
        (max_margin_ - nominal_margin_) * intensity * pressure * relevance;
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
