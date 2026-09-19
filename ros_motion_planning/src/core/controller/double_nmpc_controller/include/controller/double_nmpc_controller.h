#ifndef RMP_CONTROLLER_DOUBLE_NMPC_CONTROLLER_H_
#define RMP_CONTROLLER_DOUBLE_NMPC_CONTROLLER_H_

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_core/base_local_planner.h>
#include <tf2_ros/buffer.h>

#include "controller/controller.h"

namespace rmp::controller {

class DoubleNMPCController : public nav_core::BaseLocalPlanner, Controller {
public:
  DoubleNMPCController();
  DoubleNMPCController(std::string name, tf2_ros::Buffer* tf,
                       costmap_2d::Costmap2DROS* costmap_ros);
  ~DoubleNMPCController() override = default;

  void initialize(std::string name, tf2_ros::Buffer* tf,
                  costmap_2d::Costmap2DROS* costmap_ros) override;
  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) override;
  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel) override;
  bool isGoalReached() override;

private:
  struct Control {
    double v{0.0};
    double w{0.0};
  };

  struct TrackingPrediction {
    double position_error{0.0};
    double heading_error{0.0};
    double max_position_error{0.0};
    double max_heading_error{0.0};
    double turn_activity{0.0};
  };

  geometry_msgs::PoseStamped lookAheadPose(
      const geometry_msgs::PoseStamped& pose, double distance) const;
  Control chooseControl(const geometry_msgs::PoseStamped& pose, const Control& current,
                        const geometry_msgs::PoseStamped& reference, int horizon_steps,
                        double step_period, double speed_cap, double margin,
                        bool planner_layer) const;
  bool rolloutIsSafe(double x, double y, double yaw, const Control& control,
                     int steps, double step_period, double margin) const;
  bool poseIsSafe(double x, double y, double margin) const;
  double obstacleClearance(double x, double y) const;
  double pathCurvatureAhead(const geometry_msgs::PoseStamped& pose) const;
  TrackingPrediction evaluateTrackingPrediction(
      const geometry_msgs::PoseStamped& pose, const Control& control,
      int steps, double step_period) const;
  double predictedMinimumClearance(const geometry_msgs::PoseStamped& pose,
                                   const Control& control, int steps,
                                   double step_period) const;
  void updateTrackingMargin(const TrackingPrediction& prediction,
                            const Control& correction, double clearance);
  Control rateLimit(const Control& desired) const;
  static double normalizeAngle(double angle);
  static double clamp(double value, double low, double high);

  bool initialized_{false};
  bool goal_reached_{false};
  tf2_ros::Buffer* tf_{nullptr};
  ros::Time last_planner_update_;
  geometry_msgs::PoseStamped planner_reference_;
  Control planner_command_;

  // One-second wall-clock timing window for remote Nano profiling. These are kept
  // out of the optimization itself so timing collection does not affect control.
  ros::WallTime timing_window_start_;
  unsigned int timing_cycle_count_{0};
  unsigned int timing_planner_count_{0};
  unsigned int timing_overrun_count_{0};
  double timing_total_ms_{0.0};
  double timing_max_ms_{0.0};
  double timing_planner_total_ms_{0.0};
  double timing_planner_max_ms_{0.0};

  Control previous_command_;
  double tracking_risk_{0.0};
  double tracking_margin_{0.06};

  // Parameters. All velocity limits are local-planner parameters so the plugin can be
  // evaluated independently from the legacy APF controller.
  double control_period_{0.16};
  double command_period_{0.10};
  double planner_period_{0.48};
  double planner_update_period_{0.10};
  int tracking_horizon_steps_{3};
  int planner_horizon_steps_{6};
  double max_linear_velocity_{0.40};
  double max_angular_velocity_{1.5};
  double max_linear_acceleration_{0.25};
  double max_linear_deceleration_{0.65};
  double max_angular_acceleration_{1.2};
  double robot_radius_{0.16};
  double goal_tolerance_{0.20};
  double planner_lookahead_{0.95};
  double tracker_lookahead_{0.45};
  double max_lateral_acceleration_{0.35};
  double min_curve_speed_{0.08};
  double nominal_margin_{0.06};
  double min_margin_{0.05};
  double max_margin_{0.10};
  double obstacle_relevance_distance_{0.70};
  double risk_ewma_alpha_{0.22};
  double turn_relax_floor_{0.20};
  double margin_rise_per_cycle_{0.0015};
  double margin_fall_per_cycle_{0.0010};

  ros::Publisher risk_pub_;
  ros::Publisher margin_pub_;
  ros::Publisher clearance_pub_;
};

}  // namespace rmp::controller

#endif  // RMP_CONTROLLER_DOUBLE_NMPC_CONTROLLER_H_
