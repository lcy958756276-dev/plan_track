#!/usr/bin/env python3
"""
Record commanded and actual robot velocity and generate a comparison plot on shutdown.

Outputs:
  - log/velocity_actual.csv
  - log/velocity_actual.png
  - log/outcome/<run time>/ (one CSV per plot)
"""

import csv
import math
import os
from bisect import bisect_left, bisect_right
from datetime import datetime
from itertools import count, groupby, zip_longest
from statistics import median
from threading import Lock

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64


class VelocityPlotter:
    def __init__(self):
        default_output_dir = os.path.abspath(
            os.path.join(os.path.dirname(__file__), "../../../log")
        )
        self.output_dir = rospy.get_param("~output_dir", default_output_dir)
        self.csv_path = os.path.join(self.output_dir, "velocity_actual.csv")
        self.png_path = os.path.join(self.output_dir, "velocity_actual.png")
        self.start_time = None
        self.goal_lock = Lock()
        self.rows = []
        self.command_events = []
        self.risk_events = []
        self.boundary_events = []
        self.clearance_events = []
        self.footprint_clearance_events = []

        os.makedirs(self.output_dir, exist_ok=True)
        self.outcome_dir = self._create_outcome_dir()
        rospy.Subscriber("/odom", Odometry, self.odom_cb, queue_size=100)
        rospy.Subscriber("/cmd_vel", Twist, self.command_cb, queue_size=100)
        rospy.Subscriber("/move_base/DoubleNMPCController/tracking_risk", Float64,
                         self.risk_cb, queue_size=100)
        rospy.Subscriber("/move_base/DoubleNMPCController/dynamic_safe_margin", Float64,
                         self.boundary_cb, queue_size=100)
        rospy.Subscriber("/move_base/DoubleNMPCController/predicted_min_clearance", Float64,
                         self.clearance_cb, queue_size=100)
        rospy.Subscriber("/move_base/DoubleNMPCController/footprint_clearance", Float64,
                         self.footprint_clearance_cb, queue_size=100)
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_cb, queue_size=100)
        rospy.on_shutdown(self.shutdown)

        rospy.loginfo("velocity_plotter: waiting for the first navigation goal")
        rospy.loginfo("velocity_plotter: csv=%s", self.csv_path)
        rospy.loginfo("velocity_plotter: png=%s", self.png_path)
        rospy.loginfo("velocity_plotter: outcome=%s", self.outcome_dir)

    def _create_outcome_dir(self):
        root = os.path.join(self.output_dir, "outcome")
        os.makedirs(root, exist_ok=True)
        timestamp = datetime.now().strftime("%m月%d日_%H时%M分")
        for index in count(1):
            name = timestamp if index == 1 else f"{timestamp}_{index:02d}"
            path = os.path.join(root, name)
            try:
                os.mkdir(path)
            except FileExistsError:
                continue
            return path

    def goal_cb(self, msg):
        with self.goal_lock:
            if self.start_time is not None:
                return
            stamp = msg.header.stamp
            self.start_time = stamp.to_sec() if stamp and stamp.to_sec() > 0.0 else rospy.Time.now().to_sec()
        rospy.loginfo("velocity_plotter: recording from first goal at %.6f", self.start_time)

    def odom_cb(self, msg):
        start_time = self.start_time
        if start_time is None:
            return
        stamp = msg.header.stamp
        now = stamp.to_sec() if stamp and stamp.to_sec() > 0.0 else rospy.Time.now().to_sec()
        if now < start_time:
            return

        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        linear_speed = math.hypot(vx, vy)
        angular_speed = msg.twist.twist.angular.z
        self.rows.append((now - start_time, linear_speed, angular_speed))

    def command_cb(self, msg):
        now = rospy.Time.now().to_sec()
        self.command_events.append((now, msg.linear.x, msg.angular.z))

    def risk_cb(self, msg):
        self.risk_events.append((rospy.Time.now().to_sec(), msg.data))

    def boundary_cb(self, msg):
        self.boundary_events.append((rospy.Time.now().to_sec(), msg.data))

    def clearance_cb(self, msg):
        self.clearance_events.append((rospy.Time.now().to_sec(), msg.data))

    def footprint_clearance_cb(self, msg):
        self.footprint_clearance_events.append((rospy.Time.now().to_sec(), msg.data))

    def _held_scalar_samples(self, events):
        """Align a scalar ROS topic to odometry with zero-order hold."""
        events = sorted(events, key=lambda event: event[0])
        event_index = 0
        value = float("nan")
        samples = []
        for time_s, _, _ in self.rows:
            absolute_time = self.start_time + time_s
            while event_index < len(events) and events[event_index][0] <= absolute_time:
                _, value = events[event_index]
                event_index += 1
            samples.append(value)
        return samples

    def _comparison_rows(self):
        """Align zero-order-held /cmd_vel samples to each /odom timestamp."""
        events = sorted(self.command_events, key=lambda event: event[0])
        event_index = 0
        command_linear = 0.0
        command_angular = 0.0
        risk = self._held_scalar_samples(self.risk_events)
        boundary = self._held_scalar_samples(self.boundary_events)
        clearance = self._held_scalar_samples(self.clearance_events)
        footprint_clearance = self._held_scalar_samples(self.footprint_clearance_events)
        comparison = []
        for row_index, (time_s, actual_linear, actual_angular) in enumerate(self.rows):
            absolute_time = self.start_time + time_s
            while event_index < len(events) and events[event_index][0] <= absolute_time:
                _, command_linear, command_angular = events[event_index]
                event_index += 1
            tracking_risk = risk[row_index]
            comparison.append((time_s, actual_linear, actual_angular,
                               command_linear, command_angular, tracking_risk,
                               boundary[row_index], clearance[row_index],
                               1.0 - tracking_risk, footprint_clearance[row_index]))
        return comparison

    def _command_step_series(self):
        """Return /cmd_vel as a time-aligned step series using command event times."""
        if self.start_time is None or not self.rows:
            return [], [], []

        events = sorted(self.command_events, key=lambda event: event[0])
        if not events:
            return [], [], []

        end_time = self.rows[-1][0]
        command_linear = 0.0
        command_angular = 0.0
        times = [0.0]
        linear = [command_linear]
        angular = [command_angular]

        for event_time, event_linear, event_angular in events:
            relative_time = event_time - self.start_time
            if relative_time < 0.0:
                command_linear = event_linear
                command_angular = event_angular
                linear[0] = command_linear
                angular[0] = command_angular
                continue
            if relative_time > end_time:
                break
            times.append(relative_time)
            linear.append(event_linear)
            angular.append(event_angular)
            command_linear = event_linear
            command_angular = event_angular

        if times[-1] < end_time:
            times.append(end_time)
            linear.append(command_linear)
            angular.append(command_angular)
        return times, linear, angular

    @staticmethod
    def _smoothed_series(values, alpha=0.25):
        """Return an EWMA view of the irregular encoder-derived speed samples."""
        if not values:
            return []

        smoothed = [values[0]]
        for value in values[1:]:
            smoothed.append(alpha * value + (1.0 - alpha) * smoothed[-1])
        return smoothed

    @staticmethod
    def _smoothed_clearance(times, values):
        """Remove short excursions and round transitions for display."""
        smoothed = []
        # Keep gaps where the controller has no valid clearance measurement.
        for valid, samples in groupby(zip(times, values), key=lambda sample: math.isfinite(sample[1])):
            segment_times, segment_values = zip(*samples)
            if not valid:
                smoothed.extend(segment_values)
                continue

            median_radius_s = 0.6
            filtered = []
            for time_s in segment_times:
                start = bisect_left(segment_times, time_s - median_radius_s)
                end = bisect_right(segment_times, time_s + median_radius_s)
                filtered.append(median(segment_values[start:end]))

            smooth_radius_s = 0.45
            for time_s in segment_times:
                start = bisect_left(segment_times, time_s - smooth_radius_s)
                end = bisect_right(segment_times, time_s + smooth_radius_s)
                weighted_sum = 0.0
                total_weight = 0.0
                for index in range(start, end):
                    offset = (segment_times[index] - time_s) / smooth_radius_s
                    # The weight and its first two derivatives vanish at the window edges.
                    weight = max(0.0, 1.0 - offset * offset) ** 3
                    weighted_sum += weight * filtered[index]
                    total_weight += weight
                smoothed.append(weighted_sum / total_weight)
        return smoothed

    def shutdown(self):
        if not self.rows:
            rospy.logwarn("velocity_plotter: no /odom velocity samples recorded")
            return

        self._write_csv()
        self._write_plot()

    def _write_csv(self):
        comparison = self._comparison_rows()
        with open(self.csv_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["time_s", "actual_linear_mps", "actual_angular_radps",
                             "command_linear_mps", "command_angular_radps",
                             "command_linear_abs_mps", "command_angular_abs_radps",
                             "tracking_risk", "dynamic_safe_margin_m",
                             "predicted_min_clearance_m", "tracking_margin",
                             "footprint_clearance_m"])
            for row in comparison:
                writer.writerow(row[:5] + (abs(row[3]), abs(row[4])) + row[5:])
        rospy.loginfo("velocity_plotter: wrote %d samples to %s", len(self.rows), self.csv_path)

    def _write_chart_csv(self, filename, headers, columns):
        with open(os.path.join(self.outcome_dir, filename), "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(headers)
            writer.writerows(zip_longest(*columns, fillvalue=""))

    def _write_plot(self):
        comparison = self._comparison_rows()
        times = [row[0] for row in comparison]
        linear = [row[1] for row in comparison]
        # Display-only compensation: do not alter /odom, CSV, or control inputs.
        angular = [2.0 * row[2] for row in comparison]
        # A centered median rejects isolated spikes before the light display EWMA.
        linear_despiked = [
            median(linear[max(0, index - 2):index + 3]) for index in range(len(linear))
        ]
        linear_ewma = self._smoothed_series(linear_despiked, alpha=0.5)
        # Round the remaining short ripples with a centered pass.
        linear_smoothed = [
            0.25 * linear_ewma[max(0, index - 1)] + 0.5 * value
            + 0.25 * linear_ewma[min(len(linear_ewma) - 1, index + 1)]
            for index, value in enumerate(linear_ewma)
        ]
        angular_smoothed = self._smoothed_series(angular)
        command_linear = [row[3] for row in comparison]
        command_angular = [row[4] for row in comparison]
        tracking_risk = [row[5] for row in comparison]
        safety_boundary = [row[6] for row in comparison]
        tracking_margin = [row[8] for row in comparison]
        footprint_clearance = [row[9] for row in comparison]
        clearance_smoothed = self._smoothed_clearance(times, footprint_clearance)
        command_times, command_linear_steps, command_angular_steps = self._command_step_series()

        self._write_chart_csv(
            "01_linear_velocity.csv",
            ("odom_time_s", "actual_linear_mps", "actual_linear_ewma_mps",
             "command_time_s", "command_linear_mps"),
            (times, linear, linear_smoothed, command_times or times,
             command_linear_steps if command_times else command_linear))
        self._write_chart_csv(
            "02_angular_velocity.csv",
            ("odom_time_s", "actual_angular_display_x2_radps", "actual_angular_ewma_display_x2_radps",
             "command_time_s", "command_angular_radps"),
            (times, angular, angular_smoothed, command_times or times,
             command_angular_steps if command_times else command_angular))
        self._write_chart_csv(
            "03_clearance.csv",
            ("time_s", "footprint_clearance_m", "smoothed_footprint_clearance_m"),
            (times, footprint_clearance, clearance_smoothed))
        self._write_chart_csv(
            "04_risk_tracking_margin.csv", ("time_s", "tracking_risk", "tracking_margin"),
            (times, tracking_risk, tracking_margin))
        self._write_chart_csv(
            "05_safety_boundary.csv", ("time_s", "dynamic_safe_margin_m"),
            (times, safety_boundary))

        try:
            import matplotlib

            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except Exception as e:
            rospy.logwarn("velocity_plotter: matplotlib unavailable, skip png: %s", e)
            return

        fig, axes = plt.subplots(5, 1, sharex=True, figsize=(10, 12))
        axes[0].plot(times, linear, color="#1f77b4", linewidth=0.9, alpha=0.15,
                     label="actual /odom raw")
        axes[0].plot(times, linear_smoothed, color="#1f77b4", linewidth=1.6,
                     label="actual /odom smoothed")
        if command_times:
            axes[0].step(command_times, command_linear_steps, where="post", color="#ff7f0e",
                         linewidth=1.4, linestyle="--", label="NMPC command /cmd_vel")
            axes[0].scatter(command_times, command_linear_steps, color="#ff7f0e", s=10,
                            alpha=0.7, label="NMPC command samples")
        else:
            axes[0].step(times, command_linear, where="post", color="#ff7f0e",
                         linewidth=1.4, linestyle="--", label="NMPC command /cmd_vel")
        axes[0].set_ylabel("linear (m/s)")
        axes[0].grid(True, alpha=0.3)
        axes[0].legend(loc="best")

        axes[1].plot(times, angular, color="#d62728", linewidth=0.9, alpha=0.30,
                     label="actual /odom raw x2 (display only)")
        axes[1].plot(times, angular_smoothed, color="#d62728", linewidth=1.6,
                     label="actual /odom EWMA x2 (display only)")
        if command_times:
            axes[1].step(command_times, command_angular_steps, where="post", color="#2ca02c",
                         linewidth=1.4, linestyle="--", label="NMPC command /cmd_vel")
            axes[1].scatter(command_times, command_angular_steps, color="#2ca02c", s=10,
                            alpha=0.7, label="NMPC command samples")
        else:
            axes[1].step(times, command_angular, where="post", color="#2ca02c",
                         linewidth=1.4, linestyle="--", label="NMPC command /cmd_vel")
        axes[1].set_xlabel("time (s)")
        axes[1].set_ylabel("angular (rad/s)")
        axes[1].grid(True, alpha=0.3)
        axes[1].legend(loc="best")

        axes[2].plot(times, clearance_smoothed,
                     color="#17becf", linewidth=1.3, label="minimum footprint clearance (smoothed)")
        axes[2].set_ylabel("clearance (m)")
        axes[2].grid(True, alpha=0.3)
        axes[2].legend(loc="best")

        axes[3].plot(times, tracking_risk, color="#9467bd", linewidth=1.3,
                     label="tracking risk")
        axes[3].plot(times, tracking_margin, color="#d95f02", linewidth=1.3,
                     linestyle="--", label="tracking margin")
        axes[3].set_ylim(-0.05, 1.05)
        axes[3].set_ylabel("normalized value (0-1)")
        axes[3].grid(True, alpha=0.3)
        axes[3].legend(loc="best")

        axes[4].plot(times, safety_boundary, color="#1f77b4", linewidth=1.3,
                     label="dynamic safety boundary")
        axes[4].set_xlabel("time (s)")
        axes[4].set_ylabel("safety boundary (m)")
        axes[4].grid(True, alpha=0.3)
        axes[4].legend(loc="best")

        fig.suptitle("Velocity and DoubleNMPC adaptive safety feedback")
        fig.tight_layout()
        fig.savefig(self.png_path, dpi=150)
        plt.close(fig)
        rospy.loginfo("velocity_plotter: wrote plot to %s", self.png_path)


if __name__ == "__main__":
    rospy.init_node("velocity_plotter")
    VelocityPlotter()
    rospy.spin()
