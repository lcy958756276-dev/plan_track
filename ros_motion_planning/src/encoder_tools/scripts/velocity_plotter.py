#!/usr/bin/env python3
"""
Record commanded and actual robot velocity and generate a comparison plot on shutdown.

Outputs:
  - log/velocity_actual.csv
  - log/velocity_actual.png
"""

import csv
import math
import os

import rospy
from geometry_msgs.msg import Twist
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
        self.rows = []
        self.command_events = []
        self.risk_events = []
        self.margin_events = []
        self.clearance_events = []

        os.makedirs(self.output_dir, exist_ok=True)
        rospy.Subscriber("/odom", Odometry, self.odom_cb, queue_size=100)
        rospy.Subscriber("/cmd_vel", Twist, self.command_cb, queue_size=100)
        rospy.Subscriber("/move_base/DoubleNMPCController/tracking_risk", Float64,
                         self.risk_cb, queue_size=100)
        rospy.Subscriber("/move_base/DoubleNMPCController/dynamic_safe_margin", Float64,
                         self.margin_cb, queue_size=100)
        rospy.Subscriber("/move_base/DoubleNMPCController/predicted_min_clearance", Float64,
                         self.clearance_cb, queue_size=100)
        rospy.on_shutdown(self.shutdown)

        rospy.loginfo("velocity_plotter: recording velocity and DoubleNMPC safety feedback")
        rospy.loginfo("velocity_plotter: csv=%s", self.csv_path)
        rospy.loginfo("velocity_plotter: png=%s", self.png_path)

    def odom_cb(self, msg):
        stamp = msg.header.stamp
        now = stamp.to_sec() if stamp and stamp.to_sec() > 0.0 else rospy.Time.now().to_sec()
        if self.start_time is None:
            self.start_time = now

        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        linear_speed = math.hypot(vx, vy)
        angular_speed = msg.twist.twist.angular.z
        self.rows.append((now - self.start_time, linear_speed, angular_speed))

    def command_cb(self, msg):
        now = rospy.Time.now().to_sec()
        self.command_events.append((now, msg.linear.x, msg.angular.z))

    def risk_cb(self, msg):
        self.risk_events.append((rospy.Time.now().to_sec(), msg.data))

    def margin_cb(self, msg):
        self.margin_events.append((rospy.Time.now().to_sec(), msg.data))

    def clearance_cb(self, msg):
        self.clearance_events.append((rospy.Time.now().to_sec(), msg.data))

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
        margin = self._held_scalar_samples(self.margin_events)
        clearance = self._held_scalar_samples(self.clearance_events)
        comparison = []
        for row_index, (time_s, actual_linear, actual_angular) in enumerate(self.rows):
            absolute_time = self.start_time + time_s
            while event_index < len(events) and events[event_index][0] <= absolute_time:
                _, command_linear, command_angular = events[event_index]
                event_index += 1
            comparison.append((time_s, actual_linear, actual_angular,
                               command_linear, command_angular, risk[row_index],
                               margin[row_index], clearance[row_index]))
        return comparison

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
                             "tracking_risk", "dynamic_safe_margin_m",
                             "predicted_min_clearance_m"])
            writer.writerows(comparison)
        rospy.loginfo("velocity_plotter: wrote %d samples to %s", len(self.rows), self.csv_path)

    def _write_plot(self):
        try:
            import matplotlib

            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except Exception as e:
            rospy.logwarn("velocity_plotter: matplotlib unavailable, skip png: %s", e)
            return

        comparison = self._comparison_rows()
        times = [row[0] for row in comparison]
        linear = [row[1] for row in comparison]
        angular = [row[2] for row in comparison]
        command_linear = [row[3] for row in comparison]
        command_angular = [row[4] for row in comparison]
        tracking_risk = [row[5] for row in comparison]
        tracking_margin = [row[6] for row in comparison]
        predicted_min_clearance = [row[7] for row in comparison]

        fig, axes = plt.subplots(5, 1, sharex=True, figsize=(10, 12))
        axes[0].plot(times, linear, color="#1f77b4", linewidth=1.5, label="actual /odom")
        axes[0].step(times, command_linear, where="post", color="#ff7f0e",
                     linewidth=1.2, linestyle="--", label="command /cmd_vel")
        axes[0].set_ylabel("linear (m/s)")
        axes[0].grid(True, alpha=0.3)
        axes[0].legend(loc="best")

        axes[1].plot(times, angular, color="#d62728", linewidth=1.5, label="actual /odom")
        axes[1].step(times, command_angular, where="post", color="#2ca02c",
                     linewidth=1.2, linestyle="--", label="command /cmd_vel")
        axes[1].set_xlabel("time (s)")
        axes[1].set_ylabel("angular (rad/s)")
        axes[1].grid(True, alpha=0.3)
        axes[1].legend(loc="best")

        axes[2].plot(times, tracking_risk, color="#9467bd", linewidth=1.3,
                     label="tracking risk")
        axes[2].set_ylim(-0.05, 1.05)
        axes[2].set_ylabel("risk (0-1)")
        axes[2].grid(True, alpha=0.3)
        axes[2].legend(loc="best")

        axes[3].plot(times, tracking_margin, color="#8c564b", linewidth=1.3,
                     label="dynamic tracking margin")
        axes[3].set_ylabel("margin (m)")
        axes[3].grid(True, alpha=0.3)
        axes[3].legend(loc="best")

        axes[4].plot(times, predicted_min_clearance, color="#17becf", linewidth=1.3,
                     label="predicted minimum clearance")
        axes[4].set_xlabel("time (s)")
        axes[4].set_ylabel("clearance (m)")
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
