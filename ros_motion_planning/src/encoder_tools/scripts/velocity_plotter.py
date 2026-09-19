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

        os.makedirs(self.output_dir, exist_ok=True)
        rospy.Subscriber("/odom", Odometry, self.odom_cb, queue_size=100)
        rospy.Subscriber("/cmd_vel", Twist, self.command_cb, queue_size=100)
        rospy.on_shutdown(self.shutdown)

        rospy.loginfo("velocity_plotter: recording /cmd_vel and actual /odom velocity")
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

    def _comparison_rows(self):
        """Align zero-order-held /cmd_vel samples to each /odom timestamp."""
        events = sorted(self.command_events, key=lambda event: event[0])
        event_index = 0
        command_linear = 0.0
        command_angular = 0.0
        comparison = []
        for time_s, actual_linear, actual_angular in self.rows:
            absolute_time = self.start_time + time_s
            while event_index < len(events) and events[event_index][0] <= absolute_time:
                _, command_linear, command_angular = events[event_index]
                event_index += 1
            comparison.append((time_s, actual_linear, actual_angular,
                               command_linear, command_angular))
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
                             "command_linear_mps", "command_angular_radps"])
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

        fig, axes = plt.subplots(2, 1, sharex=True, figsize=(10, 6))
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

        fig.suptitle("Commanded vs actual velocity")
        fig.tight_layout()
        fig.savefig(self.png_path, dpi=150)
        plt.close(fig)
        rospy.loginfo("velocity_plotter: wrote plot to %s", self.png_path)


if __name__ == "__main__":
    rospy.init_node("velocity_plotter")
    VelocityPlotter()
    rospy.spin()
