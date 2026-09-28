#!/usr/bin/env python3
"""
Record actual robot velocity from /odom and generate a curve plot on shutdown.

Outputs:
  - log/velocity_actual.csv
  - log/velocity_actual.png
"""

import csv
import math
import os

import rospy
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

        os.makedirs(self.output_dir, exist_ok=True)
        rospy.Subscriber("/odom", Odometry, self.odom_cb, queue_size=100)
        rospy.on_shutdown(self.shutdown)

        rospy.loginfo("velocity_plotter: recording actual /odom velocity")
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

    def shutdown(self):
        if not self.rows:
            rospy.logwarn("velocity_plotter: no /odom velocity samples recorded")
            return

        self._write_csv()
        self._write_plot()

    def _write_csv(self):
        with open(self.csv_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["time_s", "actual_linear_mps", "actual_angular_radps"])
            writer.writerows(self.rows)
        rospy.loginfo("velocity_plotter: wrote %d samples to %s", len(self.rows), self.csv_path)

    def _write_plot(self):
        try:
            import matplotlib

            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except Exception as e:
            rospy.logwarn("velocity_plotter: matplotlib unavailable, skip png: %s", e)
            return

        times = [row[0] for row in self.rows]
        linear = [row[1] for row in self.rows]
        angular = [row[2] for row in self.rows]

        fig, axes = plt.subplots(2, 1, sharex=True, figsize=(10, 6))
        axes[0].plot(times, linear, color="#1f77b4", linewidth=1.5)
        axes[0].set_ylabel("linear (m/s)")
        axes[0].grid(True, alpha=0.3)

        axes[1].plot(times, angular, color="#d62728", linewidth=1.5)
        axes[1].set_xlabel("time (s)")
        axes[1].set_ylabel("angular (rad/s)")
        axes[1].grid(True, alpha=0.3)

        fig.suptitle("Actual velocity from /odom")
        fig.tight_layout()
        fig.savefig(self.png_path, dpi=150)
        plt.close(fig)
        rospy.loginfo("velocity_plotter: wrote plot to %s", self.png_path)


if __name__ == "__main__":
    rospy.init_node("velocity_plotter")
    VelocityPlotter()
    rospy.spin()
