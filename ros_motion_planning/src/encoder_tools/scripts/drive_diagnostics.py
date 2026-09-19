#!/usr/bin/env python3
"""Passive drive-response recorder for real-vehicle tuning.

This node never publishes a control topic and never changes a command.  It only
records the command, encoder/odometry feedback, and detected stop responses so
the lower-level PID and braking behavior can be identified off-line.
"""

import csv
import json
import math
import os
import threading

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray, Int64MultiArray


class DriveDiagnostics:
    def __init__(self):
        default_output_dir = os.path.abspath(
            os.path.join(os.path.dirname(__file__), "../../../log"))
        self.output_dir = rospy.get_param("~output_dir", default_output_dir)
        self.stop_command_threshold = rospy.get_param("~stop_command_threshold", 0.02)
        self.stop_speed_threshold = rospy.get_param("~stop_speed_threshold", 0.01)

        self.lock = threading.Lock()
        self.start_time = None
        self.command = {"v": 0.0, "w": 0.0, "stamp": None}
        self.wheel_velocity = {"left": 0.0, "right": 0.0, "stamp": None}
        self.ticks = {"left": None, "right": None, "stamp": None}
        self.latest_pose = None
        self.response_rows = []
        self.command_rows = []
        self.tick_rows = []
        self.stop_rows = []
        self.pending_stop = None

        rospy.Subscriber("/cmd_vel", Twist, self.command_cb, queue_size=100)
        rospy.Subscriber("/odom", Odometry, self.odom_cb, queue_size=100)
        rospy.Subscriber("/wheel_velocities", Float64MultiArray, self.wheel_velocity_cb,
                         queue_size=100)
        rospy.Subscriber("/wheel_ticks", Int64MultiArray, self.tick_cb, queue_size=100)
        rospy.on_shutdown(self.shutdown)

        rospy.loginfo("drive_diagnostics: passive recording enabled; no control output")
        rospy.loginfo("drive_diagnostics: output_dir=%s", self.output_dir)

    def _relative_time(self, stamp):
        if self.start_time is None:
            self.start_time = stamp
        return (stamp - self.start_time).to_sec()

    def command_cb(self, msg):
        stamp = rospy.Time.now()
        with self.lock:
            previous_v = self.command["v"]
            self.command = {"v": msg.linear.x, "w": msg.angular.z, "stamp": stamp}
            self.command_rows.append({
                "time_s": self._relative_time(stamp),
                "linear_mps": msg.linear.x,
                "angular_radps": msg.angular.z,
            })

            if (abs(previous_v) >= self.stop_command_threshold and
                    abs(msg.linear.x) < 1e-4 and self.latest_pose is not None):
                self.pending_stop = {
                    "command_time": stamp,
                    "initial_speed": self.latest_pose["linear_mps"],
                    "start_x": self.latest_pose["x"],
                    "start_y": self.latest_pose["y"],
                }
                rospy.loginfo("drive_diagnostics: stop command detected at %.3f m/s",
                              self.pending_stop["initial_speed"])

    def wheel_velocity_cb(self, msg):
        if len(msg.data) < 2:
            return
        stamp = rospy.Time.now()
        with self.lock:
            self.wheel_velocity = {"left": msg.data[0], "right": msg.data[1],
                                   "stamp": stamp}

    def tick_cb(self, msg):
        if len(msg.data) < 2:
            return
        stamp = rospy.Time.now()
        with self.lock:
            self.ticks = {"left": msg.data[0], "right": msg.data[1], "stamp": stamp}
            self.tick_rows.append({
                "time_s": self._relative_time(stamp),
                "left_tick": msg.data[0],
                "right_tick": msg.data[1],
            })

    def odom_cb(self, msg):
        stamp = msg.header.stamp if msg.header.stamp.to_sec() > 0.0 else rospy.Time.now()
        linear_speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        with self.lock:
            self.latest_pose = {
                "x": msg.pose.pose.position.x,
                "y": msg.pose.pose.position.y,
                "linear_mps": linear_speed,
                "angular_radps": msg.twist.twist.angular.z,
                "stamp": stamp,
            }
            command_age = ""
            if self.command["stamp"] is not None:
                command_age = (stamp - self.command["stamp"]).to_sec()
            self.response_rows.append({
                "time_s": self._relative_time(stamp),
                "command_linear_mps": self.command["v"],
                "command_angular_radps": self.command["w"],
                "command_age_s": command_age,
                "actual_linear_mps": linear_speed,
                "actual_angular_radps": msg.twist.twist.angular.z,
                "left_wheel_mps": self.wheel_velocity["left"],
                "right_wheel_mps": self.wheel_velocity["right"],
                "left_tick": self.ticks["left"],
                "right_tick": self.ticks["right"],
                "x_m": self.latest_pose["x"],
                "y_m": self.latest_pose["y"],
            })

            if (self.pending_stop is not None and
                    (stamp - self.pending_stop["command_time"]).to_sec() > 0.0 and
                    linear_speed <= self.stop_speed_threshold):
                elapsed = (stamp - self.pending_stop["command_time"]).to_sec()
                distance = math.hypot(self.latest_pose["x"] - self.pending_stop["start_x"],
                                      self.latest_pose["y"] - self.pending_stop["start_y"])
                self.stop_rows.append({
                    "time_s": self._relative_time(stamp),
                    "initial_speed_mps": self.pending_stop["initial_speed"],
                    "stop_time_s": elapsed,
                    "coast_distance_m": distance,
                    "stop_speed_threshold_mps": self.stop_speed_threshold,
                })
                rospy.loginfo("drive_diagnostics: stop response %.3fs, %.3fm",
                              elapsed, distance)
                self.pending_stop = None

    def _write_csv(self, name, fieldnames, rows):
        path = os.path.join(self.output_dir, name)
        with open(path, "w", newline="") as output:
            writer = csv.DictWriter(output, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(rows)
        return path

    def shutdown(self):
        with self.lock:
            os.makedirs(self.output_dir, exist_ok=True)
            response_path = self._write_csv(
                "drive_response.csv",
                ["time_s", "command_linear_mps", "command_angular_radps", "command_age_s",
                 "actual_linear_mps", "actual_angular_radps", "left_wheel_mps",
                 "right_wheel_mps", "left_tick", "right_tick", "x_m", "y_m"],
                self.response_rows)
            command_path = self._write_csv(
                "drive_command_events.csv", ["time_s", "linear_mps", "angular_radps"],
                self.command_rows)
            ticks_path = self._write_csv(
                "drive_tick_events.csv", ["time_s", "left_tick", "right_tick"],
                self.tick_rows)
            stops_path = self._write_csv(
                "drive_stop_events.csv",
                ["time_s", "initial_speed_mps", "stop_time_s", "coast_distance_m",
                 "stop_speed_threshold_mps"], self.stop_rows)
            max_actual_speed = max(
                (row["actual_linear_mps"] for row in self.response_rows), default=0.0)
            summary = {
                "response_samples": len(self.response_rows),
                "command_events": len(self.command_rows),
                "tick_events": len(self.tick_rows),
                "completed_stop_events": len(self.stop_rows),
                "max_actual_linear_mps": max_actual_speed,
                "pending_stop_at_shutdown": self.pending_stop is not None,
            }
            summary_path = os.path.join(self.output_dir, "drive_diagnostics_summary.json")
            with open(summary_path, "w") as output:
                json.dump(summary, output, indent=2, sort_keys=True)

        rospy.loginfo("drive_diagnostics: wrote %s, %s, %s, %s, %s",
                      response_path, command_path, ticks_path, stops_path, summary_path)


if __name__ == "__main__":
    rospy.init_node("drive_diagnostics")
    DriveDiagnostics()
    rospy.spin()
