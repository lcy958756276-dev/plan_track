#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gazebo_sync.py
桥接节点：将 encoder_odom 的速度转发到 Gazebo 的 /cmd_vel，
让 Gazebo 中的小车模型跟随实体车移动。

同时修复 LaserScan 时间戳，使 RViz 能正常显示点云。

用法:
    rosrun encoder_tools gazebo_sync.py
"""

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class GazeboSync:
    def __init__(self):
        # 最新速度（由 encoder_odom 的里程计更新）
        self.latest_v = 0.0
        self.latest_omega = 0.0

        # 发布 /cmd_vel → Gazebo diff_drive 插件
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # 订阅 encoder_odom 的里程计（从中提取速度）
        rospy.Subscriber("/odom", Odometry, self.odom_cb, queue_size=10)

        # 20Hz 定时发送速度到 Gazebo
        # encoder_odom 约 2Hz 更新一次，此处更频繁发布让 Gazebo 运动更平滑
        rospy.Timer(rospy.Duration(0.05), self.cmd_timer)

        # ── LaserScan 时间戳修复 ──
        self.scan_pub = rospy.Publisher("/scan_fixed", LaserScan, queue_size=10)
        rospy.Subscriber("/scan", LaserScan, self.scan_cb, queue_size=10)

        rospy.loginfo("gazebo_sync 已启动")
        rospy.loginfo("  订阅 /odom → 提取速度 → 发到 /cmd_vel 驱动 Gazebo 小车")
        rospy.loginfo("  订阅 /scan → 重发到 /scan_fixed（修正时间戳）")

    def odom_cb(self, msg):
        self.latest_v = msg.twist.twist.linear.x
        self.latest_omega = msg.twist.twist.angular.z

    def cmd_timer(self, event):
        """以 20Hz 将速度转发到 /cmd_vel，让 Gazebo diff_drive 驱动模型"""
        twist = Twist()
        twist.linear.x = self.latest_v
        twist.angular.z = self.latest_omega
        self.cmd_pub.publish(twist)

    def scan_cb(self, msg):
        """修复 LaserScan 时间戳，用当前真实时间替换 Gazebo 仿真时间"""
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_scan"
        self.scan_pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("gazebo_sync")
    GazeboSync()
    rospy.spin()
