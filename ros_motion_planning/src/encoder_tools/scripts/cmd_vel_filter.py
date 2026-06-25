#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
cmd_vel_filter.py
在 move_base 和 serial_bridge 之间做角度判断过滤。

功能:
  从 move_base 收到 /cmd_vel 后，如果发现机器人需要 "大角度转弯+前进"，
  则 override 为原地旋转（前进速度=0），防止转弯半径过大撞到障碍物。

架构:
  move_base → /cmd_vel → [本节点] → /cmd_vel_safe → serial_bridge → 电机

参数:
  ~angle_threshold (float): 角度阈值（弧度），默认 0.26 ≈ 15°
  ~max_angular    (float): 原地旋转最大角速度 (rad/s)，默认 1.5

用法（由 run_debug.sh 自动启动）:
    rosrun encoder_tools cmd_vel_filter.py
"""

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math


class CmdVelFilter:
    def __init__(self):
        self.angle_threshold = rospy.get_param("~angle_threshold", 0.26)   # ≈15°
        self.max_angular = rospy.get_param("~max_angular", 1.5)            # rad/s
        self.linear_min = rospy.get_param("~linear_min", 0.02)            # m/s，低于此视为静止

        # 当前车头朝向（来自 odom）
        self.current_yaw = 0.0
        self.odom_received = False

        # 发布过滤后的 cmd_vel
        self.pub = rospy.Publisher("/cmd_vel_safe", Twist, queue_size=1)

        # 订阅原始 cmd_vel（来自 move_base）
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_cb, queue_size=1)

        # 订阅 odom 获取当前朝向
        rospy.Subscriber("/odom", Odometry, self.odom_cb, queue_size=1)

        rospy.loginfo(
            f"[filter] 启动: angle_threshold={self.angle_threshold:.2f}rad "
            f"({self.angle_threshold*180/math.pi:.0f}°)"
        )

    def odom_cb(self, msg):
        """从 odom 提取当前 yaw"""
        q = msg.pose.pose.orientation
        # 四元数 → 欧拉角
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.odom_received = True

    def cmd_vel_cb(self, msg):
        """
        收到 move_base 的速度命令，判断是否需要拦截为大角度旋转。
        """
        vx = msg.linear.x
        wz = msg.angular.z

        # ── 情况 1：尝试同时前进 + 大幅度转弯 ──
        # 如果机器人既要往前走(|vx|>阈值)，又要转大弯(|wz|>阈值)，
        # 说明当前 heading 偏差大，应该原地先转
        if abs(vx) > self.linear_min and abs(wz) > self.angle_threshold:
            safe = Twist()
            # 只保留转向，去掉前进速度
            safe.angular.z = math.copysign(
                min(abs(wz), self.max_angular), wz
            )
            self.pub.publish(safe)

            rospy.loginfo_throttle(
                1.0,
                f"[filter] 大角度拦截: vx={vx:.2f}→0, wz={wz:.2f}→{safe.angular.z:.2f}"
            )
            return

        # ── 情况 2：正常通行 ──
        self.pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("cmd_vel_filter")
    CmdVelFilter()
    rospy.spin()
