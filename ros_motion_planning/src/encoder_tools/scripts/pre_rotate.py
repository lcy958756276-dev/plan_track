#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pre_rotate.py
在每次新 goal 到达时判断车头朝向与目标方向的角度差，
如果超过阈值则先原地旋转，转到位后再把 goal 转发给 move_base。

话题:
  订阅: /move_base_simple/goal  (来自 RViz 2D Nav Goal)
  订阅: /odom                   (获取当前位置和朝向)
  发布: /goal_rotated           (转发给 move_base)
  发布: /cmd_vel                (原地旋转时发给电机)

参数:
  ~angle_threshold (float): 触发旋转的角度阈值（度），默认 15
  ~alignment_tol   (float): 旋转到位容忍度（度），默认 3
  ~max_angular     (float): 原地旋转最大角速度 (rad/s)，默认 1.5

用法（由 run_debug.sh 自动启动）:
    rosrun encoder_tools pre_rotate.py
"""

import rospy
import math
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


class PreRotate:
    def __init__(self):
        self.angle_threshold = math.radians(rospy.get_param("~angle_threshold", 15.0))
        self.alignment_tol = math.radians(rospy.get_param("~alignment_tol", 3.0))
        self.max_angular = rospy.get_param("~max_angular", 1.5)

        # 当前位姿
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        # 旋转状态
        self.is_rotating = False
        self.pending_goal = None

        # 发布器
        self.goal_pub = rospy.Publisher("/goal_rotated", PoseStamped, queue_size=1)
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        # 订阅器
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_cb, queue_size=1)
        rospy.Subscriber("/odom", Odometry, self.odom_cb, queue_size=1)

        rospy.loginfo(
            f"[pre_rotate] 启动: 阈值={self.angle_threshold*180/math.pi:.0f}°"
        )

    def odom_cb(self, msg):
        """更新当前位置和朝向"""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.current_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        # 如果在旋转中，持续检查是否转到位
        if self.is_rotating and self.pending_goal is not None:
            self._check_rotation()

    def goal_cb(self, msg):
        """收到新 goal"""
        if self.is_rotating:
            # 旋转过程中又来了新 goal → 更新目标，继续旋转
            self.pending_goal = msg
            rospy.loginfo("[pre_rotate] 旋转中收到新 goal，更新目标")
            return

        # 计算目标方向
        dx = msg.pose.position.x - self.current_x
        dy = msg.pose.position.y - self.current_y
        target_yaw = math.atan2(dy, dx)
        angle_error = self._normalize_angle(target_yaw - self.current_yaw)

        if abs(angle_error) > self.angle_threshold:
            self.is_rotating = True
            self.pending_goal = msg
            rospy.loginfo(
                f"[pre_rotate] 角度偏差 {angle_error*180/math.pi:.1f}° > "
                f"{self.angle_threshold*180/math.pi:.0f}°，开始原地旋转"
            )
        else:
            # 偏差小，直接转发
            self.goal_pub.publish(msg)

    def _check_rotation(self):
        """检查旋转是否到位"""
        dx = self.pending_goal.pose.position.x - self.current_x
        dy = self.pending_goal.pose.position.y - self.current_y
        if dx == 0 and dy == 0:
            return  # 还没收到有效的 odom

        target_yaw = math.atan2(dy, dx)
        angle_error = self._normalize_angle(target_yaw - self.current_yaw)

        if abs(angle_error) < self.alignment_tol:
            # 转到位了！
            rospy.loginfo("[pre_rotate] 已对齐，转发 goal 到 move_base")
            self.goal_pub.publish(self.pending_goal)
            self.is_rotating = False
            self.pending_goal = None
            # 停止旋转
            stop = Twist()
            self.cmd_pub.publish(stop)
        else:
            # 继续旋转（恒定角速度，防止 odom 噪声导致轮子抖动）
            twist = Twist()
            twist.angular.z = math.copysign(self.max_angular, angle_error)
            )
            self.cmd_pub.publish(twist)

    @staticmethod
    def _normalize_angle(angle):
        """归一化到 [-pi, pi]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


if __name__ == "__main__":
    rospy.init_node("pre_rotate")
    PreRotate()
    rospy.spin()
