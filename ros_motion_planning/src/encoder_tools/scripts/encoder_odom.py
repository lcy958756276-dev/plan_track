#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
encoder_odom.py
订阅 /wheel_ticks (ltick, rtick) → 计算里程计 → 发布 /odom + TF

使用公式:
    pluse = 1.04190106 * 360 / (500 * 4 * 91)
    左轮距离增量 = Δltick * (pluse / 360) * 2π * wheel_radius
    右轮距离增量 = Δrtick * (pluse / 360) * 2π * wheel_radius
"""

import rospy
import math
from std_msgs.msg import Int64MultiArray, Float64, Float64MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
import tf.transformations as tf_trans


class EncoderOdometry:
    def __init__(self):
        # 物理参数
        wheel_radius = rospy.get_param("~wheel_radius", 0.1065)
        wheel_base   = rospy.get_param("~wheel_base", 0.245 * 2.05)

        # 编码器参数 → 每脉冲对应距离
        pluse = 1.04190106 * 360.0 / (500.0 * 4 * 91)
        self.dist_per_tick = (pluse / 360.0) * 2.0 * math.pi * wheel_radius
        self.wheel_base = wheel_base

        # 右轮编码器方向是否与左轮相反（差分驱动常见）
        self.right_reverse = rospy.get_param("~right_reverse", True)

        rospy.loginfo("=== encoder_odom 参数 ===")
        rospy.loginfo(f"wheel_radius    = {wheel_radius}")
        rospy.loginfo(f"wheel_base      = {wheel_base}")
        rospy.loginfo(f"pluse           = {pluse:.6f}")
        rospy.loginfo(f"dist_per_tick   = {self.dist_per_tick:.8f} m")
        rospy.loginfo(f"right_reverse   = {self.right_reverse} (右轮取反)")
        rospy.loginfo("========================")

        # 状态
        self.last_ltick = None
        self.last_rtick = None
        self.last_time = None
        self.left_dist = 0.0
        self.right_dist = 0.0
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.msg_count = 0

        # 发布器
        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=50)
        self.left_pub = rospy.Publisher("/left_wheel_distance", Float64, queue_size=10)
        self.right_pub = rospy.Publisher("/right_wheel_distance", Float64, queue_size=10)
        self.vel_pub = rospy.Publisher("/wheel_velocities", Float64MultiArray, queue_size=10)

        # TF
        self.tf_br = tf2_ros.TransformBroadcaster()

        # 订阅
        rospy.Subscriber("/wheel_ticks", Int64MultiArray, self.tick_cb, queue_size=100)

        rospy.loginfo("encoder_odom 已启动，等待 /wheel_ticks ...")
        rospy.loginfo("提示: 确保 read_uart.py 也在运行，否则收不到编码器数据")

    def tick_cb(self, msg):
        self.msg_count += 1

        if len(msg.data) < 2:
            rospy.logwarn("收到 /wheel_ticks 但数据长度 < 2")
            return

        ltick = msg.data[0]
        rtick = msg.data[1]

        # 第一次收到数据
        if self.last_ltick is None:
            self.last_ltick = ltick
            self.last_rtick = rtick
            self.last_time = rospy.Time.now()
            rospy.loginfo(f"收到第一条编码器数据: ltick={ltick}, rtick={rtick}")
            rospy.loginfo("里程计开始计算...")
            return

        left_inc = ltick - self.last_ltick
        right_inc = rtick - self.last_rtick

        # 防野值
        if abs(left_inc) > 1000000 or abs(right_inc) > 1000000:
            rospy.logwarn_throttle(5, f"编码器跳变过大: left_inc={left_inc}, right_inc={right_inc}，忽略")
            self.last_ltick = ltick
            self.last_rtick = rtick
            self.last_time = rospy.Time.now()
            return

        self.last_ltick = ltick
        self.last_rtick = rtick

        now = rospy.Time.now()
        dt = (now - self.last_time).to_sec()
        self.last_time = now

        if dt <= 0 or dt > 1.0:
            rospy.logwarn_throttle(3, f"时间间隔异常: dt={dt:.4f}s，跳过")
            return

        # ── 核心计算 ──
        # 右轮取反: 如果 right_reverse=True，把 right_inc 取反
        right_inc_adj = -right_inc if self.right_reverse else right_inc

        d_left  = left_inc * self.dist_per_tick
        d_right = right_inc_adj * self.dist_per_tick

        self.left_dist  += d_left
        self.right_dist += d_right

        v_left  = d_left / dt
        v_right = d_right / dt

        d_center = (d_left + d_right) / 2.0
        d_theta  = (d_right - d_left) / self.wheel_base

        self.theta += d_theta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        self.x += d_center * math.cos(self.theta)
        self.y += d_center * math.sin(self.theta)

        v = (v_left + v_right) / 2.0
        omega = (v_right - v_left) / self.wheel_base

        # 每隔 2 秒打印一次关键数据
        rospy.loginfo_throttle(2.0,
            f"编码器: ltick={ltick} rtick={rtick} "
            f"Δl={left_inc} Δr={right_inc}(调整后={right_inc_adj}) "
            f"d_left={d_left:.4f}m d_right={d_right:.4f}m"
        )
        rospy.loginfo_throttle(2.0,
            f"里程计: x={self.x:.3f} y={self.y:.3f} θ={self.theta:.3f} "
            f"v={v:.3f}m/s ω={omega:.3f}rad/s"
        )

        # 发布距离
        self.left_pub.publish(Float64(data=self.left_dist))
        self.right_pub.publish(Float64(data=self.right_dist))

        # 发布速度
        vel = Float64MultiArray()
        vel.data = [v_left, v_right, v, omega]
        self.vel_pub.publish(vel)

        # 发布 odom
        self._pub_odom(now, v, omega)

        # 发布 TF
        self._pub_tf(now)

    def _pub_odom(self, stamp, v, omega):
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        q = tf_trans.quaternion_from_euler(0, 0, self.theta)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = omega

        self.odom_pub.publish(odom)
        rospy.loginfo_throttle(2.0, f"已发布 /odom: pose=({self.x:.2f}, {self.y:.2f}, {self.theta:.2f})")

    def _pub_tf(self, stamp):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        q = tf_trans.quaternion_from_euler(0, 0, self.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_br.sendTransform(t)


if __name__ == "__main__":
    rospy.init_node("encoder_odometry")
    EncoderOdometry()
    rospy.spin()
