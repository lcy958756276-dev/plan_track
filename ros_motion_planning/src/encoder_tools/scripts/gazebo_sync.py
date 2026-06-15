#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gazebo_sync.py
桥接节点：将 encoder_odom 的里程计同步到 Gazebo。

策略（按优先级）:
  1. 直接 SetModelState（位置同步）
  2. 如果服务不可用 → 转发 /cmd_vel（速度驱动）

同时修复 LaserScan 时间戳。

用法:
    rosrun encoder_tools gazebo_sync.py
"""

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState


class GazeboSync:
    def __init__(self):
        self.latest_odom = None
        self.got_odom = False

        # ── 动态检测服务 ──
        self.have_service = False
        self.set_state = None
        self._try_connect_service()

        # ── 订阅 /odom ──
        rospy.Subscriber("/odom", Odometry, self.odom_cb, queue_size=10)

        # ── 发布 /cmd_vel（备选方案）──
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # ── 20Hz 主循环 ──
        rospy.Timer(rospy.Duration(0.05), self.main_timer)

        # ── LaserScan 时间戳修复 ──
        self.scan_pub = rospy.Publisher("/scan_fixed", LaserScan, queue_size=10)
        rospy.Subscriber("/scan", LaserScan, self.scan_cb, queue_size=10)

        rospy.loginfo("gazebo_sync 已启动")
        if self.have_service:
            rospy.loginfo("  策略: SetModelState（位置同步）")
        else:
            rospy.loginfo("  策略: /cmd_vel（速度驱动，无服务可用）")
        rospy.loginfo("  订阅 /scan → 重发到 /scan_fixed")

    def _try_connect_service(self):
        """尝试连接 Gazebo 服务（非阻塞）"""
        try:
            rospy.wait_for_service("/gazebo/set_model_state", timeout=2)
            self.set_state = rospy.ServiceProxy(
                "/gazebo/set_model_state", SetModelState
            )
            self.have_service = True
            rospy.loginfo("  已连接 /gazebo/set_model_state")
        except rospy.ROSException:
            self.have_service = False
            rospy.logwarn("  /gazebo/set_model_state 不可达，改用 /cmd_vel")

    def odom_cb(self, msg):
        self.latest_odom = msg
        self.got_odom = True

    def main_timer(self, event):
        if not self.got_odom:
            return

        if self.have_service:
            self._sync_position()
        else:
            self._publish_cmd_vel()

    def _sync_position(self):
        """方式1: 直接设置模型位姿"""
        try:
            state = ModelState()
            state.model_name = "turtlebot3_waffle"
            state.pose = self.latest_odom.pose.pose
            state.twist = self.latest_odom.twist.twist
            state.reference_frame = ""
            self.set_state(state)
        except rospy.ServiceException as e:
            rospy.logwarn_throttle(5.0, f"set_model_state 失败: {e}")
            # 服务挂了 → 降级到 cmd_vel
            self.have_service = False

    def _publish_cmd_vel(self):
        """方式2: 转发速度到 /cmd_vel"""
        twist = Twist()
        twist.linear.x = self.latest_odom.twist.twist.linear.x
        twist.angular.z = self.latest_odom.twist.twist.angular.z
        self.cmd_pub.publish(twist)

        # 每 2s 打印一次实际发出的速度（方便调试）
        rospy.loginfo_throttle(
            2.0, f"cmd_vel: v={twist.linear.x:.3f} ω={twist.angular.z:.3f}"
        )

    def scan_cb(self, msg):
        """修复 LaserScan 时间戳"""
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_scan"
        self.scan_pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("gazebo_sync")
    GazeboSync()
    rospy.spin()
