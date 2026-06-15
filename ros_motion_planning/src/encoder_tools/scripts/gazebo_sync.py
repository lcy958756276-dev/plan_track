#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gazebo_sync.py
桥接节点：将 encoder_odom 的里程计位置定期同步到 Gazebo，
让 Gazebo 中的小车模型跟随实体车移动。

同时修复 LaserScan 时间戳，使 RViz 能正常显示点云。

用法:
    rosrun encoder_tools gazebo_sync.py
"""

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState


class GazeboSync:
    def __init__(self):
        self.latest_odom = None
        self.got_odom = False

        # ── 连接 Gazebo 服务（等它就绪，不限时）──
        rospy.loginfo("等待 /gazebo/set_model_state 服务...")
        try:
            rospy.wait_for_service("/gazebo/set_model_state", timeout=30)
            self.set_state = rospy.ServiceProxy(
                "/gazebo/set_model_state", SetModelState
            )
            rospy.loginfo("  /gazebo/set_model_state 已连接")
            self.have_service = True
        except rospy.ROSException:
            rospy.logwarn("  /gazebo/set_model_state 服务超时（30s），降级为只转发 scan")
            self.have_service = False
            self.set_state = None

        # ── 订阅 /odom ──
        rospy.Subscriber("/odom", Odometry, self.odom_cb, queue_size=10)

        # ── 10Hz 同步位置到 Gazebo ──
        if self.have_service:
            rospy.Timer(rospy.Duration(0.1), self.sync_timer)

        # ── LaserScan 时间戳修复 ──
        self.scan_pub = rospy.Publisher("/scan_fixed", LaserScan, queue_size=10)
        rospy.Subscriber("/scan", LaserScan, self.scan_cb, queue_size=10)

        rospy.loginfo("gazebo_sync 已启动")
        if self.have_service:
            rospy.loginfo("  订阅 /odom → 每 0.1s 同步到 Gazebo 模型位置")
        else:
            rospy.loginfo("  服务不可用，仅转发 /scan → /scan_fixed")

    def odom_cb(self, msg):
        self.latest_odom = msg
        self.got_odom = True

    def sync_timer(self, event):
        if not self.got_odom:
            return

        pose = self.latest_odom.pose.pose
        twist = self.latest_odom.twist.twist

        try:
            state = ModelState()
            state.model_name = "turtlebot3_waffle"
            state.pose = pose
            state.twist = twist
            state.reference_frame = ""  # Gazebo 世界坐标系
            self.set_state(state)
        except rospy.ServiceException as e:
            rospy.logwarn_throttle(5.0, f"set_model_state 失败: {e}")

    def scan_cb(self, msg):
        """修复 LaserScan 时间戳，用当前真实时间替换 Gazebo 仿真时间"""
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_scan"
        self.scan_pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("gazebo_sync")
    GazeboSync()
    rospy.spin()
