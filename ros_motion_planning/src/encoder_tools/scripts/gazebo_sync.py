#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gazebo_sync.py
桥接节点：将 encoder_odom 的里程计位置同步到 Gazebo，
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
        # 从 encoder_odom 获取最新位姿
        self.latest_odom = None
        self.last_pose_time = rospy.Time(0)

        # Gazebo 服务
        rospy.wait_for_service("/gazebo/set_model_state", timeout=5)
        self.set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

        # 订阅 encoder_odom 的里程计
        rospy.Subscriber("/odom", Odometry, self.odom_cb, queue_size=10)

        # 修复 LaserScan 时间戳：重新发布
        self.scan_pub = rospy.Publisher("/scan_fixed", LaserScan, queue_size=10)
        rospy.Subscriber("/scan", LaserScan, self.scan_cb, queue_size=10)

        # 10Hz 定时同步 Gazebo 模型位姿
        rospy.Timer(rospy.Duration(0.1), self.sync_timer)

        rospy.loginfo("gazebo_sync 已启动")
        rospy.loginfo("  订阅 /odom → 每 0.1s 同步到 Gazebo 模型位姿")
        rospy.loginfo("  订阅 /scan → 重发到 /scan_fixed（修正时间戳）")

    def odom_cb(self, msg):
        self.latest_odom = msg

    def sync_timer(self, event):
        if self.latest_odom is None:
            return

        pose = self.latest_odom.pose.pose
        now = rospy.Time.now()

        try:
            model_state = ModelState()
            model_state.model_name = "turtlebot3_waffle"
            model_state.pose = pose
            model_state.twist = self.latest_odom.twist.twist
            # 空字符串 = 相对 Gazebo 世界坐标系
            # Gazebo 世界原点 = spawn_model 的 (0,0,0) = odom 坐标系原点
            model_state.reference_frame = ""

            self.set_state(model_state)
            self.last_pose_time = now
        except rospy.ServiceException as e:
            rospy.logwarn_throttle(5.0, f"设置 Gazebo 模型位姿失败: {e}")

    def scan_cb(self, msg):
        """修复 LaserScan 时间戳，用当前真实时间替换 Gazebo 仿真时间"""
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_scan"
        self.scan_pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("gazebo_sync")
    GazeboSync()
    rospy.spin()
