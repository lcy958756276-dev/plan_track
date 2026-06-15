#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gazebo_sync.py
桥接节点：不断尝试通过 /gazebo/set_model_state 将 encoder_odom 的位置同步到 Gazebo。
如果服务不可用，持续重试直至成功（不依赖 /cmd_vel）。

同时修复 LaserScan 时间戳。

用法:
    rosrun encoder_tools gazebo_sync.py
"""

import rospy
import sys
import os
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState


class GazeboSync:
    def __init__(self):
        self.latest_odom = None
        self.got_odom = False
        self.have_service = False
        self.set_state = None

        # ── 订阅 /odom ──
        rospy.Subscriber("/odom", Odometry, self.odom_cb, queue_size=10)

        # ── LaserScan 时间戳修复 ──
        self.scan_pub = rospy.Publisher("/scan_fixed", LaserScan, queue_size=10)
        rospy.Subscriber("/scan", LaserScan, self.scan_cb, queue_size=10)

        # ── 主循环（10Hz）：不断尝试服务 + 同步位置 ──
        rospy.Timer(rospy.Duration(0.1), self.main_timer)

        rospy.loginfo("gazebo_sync 已启动")
        rospy.loginfo("  持续尝试连接 /gazebo/set_model_state ...")

    def odom_cb(self, msg):
        self.latest_odom = msg
        self.got_odom = True

    def main_timer(self, event):
        now = rospy.Time.now()

        # ── 如果还没连上服务，持续尝试 ──
        if not self.have_service:
            try:
                rospy.wait_for_service("/gazebo/set_model_state", timeout=0.5)
                self.set_state = rospy.ServiceProxy(
                    "/gazebo/set_model_state", SetModelState
                )
                self.have_service = True
                rospy.loginfo("  ✅ 已连接 /gazebo/set_model_state")
            except rospy.ROSException:
                # 还没就绪，等下一轮
                rospy.loginfo_throttle(5.0, "  等待 /gazebo/set_model_state 服务就绪...")
                # 每 10 秒打印一次所有 Gazebo 相关服务，诊断
                diag_counter = int(rospy.Time.now().to_sec() / 10)
                if not hasattr(self, '_last_diag') or self._last_diag != diag_counter:
                    self._last_diag = diag_counter
                    try:
                        all_svc = rospy.get_service_names()
                        gazebo_svc = [s for s in all_svc if 'gazebo' in s.lower()]
                        rospy.loginfo(f"  [诊断] 当前 ROS 服务数={len(all_svc)}")
                        rospy.loginfo(f"  [诊断] Gazebo 相关服务: {gazebo_svc}")
                    except Exception as e:
                        rospy.logwarn(f"  [诊断] 获取服务列表失败: {e}")
                return
            except rospy.ServiceException as e:
                rospy.logwarn_throttle(5.0, f"  服务调用异常: {e}")
                return

        # ── 有了服务才能同步位置 ──
        if not self.got_odom:
            return

        try:
            pose = self.latest_odom.pose.pose
            twist = self.latest_odom.twist.twist

            state = ModelState()
            state.model_name = "turtlebot3_waffle"
            state.pose = pose
            state.twist = twist
            state.reference_frame = ""

            self.set_state(state)

            rospy.loginfo_throttle(
                2.0,
                f"已同步 Gazebo 位置: x={pose.position.x:.2f} y={pose.position.y:.2f} θ={pose.orientation.z:.2f}",
            )
        except rospy.ServiceException as e:
            rospy.logwarn(f"set_model_state 失败: {e}")
            self.have_service = False  # 服务挂了，重新尝试
            self.set_state = None

    def scan_cb(self, msg):
        """修复 LaserScan 时间戳"""
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_scan"
        self.scan_pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("gazebo_sync")
    GazeboSync()
    rospy.spin()
