#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gazebo_sync.py
桥接节点：不断尝试通过 /gz_debug/set_model_state 将 encoder_odom 的位置同步到 Gazebo。
如果服务不可用，持续重试直至成功（不依赖 /cmd_vel）。

同时修复 LaserScan 时间戳。

用法:
    rosrun encoder_tools gazebo_sync.py
"""

import math
import rospy
import sys
import os
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState


class GazeboSync:
    def __init__(self):
        self.latest_odom = None
        self.got_odom = False
        self.have_service = False
        self.set_state = None
        self.service_retry_delay = 0.5  # 初始重试间隔（指数退避）

        # ── 订阅 /odom ──
        rospy.Subscriber("/odom", Odometry, self.odom_cb, queue_size=10)

        # ── LaserScan 时间戳修复 ──
        self.scan_pub = rospy.Publisher("/scan_fixed", LaserScan, queue_size=10)
        rospy.Subscriber("/scan", LaserScan, self.scan_cb, queue_size=10)

        # ── 周期性清理 costmap 噪声累积 ──
        # 10 秒清一次，清除累积噪声，同时给规划器充足时间避开真正障碍物
        self.clear_costmap_srv = None
        rospy.Timer(rospy.Duration(10.0), self.clear_costmap_timer)

        # ── 主循环（5Hz）：不断尝试服务 + 同步位置 ──
        # 注意：10Hz 太频繁，set_model_state 容易超时（returned no response）
        rospy.Timer(rospy.Duration(0.2), self.main_timer)

        rospy.loginfo("gazebo_sync 已启动")
        rospy.loginfo("  持续尝试连接 /gz_debug/set_model_state ...")

    def odom_cb(self, msg):
        self.latest_odom = msg
        self.got_odom = True

    def main_timer(self, event):
        now = rospy.Time.now()

        # ── 如果还没连上服务，持续尝试（指数退避） ──
        if not self.have_service:
            # 还没到重试时间就跳过
            if hasattr(self, '_next_retry') and rospy.Time.now().to_sec() < self._next_retry:
                return
            try:
                rospy.wait_for_service("/gz_debug/set_model_state", timeout=0.5)
                self.set_state = rospy.ServiceProxy(
                    "/gz_debug/set_model_state", SetModelState
                )
                self.have_service = True
                self.service_retry_delay = 0.5   # 成功后重置退避
                rospy.loginfo("  ✅ 已连接 /gz_debug/set_model_state")
            except rospy.ROSException:
                # 还没就绪，指数退避
                self._next_retry = rospy.Time.now().to_sec() + self.service_retry_delay
                self.service_retry_delay = min(self.service_retry_delay * 2, 10.0)  # 最大10秒
                rospy.loginfo_throttle(5.0, f"  等待 /gz_debug/set_model_state 服务就绪... (退避 {self.service_retry_delay:.0f}s)")
                # 每 10 秒打印一次所有 Gazebo 相关服务，诊断
                diag_counter = int(rospy.Time.now().to_sec() / 10)
                if not hasattr(self, '_last_diag') or self._last_diag != diag_counter:
                    self._last_diag = diag_counter
                    try:
                        master = rospy.get_master()
                        code, msg, all_svc = master.getServiceNames()
                        gz_svc = [s for s in all_svc if 'gz_debug' in s.lower() or 'gazebo' in s.lower()]
                        rospy.loginfo(f"  [诊断] Gazebo/gz_debug 服务: {gz_svc}")
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

            wheel_radius = 0.1065    # 与 encoder_odom.py 保持一致
            joint_z = 0.03            # 轮子 joint 在 base_link 下的 z 偏移
            ground_clearance = 0.005  # 微小离地间隙，避免碰撞穿透
            model_z = wheel_radius - joint_z + ground_clearance  # base_link 高度，使轮底贴地

            state = ModelState()
            state.model_name = "my_robot"
            state.pose = pose
            state.pose.position.z = model_z
            state.twist = twist
            state.reference_frame = ""

            self.set_state(state)

            rospy.loginfo_throttle(
                2.0,
                f"已同步 Gazebo 位置: x={pose.position.x:.2f} y={pose.position.y:.2f} θ={pose.orientation.z:.2f}",
            )
        except rospy.ServiceException as e:
            rospy.logwarn_throttle(5.0, f"set_model_state 失败: {e}")
            self.have_service = False  # 服务挂了，重新尝试
            self.set_state = None
            # 指数退避重试，防止高频重试把 Gazebo 打崩
            self.service_retry_delay = min(self.service_retry_delay * 2, 10.0)
            self._next_retry = rospy.Time.now().to_sec() + self.service_retry_delay

    def scan_cb(self, msg):
        """修复 LaserScan 时间戳 + 过滤自检测 + 去除孤点（混合像素）"""
        # 机器人足迹（相对 base_link）
        # footprint: [[-0.1, -0.3], [-0.1, 0.3], [0.8, 0.3], [0.8, -0.3]]
        # base_scan 在 base_link 下的坐标: x=0.19, y=0
        # 在 base_scan 坐标系内，车身范围大致为一个矩形
        SELF_MIN_X = -0.3   # 车身在 base_scan 背后的范围
        SELF_MAX_X = 0.2
        SELF_MIN_Y = -0.35
        SELF_MAX_Y = 0.35

        ranges = list(msg.ranges)
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment

        for i in range(len(ranges)):
            r = ranges[i]
            if r < msg.range_min or r > msg.range_max:
                continue

            # (1) 过滤自检测：计算该点在 base_scan 坐标系下的 (x, y)
            theta = angle_min + i * angle_increment
            px = r * math.cos(theta)
            py = r * math.sin(theta)
            if SELF_MIN_X <= px <= SELF_MAX_X and SELF_MIN_Y <= py <= SELF_MAX_Y:
                ranges[i] = msg.range_max + 1.0  # 设为无效
                continue

            # (2) 混合像素过滤：与左右邻居对比（用原始数据 msg.ranges，避免被自过滤影响）
            if 0 < i < len(ranges) - 1:
                r_prev = msg.ranges[i - 1]
                r_next = msg.ranges[i + 1]
                if r_prev < msg.range_max and r_next < msg.range_max:
                    if abs(r - r_prev) > 0.07 and abs(r - r_next) > 0.07:
                        ranges[i] = msg.range_max + 1.0  # 设为无效

        msg.ranges = tuple(ranges)
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_scan"
        self.scan_pub.publish(msg)

    def clear_costmap_timer(self, event):
        """定期清理 costmap 噪声累积"""
        if self.clear_costmap_srv is None:
            try:
                rospy.wait_for_service("/move_base/clear_costmaps", timeout=0.5)
                self.clear_costmap_srv = rospy.ServiceProxy(
                    "/move_base/clear_costmaps", Empty
                )
            except (rospy.ROSException, rospy.ServiceException):
                return

        try:
            self.clear_costmap_srv()
            rospy.loginfo_throttle(30.0, "已清理 costmap (10s 周期)")
        except rospy.ServiceException:
            self.clear_costmap_srv = None


if __name__ == "__main__":
    rospy.init_node("gazebo_sync")
    GazeboSync()
    rospy.spin()