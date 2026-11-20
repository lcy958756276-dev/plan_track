#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gazebo_sync.py
桥接节点：不断尝试通过 /gz_debug/set_model_state 将 encoder_odom 的位置同步到 Gazebo。
如果服务不可用，持续重试直至成功（不依赖 /cmd_vel）。

同时将 Gazebo 激光与采样时的仿真雷达位姿配对后转发给导航。

用法:
    rosrun encoder_tools gazebo_sync.py
"""

from bisect import bisect_left
from collections import deque
from copy import deepcopy
import math
from threading import Lock
from time import monotonic

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from tf.transformations import quaternion_slerp


def interpolate_scan_pose(samples, stamp):
    """Return position, orientation, and bracket width in Gazebo sim time."""
    index = bisect_left([sample[0] for sample in samples], stamp)
    if index < len(samples) and samples[index][0] == stamp:
        _, position, orientation = samples[index]
        return position, orientation, 0.0
    if index == 0 or index == len(samples):
        return None
    before, p0, q0 = samples[index - 1]
    after, p1, q1 = samples[index]
    span = after - before
    if span > 0.15:
        return None
    fraction = (stamp - before) / span
    position = tuple(a + fraction * (b - a) for a, b in zip(p0, p1))
    return position, quaternion_slerp(q0, q1, fraction), span


class GazeboSync:
    def __init__(self):
        self.latest_odom = None
        self.got_odom = False
        self.have_service = False
        self.set_state = None
        self.service_retry_delay = 0.5  # 初始重试间隔（指数退避）
        self.model_synced = False
        self.scan_lock = Lock()
        self.scan_poses = deque(maxlen=128)
        self.pending_scans = deque(maxlen=4)
        self.previous_scan_arrival = None
        self.scan_publish_times = deque(maxlen=12)

        # ── 订阅 /odom ──
        rospy.Subscriber("/odom", Odometry, self.odom_cb, queue_size=10)

        # Pair Gazebo samples in sim time, then publish the matched scan and TF
        # together on the navigation stack's wall-clock timeline.
        self.scan_tf = tf2_ros.TransformBroadcaster()
        self.scan_pub = rospy.Publisher("/scan_fixed", LaserScan, queue_size=10)
        rospy.Subscriber("/gazebo_scan_pose", Odometry, self.scan_pose_cb, queue_size=100)
        rospy.Subscriber("/scan", LaserScan, self.scan_cb, queue_size=4)
        rospy.Timer(rospy.Duration(0.05), self.publish_aligned_scans)

        # 注：clear_costmaps 由 clear_scheduler 同步调度，不再在此异步定时
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
            odom = self.latest_odom
            pose = deepcopy(odom.pose.pose)

            wheel_radius = 0.1065    # 与 encoder_odom.py 保持一致
            joint_z = 0.03            # 轮子 joint 在 base_link 下的 z 偏移
            ground_clearance = 0.005  # 微小离地间隙，避免碰撞穿透
            model_z = wheel_radius - joint_z + ground_clearance  # base_link 高度，使轮底贴地

            state = ModelState()
            state.model_name = "my_robot"
            state.pose = pose
            state.pose.position.z = model_z
            state.twist = odom.twist.twist
            state.reference_frame = ""

            response = self.set_state(state)
            if not response.success:
                rospy.logwarn_throttle(2.0, "Gazebo model sync declined: %s", response.status_message)
                return
            if not self.model_synced:
                with self.scan_lock:
                    self.scan_poses.clear()
                    self.model_synced = True

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

    def scan_pose_cb(self, msg):
        stamp = msg.header.stamp.to_sec()
        if not self.model_synced or stamp <= 0.0:
            return
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        with self.scan_lock:
            if self.scan_poses and stamp < self.scan_poses[-1][0]:
                self.scan_poses.clear()
                self.pending_scans.clear()
                rospy.logwarn("Gazebo scan clock reset; cleared pending samples")
            self.scan_poses.append((stamp, (p.x, p.y, p.z), (q.x, q.y, q.z, q.w)))

    def scan_cb(self, msg):
        if self.model_synced and msg.header.stamp.to_sec() > 0.0:
            with self.scan_lock:
                self.pending_scans.append((monotonic(), msg))

    def publish_aligned_scans(self, _event):
        ready = []
        now = monotonic()
        with self.scan_lock:
            samples = list(self.scan_poses)
            while self.pending_scans:
                arrived, msg = self.pending_scans[0]
                sim_stamp = msg.header.stamp.to_sec()
                pose = interpolate_scan_pose(samples, sim_stamp)
                if now - arrived > 0.75:
                    self.pending_scans.popleft()
                    rospy.logwarn_throttle(2.0, "[SCAN_ALIGN] dropped sim_stamp=%.3f: pose unavailable or scan too old", sim_stamp)
                    continue
                if pose is None:
                    break
                self.pending_scans.popleft()
                ready.append((arrived, msg, pose))
        for arrived, msg, pose in ready:
            self.publish_scan(arrived, msg, pose)

    def publish_scan(self, arrived, msg, pose):
        """Publish the scan from its captured Gazebo origin for marking and clearing."""
        # The new_robot collision mesh is below the laser plane. Forward its
        # ranges as measured so nearby returns and clearing rays keep their meaning.
        position, orientation, bracket_s = pose
        sim_stamp = msg.header.stamp.to_sec()
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        # set_model_state places the encoder odometry pose in Gazebo world.
        transform.header.frame_id = "odom"
        transform.child_frame_id = "gazebo_scan_capture"
        transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z = position
        rotation = transform.transform.rotation
        rotation.x, rotation.y, rotation.z, rotation.w = map(float, orientation)
        # ObstacleLayer also asks for TF 50 ms beyond each LaserScan stamp.
        # Hold the captured origin for that interval so a slow next scan
        # cannot delay processing of this one.
        end_transform = deepcopy(transform)
        end_transform.header.stamp += rospy.Duration(0.05)
        self.scan_tf.sendTransform([transform, end_transform])

        msg.header.stamp = transform.header.stamp
        msg.header.frame_id = transform.child_frame_id
        # Gazebo ray scans are snapshots and this TF describes that snapshot.
        msg.time_increment = 0.0
        self.scan_pub.publish(msg)
        published_at = monotonic()
        period = 0.0 if self.previous_scan_arrival is None else arrived - self.previous_scan_arrival
        self.previous_scan_arrival = arrived

        # Measure the same delivered scans in Gazebo time and wall time.
        if self.scan_publish_times and sim_stamp <= self.scan_publish_times[-1][0]:
            self.scan_publish_times.clear()
        self.scan_publish_times.append((sim_stamp, published_at))
        first_sim, first_publish = self.scan_publish_times[0]
        sim_span = sim_stamp - first_sim
        publish_span = published_at - first_publish
        intervals = len(self.scan_publish_times) - 1
        sim_hz = intervals / sim_span if sim_span > 0.0 else 0.0
        fixed_hz = intervals / publish_span if publish_span > 0.0 else 0.0
        sim_wall_ratio = sim_span / publish_span if publish_span > 0.0 else 0.0
        valid_returns = sum(msg.range_min <= r <= msg.range_max for r in msg.ranges)
        no_returns = sum(r == math.inf for r in msg.ranges)
        rospy.loginfo_throttle(
            5.0,
            "[SCAN_ALIGN] sim_stamp=%.3f wall_stamp=%.3f scan_period_s=%.3f "
            "scan_sim_hz=%.2f scan_fixed_hz=%.2f sim_wall_ratio=%.3f "
            "pair_wait_s=%.3f sim_bracket_s=%.3f scan_origin=(%.3f,%.3f,%.3f) "
            "rays=%d valid_returns=%d no_returns=%d",
            sim_stamp, msg.header.stamp.to_sec(), period, sim_hz, fixed_hz, sim_wall_ratio,
            published_at - arrived,
            bracket_s, *position, len(msg.ranges), valid_returns, no_returns,
        )


if __name__ == "__main__":
    rospy.init_node("gazebo_sync")
    GazeboSync()
    rospy.spin()
