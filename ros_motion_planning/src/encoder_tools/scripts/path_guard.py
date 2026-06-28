#!/usr/bin/env python3
"""
path_guard.py
独立安全节点：拦截 move_base 内部产生的全局路径，检查是否穿过障碍物或
inflation 层，如果是则丢弃并用上一个好的 goal 强制重规划。

工作流：
  1. 监听 /goal_rotated（已预处理过的 goal，跳过 pre_rotate 重入）
  2. 监听 /move_base/GlobalPlanner/plan（新路径）
  3. 对新路径检查是否有任何点在 /map 静态地图的占据格上
  4. 路径正常 → 保存当前 goal 为 last_good_goal
  5. 路径异常 → 冷却期内忽略自身重发，用 last_good_goal 触发重规划

用法：rosrun encoder_tools path_guard.py
"""

import rospy
import math
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped

class PathGuard:
    def __init__(self):
        # ── 状态 ──
        self.last_good_goal = None      # 上次正常路径对应的 goal
        self.goal_pending = None        # 当前待检查的 goal
        self.goal_cooldown_until = 0.0  # 冷却截止时间（秒），防自循环

        # 静态地图数据
        self.map_loaded = False
        self.map_resolution = 0.0
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0
        self.map_width = 0
        self.map_height = 0
        self.map_data = None

        # ── 发布器：重发 goal 到 move_base（和 pre_rotate 输出同一个 topic）──
        self.goal_pub = rospy.Publisher("/goal_rotated", PoseStamped, queue_size=1)

        # ── 订阅器 ──
        rospy.Subscriber("/goal_rotated", PoseStamped, self.goal_cb, queue_size=10)
        rospy.Subscriber("/move_base/GlobalPlanner/plan", Path, self.plan_cb, queue_size=10)
        rospy.Subscriber("/map", OccupancyGrid, self.map_cb, queue_size=1)

        rospy.loginfo("path_guard: ✅ 已启动")

    # ─── /map 回调 ───
    def map_cb(self, msg):
        if self.map_loaded:
            return
        self.map_resolution = msg.info.resolution
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_data = msg.data
        self.map_loaded = True
        rospy.loginfo(f"path_guard: /map 已加载 {self.map_width}x{self.map_height}")

    # ─── /goal_rotated 回调 ───
    def goal_cb(self, msg):
        now = rospy.Time.now().to_sec()
        if now < self.goal_cooldown_until:
            # 冷却期内 → 这是自己重发的，忽略
            return
        self.goal_pending = msg
        rospy.loginfo("path_guard: 📥 收到新 goal，等待 plan 检查...")

    # ─── /move_base/GlobalPlanner/plan 回调（核心） ───
    def plan_cb(self, msg):
        if not msg.poses or self.goal_pending is None or not self.map_loaded:
            return

        # 检查：路径是否有任何点压在 /map 的占据格上
        if self._path_collides(msg):
            rospy.logwarn("path_guard: ⛔ 路径穿墙！丢弃并触发重规划")

            # 决定重发哪个 goal
            reuse = self.last_good_goal if self.last_good_goal is not None else self.goal_pending

            # 设冷却 1.0 秒，防止自循环
            self.goal_cooldown_until = rospy.Time.now().to_sec() + 1.0

            # 短暂等待让 costmap 恢复，再重发
            rospy.sleep(0.3)
            self.goal_pub.publish(reuse)
            rospy.loginfo("path_guard: 🔄 已重发 goal，触发重规划")

            self.goal_pending = None
            return

        # ✅ 路径安全
        self.last_good_goal = self.goal_pending
        rospy.loginfo("path_guard: ✅ 路径安全，已记忆此 goal")
        self.goal_pending = None

    # ─── 碰撞检测（纯 /map，不被 clear_costmaps 影响） ───
    def _path_collides(self, plan):
        """遍历路径所有点，有任意一个落在 /map 占据格 (>=75) 则返回 True"""
        for pose in plan.poses:
            wx = pose.pose.position.x
            wy = pose.pose.position.y
            mx = int((wx - self.map_origin_x) / self.map_resolution)
            my = int((wy - self.map_origin_y) / self.map_resolution)
            if mx < 0 or mx >= self.map_width or my < 0 or my >= self.map_height:
                return True
            idx = my * self.map_width + mx
            if idx < 0 or idx >= len(self.map_data):
                return True
            if self.map_data[idx] >= 75:
                return True
        return False


if __name__ == "__main__":
    rospy.init_node("path_guard")
    PathGuard()
    rospy.spin()
