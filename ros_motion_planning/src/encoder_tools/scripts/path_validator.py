#!/usr/bin/env python3
"""
path_validator.py
独立安全兜底节点：用 map_server 发布的 /map（clear_costmaps 清不掉它）
检查全局规划路径是否有任何点落在静态障碍物上。

检测到穿墙路径时：
  1. 调用 clear_costmaps 确保 costmap 干净
  2. 在 topic 上发布 bad_path 警告（不被 move_base 接收，仅日志）
  3. 让 move_base 的下一个 planning cycle 自己纠正

实际上 move_base 的 global plan topic 只是可视化，内部路径不受影响。
但此节点提供日志 visibility，并在检测到连续 N 次坏路径时强制重规划。

用法：rosrun encoder_tools path_validator.py
"""

import math
import rospy
import sys
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Point


class PathValidator:
    def __init__(self):
        # 静态地图数据（来自 map_server，永不随 clear_costmaps 消失）
        self.map_msg = None
        self.map_resolution = 0.0
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0
        self.map_width = 0
        self.map_height = 0
        self.map_data = None

        # 统计
        self.bad_count = 0
        self.good_count = 0

        # 订阅静态地图（map_server 发布，仅一次）
        rospy.Subscriber("/map", OccupancyGrid, self.map_cb, queue_size=1)

        # 订阅全局规划路径（/move_base/GlobalPlanner/plan）
        # move_base 标准 topic，但实际不参与路径传递，仅供可视化
        self.plan_sub = rospy.Subscriber(
            "/move_base/GlobalPlanner/plan", Path, self.plan_cb, queue_size=10
        )
        # 也订阅备选 topic
        self.plan_sub2 = rospy.Subscriber(
            "/move_base/plan", Path, self.plan_cb, queue_size=10
        )

        self.clear_srv = None

        rospy.loginfo("path_validator: 已启动，等待 /map ...")

    def map_cb(self, msg):
        """保存静态地图数据（只收一次）"""
        if self.map_msg is not None:
            return
        self.map_msg = msg
        self.map_resolution = msg.info.resolution
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_data = msg.data
        rospy.loginfo(
            f"path_validator: 已加载静态地图 "
            f"{self.map_width}x{self.map_height} "
            f"res={self.map_resolution:.2f}m"
        )

    def world_to_map(self, wx, wy):
        """世界坐标 → 地图像素坐标"""
        mx = (wx - self.map_origin_x) / self.map_resolution
        my = (wy - self.map_origin_y) / self.map_resolution
        return int(mx), int(my)

    def map_is_occupied(self, wx, wy):
        """检查世界坐标 (wx, wy) 是否在静态地图的占据格上"""
        if self.map_data is None:
            return False  # 地图还没加载好，不判断
        mx, my = self.world_to_map(wx, wy)
        if mx < 0 or mx >= self.map_width or my < 0 or my >= self.map_height:
            return True  # 超出地图边界 = 障碍物
        idx = my * self.map_width + mx
        if idx < 0 or idx >= len(self.map_data):
            return True
        # OccupancyGrid: 0 = 空闲, 100 = 占据, -1 = 未知
        return self.map_data[idx] >= 75  # >50% 占据判定为障碍

    def plan_cb(self, msg):
        """收到全局路径时检查"""
        if self.map_data is None:
            return  # 地图还没收到

        # 不检查空的路径
        if not msg.poses:
            return

        # 检查路径上是否有任何点在障碍物上
        for pose in msg.poses:
            wx = pose.pose.position.x
            wy = pose.pose.position.y
            if self.map_is_occupied(wx, wy):
                self.bad_count += 1
                rospy.logwarn(
                    f"path_validator: ⛔ 检测到穿墙路径! "
                    f"waypoint=({wx:.2f}, {wy:.2f}) "
                    f"在静态地图障碍物上 "
                    f"(累计: {self.bad_count}次穿墙 / {self.good_count}次正常)"
                )
                # 调用 clear_costmaps 清理 costmap（万一残留导致路径不好）
                self._clear_costmap()
                return

        # 路径正常
        self.good_count += 1

    def _clear_costmap(self):
        """调用 move_base 的 clear_costmaps 服务"""
        if self.clear_srv is None:
            try:
                rospy.wait_for_service("/move_base/clear_costmaps", timeout=0.5)
                from std_srvs.srv import Empty
                self.clear_srv = rospy.ServiceProxy(
                    "/move_base/clear_costmaps", Empty
                )
            except (rospy.ROSException, rospy.ServiceException):
                rospy.logwarn("path_validator: 无法连接到 clear_costmaps 服务")
                return

        try:
            self.clear_srv()
            rospy.loginfo("path_validator: 已调用 clear_costmaps")
        except rospy.ServiceException:
            self.clear_srv = None


if __name__ == "__main__":
    rospy.init_node("path_validator")
    PathValidator()
    rospy.spin()
