#!/usr/bin/env python3
"""
clear_scheduler.py
地图刷新调度器。

静态地图和动态激光障碍必须分开处理：
  1. 不能周期调用 /move_base/clear_costmaps；该服务会 reset 所有 layer，
     包括 static layer。static layer 重新接收 /map 前，重规划可能短暂看不到
     墙和固定障碍。
  2. 红色点云对应的 ObstacleLayer 已配置 clearing=true 和
     observation_persistence=0，由每帧 /scan_fixed 自动标记和射线清除。
  3. 本节点保留 pause/goal 状态和诊断日志，但绝不重置 static layer。

用法：rosrun encoder_tools clear_scheduler.py
"""

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped


class ClearScheduler:
    def __init__(self):
        self.refresh_period = rospy.get_param("~refresh_period", 1.0)
        self.paused = False
        self.have_goal = False

        rospy.Subscriber("/clear_scheduler/pause", Bool, self.pause_cb, queue_size=10)
        rospy.Subscriber("/goal_rotated", PoseStamped, self.goal_cb, queue_size=10)
        rospy.Timer(rospy.Duration(self.refresh_period), self.timer_cb)

        rospy.loginfo(
            "clear_scheduler: started; static costmap is never reset, "
            "dynamic obstacle layer refreshes from /scan_fixed (period %.2fs diagnostic)",
            self.refresh_period,
        )

    def pause_cb(self, msg):
        self.paused = msg.data
        rospy.loginfo("clear_scheduler: pause=%s", self.paused)

    def goal_cb(self, msg):
        self.have_goal = True
        rospy.loginfo(
            "clear_scheduler: active goal=(%.3f, %.3f), periodic map refresh enabled",
            msg.pose.position.x,
            msg.pose.position.y,
        )

    def timer_cb(self, event):
        if self.paused or not self.have_goal:
            return
        self._call_clear()

    def _call_clear(self):
        # Do not call /move_base/clear_costmaps here.  It resets StaticLayer as
        # well as ObstacleLayer and creates a planning window with no static
        # obstacles.  Dynamic red point-cloud cells are refreshed by the laser
        # obstacle layer itself (clearing=true, observation_persistence=0).
        rospy.loginfo_throttle(
            5.0,
            "clear_scheduler: preserving static costmap; dynamic obstacle layer "
            "is refreshed only by incoming /scan_fixed",
        )


if __name__ == "__main__":
    rospy.init_node("clear_scheduler")
    ClearScheduler()
    rospy.spin()
