#!/usr/bin/env python3
"""
clear_scheduler.py
地图刷新调度器。

现在 clear_costmaps 和全局路径规划解绑：
  1. 新 goal 规划路径期间，pre_rotate 会通过 /clear_scheduler/pause 暂停 clear
  2. 路径生成完成后恢复 clear
  3. 有 active goal 后，每 1s 独立 clear 一次 costmap，不 cancel goal，不重发 goal

用法：rosrun encoder_tools clear_scheduler.py
"""

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty


class ClearScheduler:
    def __init__(self):
        self.refresh_period = rospy.get_param("~refresh_period", 1.0)
        self.paused = False
        self.have_goal = False
        self.clear_srv = None

        rospy.Subscriber("/clear_scheduler/pause", Bool, self.pause_cb, queue_size=10)
        rospy.Subscriber("/goal_rotated", PoseStamped, self.goal_cb, queue_size=10)
        rospy.Timer(rospy.Duration(self.refresh_period), self.timer_cb)

        rospy.loginfo(
            "clear_scheduler: started, clear_costmaps every %.2fs after goal, no replanning",
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
        if self.clear_srv is None:
            try:
                rospy.wait_for_service("/move_base/clear_costmaps", timeout=0.2)
                self.clear_srv = rospy.ServiceProxy("/move_base/clear_costmaps", Empty)
            except (rospy.ROSException, rospy.ServiceException):
                rospy.logwarn_throttle(2.0, "clear_scheduler: clear_costmaps service unavailable")
                return
        try:
            self.clear_srv()
            rospy.loginfo_throttle(1.0, "clear_scheduler: clear_costmaps called")
        except rospy.ServiceException as e:
            self.clear_srv = None
            rospy.logwarn_throttle(2.0, "clear_scheduler: clear_costmaps failed: %s", e)


if __name__ == "__main__":
    rospy.init_node("clear_scheduler")
    ClearScheduler()
    rospy.spin()
