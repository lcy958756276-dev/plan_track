#!/usr/bin/env python3
"""
clear_scheduler.py
按阻塞状态触发的地图刷新调度器。

  1. 新 goal 规划路径期间，pre_rotate 会通过 /clear_scheduler/pause 暂停 clear
  2. NMPC 因代价地图安全检查持续阻塞 1s 后，只 clear 一次 costmap
  3. 不 cancel goal、不重发 goal；地图恢复后控制器继续跟踪原路线
  4. 必须先恢复畅通，才会为下一次持续阻塞重新触发 clear

用法：rosrun encoder_tools clear_scheduler.py
"""

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty


class ClearScheduler:
    def __init__(self):
        self.blocked_duration = max(0.1, rospy.get_param("~blocked_duration", 1.0))
        self.check_period = max(0.02, rospy.get_param("~check_period", 0.1))
        self.blocked_topic = rospy.get_param(
            "~blocked_topic", "/move_base/DoubleNMPCController/path_blocked"
        )
        self.paused = False
        self.have_goal = False
        self.blocked = False
        self.blocked_since = None
        self.clear_armed = True
        self.clear_srv = None

        rospy.Subscriber("/clear_scheduler/pause", Bool, self.pause_cb, queue_size=10)
        rospy.Subscriber("/goal_rotated", PoseStamped, self.goal_cb, queue_size=10)
        rospy.Subscriber(self.blocked_topic, Bool, self.blocked_cb, queue_size=10)
        rospy.Timer(rospy.Duration(self.check_period), self.timer_cb)

        rospy.loginfo(
            "clear_scheduler: started, clear after %.2fs sustained path blockage on %s",
            self.blocked_duration,
            self.blocked_topic,
        )

    def pause_cb(self, msg):
        self.paused = msg.data
        rospy.loginfo("clear_scheduler: pause=%s", self.paused)

    def goal_cb(self, msg):
        self.have_goal = True
        self.blocked = False
        self.blocked_since = None
        self.clear_armed = True
        rospy.loginfo(
            "clear_scheduler: active goal=(%.3f, %.3f), blockage-triggered refresh enabled",
            msg.pose.position.x,
            msg.pose.position.y,
        )

    def blocked_cb(self, msg):
        now = rospy.Time.now()
        if msg.data:
            if not self.blocked:
                self.blocked_since = now
            self.blocked = True
            return

        if self.blocked:
            rospy.loginfo("clear_scheduler: path clear, trigger re-armed")
        self.blocked = False
        self.blocked_since = None
        self.clear_armed = True

    def timer_cb(self, event):
        if self.paused or not self.have_goal or not self.blocked:
            return
        if not self.clear_armed or self.blocked_since is None:
            return
        blocked_time = (rospy.Time.now() - self.blocked_since).to_sec()
        if blocked_time < self.blocked_duration:
            return
        if self._call_clear():
            self.clear_armed = False
            rospy.logwarn(
                "clear_scheduler: path blocked for %.2fs; cleared costmaps once, goal kept active",
                blocked_time,
            )

    def _call_clear(self):
        if self.clear_srv is None:
            try:
                rospy.wait_for_service("/move_base/clear_costmaps", timeout=0.2)
                self.clear_srv = rospy.ServiceProxy("/move_base/clear_costmaps", Empty)
            except (rospy.ROSException, rospy.ServiceException):
                rospy.logwarn_throttle(2.0, "clear_scheduler: clear_costmaps service unavailable")
                return False
        try:
            self.clear_srv()
            return True
        except rospy.ServiceException as e:
            self.clear_srv = None
            rospy.logwarn_throttle(2.0, "clear_scheduler: clear_costmaps failed: %s", e)
            return False


if __name__ == "__main__":
    rospy.init_node("clear_scheduler")
    ClearScheduler()
    rospy.spin()
