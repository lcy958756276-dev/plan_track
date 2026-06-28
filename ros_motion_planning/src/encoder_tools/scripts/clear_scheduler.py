#!/usr/bin/env python3
"""
clear_scheduler.py
同步 clear_costmaps 和规划周期的调度器。

思路：不让 clear 和 planner 异步跑，而是：
  1. 正常规划 N 次 (N=4)
  2. 第 N 次完毕后 → clear_costmaps
  3. 等待 costmap 恢复（~0.7s > 一个更新周期 0.5s）
  4. 恢复规划 → 再跑 N 次 → 重复

通过 cancel goal + 重发 goal 来实现规划暂停与恢复。

用法：rosrun encoder_tools clear_scheduler.py
"""

import rospy
import math
import actionlib
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction
from std_srvs.srv import Empty


class ClearScheduler:
    # 每 N 次规划后清一次
    PLANS_PER_CYCLE = 4
    # 清完后等待 costmap 恢复（s），> 1/update_frequency
    RECOVERY_WAIT = 0.7

    def __init__(self):
        self.plan_count = 0          # 当前周期的规划次数
        self.last_goal = None        # 最近一个正常 goal
        self.clearing = False        # 正在清空恢复中
        self.goal_from_self = False  # 是自己发的 goal 吗

        # ── 发布器 ──
        self.goal_pub = rospy.Publisher("/goal_rotated", PoseStamped, queue_size=1)

        # ── 订阅器 ──
        rospy.Subscriber("/goal_rotated", PoseStamped, self.goal_cb, queue_size=10)
        rospy.Subscriber(
            "/move_base/GlobalPlanner/plan", Path, self.plan_cb, queue_size=10
        )

        # ── Action client（用于 cancel goal）──
        self.move_client = actionlib.SimpleActionClient(
            "move_base", MoveBaseAction
        )
        rospy.loginfo("clear_scheduler: 等待 move_base action server...")
        if self.move_client.wait_for_server(timeout=rospy.Duration(5.0)):
            rospy.loginfo("clear_scheduler: ✅ move_base action 已连接")
        else:
            rospy.logwarn("clear_scheduler: ⚠ move_base action 超时，取消功能不可用")

        # ── Service client ──
        self.clear_srv = None

        rospy.loginfo(
            f"clear_scheduler: 已启动，每 {self.PLANS_PER_CYCLE} 次规划清一次"
        )

    # ── /goal_rotated 回调 ──
    def goal_cb(self, msg):
        if self.goal_from_self:
            # 自己发的 goal，不覆盖 last_goal（last_goal 记录的是"用户 goal"）
            return
        self.last_goal = msg
        rospy.loginfo("clear_scheduler: 📥 记录新 goal")

    # ── /move_base/GlobalPlanner/plan 回调 ──
    def plan_cb(self, msg):
        if not msg.poses:
            return

        # 清空恢复中：收到的 plan 是恢复前的残留，忽略
        if self.clearing:
            return

        self.plan_count += 1

        if self.plan_count >= self.PLANS_PER_CYCLE:
            self.plan_count = 0
            self._do_clear_cycle()

    def _do_clear_cycle(self):
        """执行一次 清空 → 等待恢复 → 重发 goal 的完整周期"""
        rospy.loginfo("clear_scheduler: 🔄 开始清空周期")

        # 记录当前 goal，后续重发要用
        goal = self.last_goal
        if goal is None:
            rospy.logwarn("clear_scheduler: ⚠ 没有可用的 goal，跳过清空")
            return

        # 1. 标记恢复中，后续 plan_cb 忽略
        self.clearing = True

        # 2. Cancel 当前 goal → move_base 停止规划+执行
        if self.move_client.gh is not None and self.move_client.gh.is_alive():
            self.move_client.cancel_all_goals()
            rospy.loginfo("clear_scheduler: 🛑 已 cancel 当前 goal")
            # 等一小段时间让 cancel 生效
            rospy.sleep(0.1)

        # 3. 调用 clear_costmaps
        self._call_clear()

        # 4. 等待 costmap 完全恢复（> 1 个更新周期）
        rospy.sleep(self.RECOVERY_WAIT)
        rospy.loginfo("clear_scheduler: ⏳ costmap 恢复完成")

        # 5. 重发 goal → move_base 重新规划（从当前位置到目标）
        #    需要设置 flag 防止 goal_cb 覆盖 last_goal
        self.goal_from_self = True
        self.goal_pub.publish(goal)
        self.goal_from_self = False

        # 6. 恢复规划计数和标记
        self.clearing = False
        rospy.loginfo("clear_scheduler: ✅ 清空周期结束，恢复规划")

    def _call_clear(self):
        """调用 clear_costmaps"""
        if self.clear_srv is None:
            try:
                rospy.wait_for_service("/move_base/clear_costmaps", timeout=0.5)
                self.clear_srv = rospy.ServiceProxy(
                    "/move_base/clear_costmaps", Empty
                )
            except (rospy.ROSException, rospy.ServiceException):
                rospy.logwarn("clear_scheduler: ⚠ clear_costmaps 服务不可用")
                return
        try:
            self.clear_srv()
            rospy.loginfo("clear_scheduler: 🧹 clear_costmaps 已调用")
        except rospy.ServiceException:
            self.clear_srv = None


if __name__ == "__main__":
    rospy.init_node("clear_scheduler")
    ClearScheduler()
    rospy.spin()
