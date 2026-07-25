#!/usr/bin/env python3
import rospy
import math
import copy
import actionlib
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetPlan
from move_base_msgs.msg import MoveBaseAction
from tf.transformations import euler_from_quaternion


class PreRotate:
    def __init__(self):
        self.alignment_tol = math.radians(3.0)
        self.max_angular = 0.45                         # rad/s，慢一点更稳
        self.plan_heading_dist = 0.35                    # 沿全局路径取多远的点来决定初始朝向
        self.plan_retry_timeout = 3.0                    # 等待全局规划结果的最长时间
        self.stop_before_rotate = 1.0                    # 新 goal 后先停稳，再按路径方向旋转
        self.plan_goal_tolerance = 0.3                   # 防止 make_plan 返回上一条旧路径
        self.make_plan_service = "/move_base/PathPlanner/make_plan"

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.rotating = False
        self.goal = None
        self.target_yaw = None
        self.make_plan = None
        self.move_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        self.goal_pub = rospy.Publisher("/goal_rotated", PoseStamped, queue_size=1)
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.clear_pause_pub = rospy.Publisher(
            "/clear_scheduler/pause", Bool, queue_size=1, latch=True
        )

        rospy.loginfo(
            "pre_rotate: debug build, make_plan_service=%s plan_goal_tolerance=%.3fm",
            self.make_plan_service,
            self.plan_goal_tolerance,
        )
        self._set_clear_paused(False, "startup")

        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_cb)
        rospy.Subscriber("/odom", Odometry, self.odom_cb)

        # 50Hz 定时检查旋转状态，比等 odom 回调更及时
        rospy.Timer(rospy.Duration(0.02), self._timer_cb)

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

    def _timer_cb(self, event):
        if self.rotating and self.goal is not None:
            self._check_rotation()

    def goal_cb(self, msg):
        rospy.loginfo(
            "pre_rotate: new goal frame=%s goal=(%.3f, %.3f) odom=(%.3f, %.3f, %.1fdeg)",
            msg.header.frame_id or "map",
            msg.pose.position.x,
            msg.pose.position.y,
            self.x,
            self.y,
            self.yaw * 180 / math.pi,
        )
        self._set_clear_paused(True, "new goal, wait for one-shot plan")
        self._cancel_move_base()
        self._stop_robot()
        rospy.sleep(self.stop_before_rotate)
        self._stop_robot()

        target_yaw = self._get_initial_path_yaw(msg)
        self._set_clear_paused(False, "one-shot plan finished")
        if target_yaw is None:
            rospy.logerr("pre_rotate: no valid global path heading, keep robot stopped")
            self._stop_robot()
            return

        err = self._norm(target_yaw - self.yaw)

        if abs(err) > self.alignment_tol:
            self.rotating = True
            self.goal = msg
            self.target_yaw = target_yaw
            rospy.loginfo(
                "pre_rotate: path heading err=%.1fdeg, start rotation",
                err * 180 / math.pi,
            )
        else:
            rospy.loginfo("pre_rotate: already aligned with path, send goal to move_base")
            self.goal_pub.publish(msg)

    def _check_rotation(self):
        if self.target_yaw is None:
            return

        err = self._norm(self.target_yaw - self.yaw)

        if abs(err) < self.alignment_tol:
            # 先停稳再转发 goal
            self._stop_robot()
            rospy.sleep(0.1)
            self._stop_robot()
            rospy.loginfo("pre_rotate: aligned, send goal to move_base")
            self.goal_pub.publish(self.goal)
            self.rotating = False
            self.goal = None
            self.target_yaw = None
        else:
            twist = Twist()
            twist.angular.z = self.max_angular if err > 0 else -self.max_angular
            self.cmd_pub.publish(twist)

    def _cancel_move_base(self):
        try:
            if self.move_client.wait_for_server(timeout=rospy.Duration(0.2)):
                self.move_client.cancel_all_goals()
                rospy.loginfo("pre_rotate: canceled current move_base goal")
            else:
                rospy.logwarn_throttle(2.0, "pre_rotate: move_base action server not ready")
        except Exception as e:
            rospy.logwarn_throttle(2.0, "pre_rotate: cancel move_base failed: %s", e)

    def _stop_robot(self):
        self.cmd_pub.publish(Twist())

    def _set_clear_paused(self, paused, reason):
        msg = Bool()
        msg.data = paused
        self.clear_pause_pub.publish(msg)
        rospy.loginfo("pre_rotate: clear_scheduler pause=%s (%s)", paused, reason)

    def _get_initial_path_yaw(self, goal):
        """先请求一次全局路径，用路径开头方向作为预旋转方向。"""
        deadline = rospy.Time.now().to_sec() + self.plan_retry_timeout
        attempt = 0
        while not rospy.is_shutdown() and rospy.Time.now().to_sec() < deadline:
            attempt += 1
            path = self._make_plan(goal)
            if path and path.poses:
                if not self._path_reaches_goal(path, goal):
                    self._log_plan_summary(path, goal, attempt, accepted=False)
                    rospy.logwarn_throttle(
                        1.0,
                        "pre_rotate: make_plan path endpoint is far from new goal, retry",
                    )
                    rospy.sleep(0.1)
                    continue
                self._log_plan_summary(path, goal, attempt, accepted=True)
                yaw = self._yaw_from_plan(path)
                if yaw is not None:
                    rospy.loginfo("pre_rotate: use global path heading %.1fdeg", yaw * 180 / math.pi)
                    return yaw
            rospy.sleep(0.1)

        rospy.logwarn("pre_rotate: make_plan did not provide a usable path heading")
        return None

    def _make_plan(self, goal):
        try:
            if self.make_plan is None:
                rospy.wait_for_service(self.make_plan_service, timeout=1.0)
                self.make_plan = rospy.ServiceProxy(self.make_plan_service, GetPlan)
                rospy.loginfo("pre_rotate: connected make_plan service %s", self.make_plan_service)

            start = PoseStamped()
            start.header.stamp = rospy.Time.now()
            start.header.frame_id = "map"
            start.pose.position.x = self.x
            start.pose.position.y = self.y
            start.pose.orientation.w = 1.0

            plan_goal = copy.deepcopy(goal)
            plan_goal.header.stamp = rospy.Time.now()
            if not plan_goal.header.frame_id:
                plan_goal.header.frame_id = "map"

            resp = self.make_plan(start, plan_goal, 0.0)
            if resp.plan.poses:
                return resp.plan
            rospy.logwarn("pre_rotate: make_plan returned empty path")
        except (rospy.ROSException, rospy.ServiceException) as e:
            self.make_plan = None
            rospy.logwarn_throttle(2.0, "pre_rotate: make_plan failed: %s", e)
        return None

    def _path_reaches_goal(self, path, goal):
        end = path.poses[-1].pose.position
        goal_pos = goal.pose.position
        dist = math.hypot(end.x - goal_pos.x, end.y - goal_pos.y)
        return dist <= self.plan_goal_tolerance

    def _log_plan_summary(self, path, goal, attempt, accepted):
        start = path.poses[0].pose.position
        end = path.poses[-1].pose.position
        goal_pos = goal.pose.position
        dist = math.hypot(end.x - goal_pos.x, end.y - goal_pos.y)
        rospy.logwarn(
            "pre_rotate: plan attempt=%d accepted=%s poses=%d "
            "start=(%.3f, %.3f) end=(%.3f, %.3f) goal=(%.3f, %.3f) end_goal_dist=%.3fm tol=%.3fm",
            attempt,
            accepted,
            len(path.poses),
            start.x,
            start.y,
            end.x,
            end.y,
            goal_pos.x,
            goal_pos.y,
            dist,
            self.plan_goal_tolerance,
        )

    def _yaw_from_plan(self, path):
        if len(path.poses) < 2:
            return None

        start = path.poses[0].pose.position
        last = start
        traveled = 0.0

        for pose in path.poses[1:]:
            current = pose.pose.position
            step = math.hypot(current.x - last.x, current.y - last.y)
            traveled += step
            if traveled >= self.plan_heading_dist:
                dx = current.x - start.x
                dy = current.y - start.y
                if math.hypot(dx, dy) > 1e-3:
                    return math.atan2(dy, dx)
            last = current

        for pose in path.poses[1:]:
            current = pose.pose.position
            dx = current.x - start.x
            dy = current.y - start.y
            if math.hypot(dx, dy) > 1e-3:
                return math.atan2(dy, dx)
        return None

    @staticmethod
    def _norm(a):
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        return a


if __name__ == "__main__":
    rospy.init_node("pre_rotate")
    PreRotate()
    rospy.spin()
