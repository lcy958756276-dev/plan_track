#!/usr/bin/env python3
import rospy
import math
import copy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetPlan
from tf.transformations import euler_from_quaternion


class PreRotate:
    def __init__(self):
        self.angle_threshold = math.radians(15.0)
        self.alignment_tol = math.radians(3.0)
        self.max_angular = 0.45                         # rad/s，慢一点更稳
        self.plan_heading_dist = 0.15                    # 沿全局路径取多远的点来决定初始朝向

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.rotating = False
        self.goal = None
        self.target_yaw = None
        self.make_plan = None

        self.goal_pub = rospy.Publisher("/goal_rotated", PoseStamped, queue_size=1)
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

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
        if self.rotating:
            self.goal = msg
            self.target_yaw = self._get_initial_path_yaw(msg)
            return

        target_yaw = self._get_initial_path_yaw(msg)
        if target_yaw is None:
            self.goal_pub.publish(msg)
            return

        err = self._norm(target_yaw - self.yaw)

        if abs(err) > self.angle_threshold:
            self.rotating = True
            self.goal = msg
            self.target_yaw = target_yaw
            rospy.loginfo(
                "pre_rotate: path heading err=%.1fdeg, start rotation",
                err * 180 / math.pi,
            )
        else:
            self.goal_pub.publish(msg)

    def _check_rotation(self):
        if self.target_yaw is None:
            return

        err = self._norm(self.target_yaw - self.yaw)

        if abs(err) < self.alignment_tol:
            # 先停稳再转发 goal
            self.cmd_pub.publish(Twist())
            rospy.sleep(0.1)
            self.cmd_pub.publish(Twist())
            rospy.loginfo("pre_rotate: aligned, send goal to move_base")
            self.goal_pub.publish(self.goal)
            self.rotating = False
            self.goal = None
            self.target_yaw = None
        else:
            twist = Twist()
            twist.angular.z = self.max_angular if err > 0 else -self.max_angular
            self.cmd_pub.publish(twist)

    def _get_initial_path_yaw(self, goal):
        """先请求一次全局路径，用路径开头方向作为预旋转方向。"""
        path = self._make_plan(goal)
        if path and path.poses:
            yaw = self._yaw_from_plan(path)
            if yaw is not None:
                rospy.loginfo("pre_rotate: use global path heading %.1fdeg", yaw * 180 / math.pi)
                return yaw

        yaw = self._yaw_to_goal(goal)
        if yaw is not None:
            rospy.logwarn("pre_rotate: make_plan unavailable, fallback to goal heading")
        return yaw

    def _make_plan(self, goal):
        try:
            if self.make_plan is None:
                rospy.wait_for_service("/move_base/make_plan", timeout=0.5)
                self.make_plan = rospy.ServiceProxy("/move_base/make_plan", GetPlan)

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

    def _yaw_from_plan(self, path):
        for pose in path.poses:
            dx = pose.pose.position.x - self.x
            dy = pose.pose.position.y - self.y
            if math.hypot(dx, dy) >= self.plan_heading_dist:
                return math.atan2(dy, dx)

        if len(path.poses) >= 2:
            p0 = path.poses[0].pose.position
            for pose in path.poses[1:]:
                p1 = pose.pose.position
                dx = p1.x - p0.x
                dy = p1.y - p0.y
                if math.hypot(dx, dy) > 1e-3:
                    return math.atan2(dy, dx)
        return None

    def _yaw_to_goal(self, goal):
        dx = goal.pose.position.x - self.x
        dy = goal.pose.position.y - self.y
        if math.hypot(dx, dy) < 1e-6:
            return None
        return math.atan2(dy, dx)

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
