#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


class PreRotate:
    def __init__(self):
        self.angle_threshold = math.radians(15.0)
        self.alignment_tol = math.radians(3.0)
        self.max_angular = 0.6                         # rad/s，慢一点更稳

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.rotating = False
        self.goal = None

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
            return

        dx = msg.pose.position.x - self.x
        dy = msg.pose.position.y - self.y
        if dx == 0 and dy == 0:
            self.goal_pub.publish(msg)
            return

        target_yaw = math.atan2(dy, dx)
        err = self._norm(target_yaw - self.yaw)

        if abs(err) > self.angle_threshold:
            self.rotating = True
            self.goal = msg
            rospy.loginfo("pre_rotate: err=%.1fdeg, start rotation", err * 180 / math.pi)
        else:
            self.goal_pub.publish(msg)

    def _check_rotation(self):
        dx = self.goal.pose.position.x - self.x
        dy = self.goal.pose.position.y - self.y
        if dx == 0 and dy == 0:
            return

        target_yaw = math.atan2(dy, dx)
        err = self._norm(target_yaw - self.yaw)

        if abs(err) < self.alignment_tol:
            # 先停稳再转发 goal
            self.cmd_pub.publish(Twist())
            rospy.sleep(0.1)
            self.cmd_pub.publish(Twist())
            rospy.loginfo("pre_rotate: aligned, send goal to move_base")
            self.goal_pub.publish(self.goal)
            self.rotating = False
            self.goal = None
        else:
            twist = Twist()
            # 越靠近目标角度越慢，比例降速
            ratio = min(1.0, abs(err) / self.angle_threshold)
            speed = max(0.05, self.max_angular * ratio)
            twist.angular.z = speed if err > 0 else -speed
            self.cmd_pub.publish(twist)

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
