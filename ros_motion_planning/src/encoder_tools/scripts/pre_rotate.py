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
        self.kp_rot = 1.5                              # 比例减速系数，越靠近目标转得越慢
        self.counter_steer = 0.3                      # 反向脉冲角速度 (rad/s)
        self.counter_duration = 0.15                  # 反向脉冲时长 (s)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.rotating = False
        self.rotate_dir = 0                           # +1=顺时针, -1=逆时针
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
            self.rotate_dir = 1 if err > 0 else -1    # 记录旋转方向：+1=顺时针, -1=逆时针
            rospy.loginfo("pre_rotate: err=%.1fdeg, start rotation (%s)",
                          err * 180 / math.pi,
                          "CW" if self.rotate_dir > 0 else "CCW")
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
            # 停稳
            self.cmd_pub.publish(Twist())
            rospy.sleep(0.15)

            # 反向脉冲：补偿慢减速侧轮子
            # 顺时针旋转后右轮响应快、左轮响应慢→逆时针脉冲帮左轮减速
            counter_twist = Twist()
            counter_twist.angular.z = -self.rotate_dir * self.counter_steer
            self.cmd_pub.publish(counter_twist)
            rospy.sleep(self.counter_duration)

            # 再停稳，然后转发 goal
            self.cmd_pub.publish(Twist())
            rospy.sleep(0.1)
            rospy.loginfo("pre_rotate: aligned, counter-steer done, send goal to move_base")
            self.goal_pub.publish(self.goal)
            self.rotating = False
            self.goal = None
        else:
            twist = Twist()
            raw = self.kp_rot * err
            twist.angular.z = max(-self.max_angular, min(self.max_angular, raw))
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
