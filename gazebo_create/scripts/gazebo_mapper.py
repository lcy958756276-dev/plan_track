#!/usr/bin/env python3
"""
gazebo_mapper.py
在 Gazebo 中通过 cmd_vel 控制小车移动，同时发布 odom 给 gmapping 建图。
原理：订阅 /cmd_vel → 积分出位置 → set_model_state 移动 Gazebo 模型 + 发布 /odom
"""
import sys
import argparse
import rospy
import math
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import SetModelState, GetModelState
from gazebo_msgs.msg import ModelState
import tf2_ros
import tf.transformations as tft

class GazeboMapper:
    def __init__(self, gazebo_ns, init_yaw_offset):
        rospy.init_node('gazebo_mapper', anonymous=True)

        self.model_name = rospy.get_param('~model_name', 'my_car')
        self.odom_frame = 'odom'
        self.base_frame = 'base_footprint'
        self.gazebo_ns = gazebo_ns
        self.init_yaw_offset = init_yaw_offset

        # 位置/角度
        self.x = 0.0
        self.y = 0.0
        self.th = init_yaw_offset

        # 等待服务
        rospy.wait_for_service(self.gazebo_ns + '/set_model_state')
        rospy.wait_for_service(self.gazebo_ns + '/get_model_state')
        self.set_state = rospy.ServiceProxy(self.gazebo_ns + '/set_model_state', SetModelState)
        self.get_state = rospy.ServiceProxy(self.gazebo_ns + '/get_model_state', GetModelState)

        self._sync_init_pose()

        # 订阅 cmd_vel
        self.sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_callback)

        # 发布 odom
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)

        # TF broadcaster
        self.tf_broad = tf2_ros.TransformBroadcaster()

        # 速度积分
        self.last_time = rospy.Time.now()
        self.vx = 0.0
        self.vth = 0.0

        self.timer = rospy.Timer(rospy.Duration(0.05), self.timer_callback)  # 20Hz

        rospy.loginfo(f"✅ gazebo_mapper 已启动 (ns={gazebo_ns}, yaw_offset={init_yaw_offset})")

    def _sync_init_pose(self):
        """获取模型在 Gazebo 中的初始位置，加上朝向偏移"""
        try:
            resp = self.get_state(self.model_name, 'world')
            if resp.success:
                self.x = resp.pose.position.x
                self.y = resp.pose.position.y
                _, _, self.th = tft.euler_from_quaternion([
                    resp.pose.orientation.x,
                    resp.pose.orientation.y,
                    resp.pose.orientation.z,
                    resp.pose.orientation.w
                ])
                self.th += self.init_yaw_offset
                rospy.loginfo(f"初始位置: x={self.x:.3f}, y={self.y:.3f}, th={self.th:.3f}")
        except Exception as e:
            rospy.logwarn(f"获取初始位置失败: {e}")

    def cmd_callback(self, msg):
        self.vx = msg.linear.x
        self.vth = msg.angular.z

    def timer_callback(self, event):
        now = rospy.Time.now()
        dt = (now - self.last_time).to_sec()
        self.last_time = now

        if dt <= 0:
            return

        # 积分位置
        self.x += self.vx * math.cos(self.th) * dt
        self.y += self.vx * math.sin(self.th) * dt
        self.th += self.vth * dt

        # 设置 Gazebo 模型位置
        q = tft.quaternion_from_euler(0, 0, self.th)
        model_state = ModelState()
        model_state.model_name = self.model_name
        model_state.pose.position.x = self.x
        model_state.pose.position.y = self.y
        model_state.pose.position.z = 0.1105
        model_state.pose.orientation.x = q[0]
        model_state.pose.orientation.y = q[1]
        model_state.pose.orientation.z = q[2]
        model_state.pose.orientation.w = q[3]
        model_state.reference_frame = 'world'

        try:
            self.set_state(model_state)
        except Exception as e:
            rospy.logwarn_throttle(5.0, f"set_model_state 失败: {e}")

        # 发布 odom
        odom = Odometry()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.header.stamp = now
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.angular.z = self.vth
        self.odom_pub.publish(odom)

        # 发布 TF odom → base_footprint
        tf = TransformStamped()
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id = self.base_frame
        tf.header.stamp = now
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.translation.z = 0
        tf.transform.rotation.x = q[0]
        tf.transform.rotation.y = q[1]
        tf.transform.rotation.z = q[2]
        tf.transform.rotation.w = q[3]
        self.tf_broad.sendTransform(tf)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--gazebo-namespace', default='/gazebo')
    parser.add_argument('--init-yaw-offset', type=float, default=0.0)
    # 过滤掉 ROS 参数（以 __ 或 _ 开头）
    args, unknown = parser.parse_known_args()
    # 把剩余的参数传给 rospy
    if unknown:
        sys.argv[1:] = unknown
    try:
        GazeboMapper(args.gazebo_namespace, args.init_yaw_offset)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
