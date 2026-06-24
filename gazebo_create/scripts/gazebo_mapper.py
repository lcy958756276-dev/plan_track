#!/usr/bin/env python3 -u
"""
gazebo_mapper.py
在 Gazebo 中通过 cmd_vel 控制小车移动，同时发布 odom 给 gmapping 建图。
"""
import sys
import os
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

def log(msg):
    """强制刷新的日志输出"""
    print(f"[gazebo_mapper] {msg}", flush=True)

class GazeboMapper:
    def __init__(self, gazebo_ns, init_yaw_offset):
        rospy.init_node('gazebo_mapper', anonymous=True)
        log(f"节点初始化完成")

        self.model_name = rospy.get_param('~model_name', 'my_car')
        self.odom_frame = 'odom'
        self.base_frame = 'base_footprint'
        self.gazebo_ns = gazebo_ns
        self.init_yaw_offset = init_yaw_offset

        self.x = 0.0
        self.y = 0.0
        self.th = init_yaw_offset

        log(f"等待 Gazebo 服务: {gazebo_ns}/set_model_state")
        rospy.wait_for_service(gazebo_ns + '/set_model_state', timeout=10)
        rospy.wait_for_service(gazebo_ns + '/get_model_state', timeout=10)
        self.set_state = rospy.ServiceProxy(gazebo_ns + '/set_model_state', SetModelState)
        self.get_state = rospy.ServiceProxy(gazebo_ns + '/get_model_state', GetModelState)
        log(f"Gazebo 服务已连接")

        self._sync_init_pose()

        self.sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_callback)
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.tf_broad = tf2_ros.TransformBroadcaster()

        self.last_time = rospy.Time.now()
        self.vx = 0.0
        self.vth = 0.0

        # 延迟一下再启动定时器，避免刚启动就和 spawn 竞争
        rospy.sleep(1.0)
        self.timer = rospy.Timer(rospy.Duration(0.05), self.timer_callback)

        log(f"✅ 启动完成 (ns={gazebo_ns}, yaw_offset={init_yaw_offset})")

    def _sync_init_pose(self):
        try:
            resp = self.get_state(self.model_name, 'world')
            if resp.success:
                self.x = resp.pose.position.x
                self.y = resp.pose.position.y
                _, _, self.th = tft.euler_from_quaternion([
                    resp.pose.orientation.x, resp.pose.orientation.y,
                    resp.pose.orientation.z, resp.pose.orientation.w
                ])
                self.th += self.init_yaw_offset
                log(f"初始位置: x={self.x:.3f}, y={self.y:.3f}, th={self.th:.3f}")
        except Exception as e:
            log(f"获取初始位置失败: {e}，使用默认 th={self.th:.3f}")

    def cmd_callback(self, msg):
        self.vx = msg.linear.x
        self.vth = msg.angular.z

    def timer_callback(self, event):
        now = rospy.Time.now()
        dt = (now - self.last_time).to_sec()
        self.last_time = now
        if dt <= 0:
            return

        self.x += self.vx * math.cos(self.th) * dt
        self.y += self.vx * math.sin(self.th) * dt
        self.th += self.vth * dt

        q = tft.quaternion_from_euler(0, 0, self.th)

        # 设置 Gazebo 模型位置
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
            log(f"set_model_state 失败: {e}")

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

        # 发布 TF
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
    args, unknown = parser.parse_known_args()

    # 打印收到的参数
    log(f"参数: gazebo_namespace={args.gazebo_namespace}, init_yaw_offset={args.init_yaw_offset}")
    log(f"未解析参数: {unknown}")

    try:
        GazeboMapper(args.gazebo_namespace, args.init_yaw_offset)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        log(f"致命错误: {e}")
        raise
