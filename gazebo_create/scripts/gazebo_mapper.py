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
    def __init__(self, gazebo_ns, init_yaw_offset, gazebo_yaw_offset):
        rospy.init_node('gazebo_mapper', anonymous=True)
        log(f"等待仿真时钟同步...")
        # ROS1 Noetic 用轮询等待时钟，没有 waitForSync()
        while rospy.Time.now().to_sec() == 0 and not rospy.is_shutdown():
            rospy.sleep(0.1)
        log(f"仿真时钟已同步 (t={rospy.Time.now().to_sec()}s)")

        self.model_name = rospy.get_param('~model_name', 'my_car')
        self.odom_frame = 'odom'
        self.base_frame = 'base_footprint'
        self.gazebo_ns = gazebo_ns
        self.init_yaw_offset = init_yaw_offset
        self.gazebo_yaw_offset = gazebo_yaw_offset

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

        # 立即启动定时器，快速覆盖 spawn 默认朝向
        self.timer = rospy.Timer(rospy.Duration(0.05), self.timer_callback)

        log(f"✅ 启动完成 (ns={gazebo_ns}, yaw_offset={init_yaw_offset})")

    def _sync_init_pose(self):
        """读 x,y 位置，朝向直接用 init_yaw_offset（不读 Gazebo 模型朝向，已被 -Y 预旋转）"""
        try:
            resp = self.get_state(self.model_name, 'world')
            if resp.success:
                self.x = resp.pose.position.x
                self.y = resp.pose.position.y
                self.th = self.init_yaw_offset  # 不读 Gazebo 朝向，用 init_yaw_offset
                log(f"初始位置: x={self.x:.3f}, y={self.y:.3f}, th={self.th:.3f} (init_yaw_offset)")
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

        # base_link 相对于 base_footprint 的偏移（base_footprint_joint xyz 的 x 值）
        # 转向时保持 base_link 不动，让 base_footprint 绕它转
        OX, OY = 0.254, 0.0

        # 1. 记录转向前的 base_link 世界位置
        old_th = self.th
        bl_x = self.x + OX * math.cos(old_th) - OY * math.sin(old_th)
        bl_y = self.y + OX * math.sin(old_th) + OY * math.cos(old_th)

        # 2. 更新朝向
        self.th += self.vth * dt

        # 3. 保持 base_link 不动，调整 base_footprint
        self.x = bl_x - (OX * math.cos(self.th) - OY * math.sin(self.th))
        self.y = bl_y - (OX * math.sin(self.th) + OY * math.cos(self.th))

        # 4. 前进运动（整体平移）
        move_th = self.th + self.gazebo_yaw_offset
        self.x += self.vx * math.cos(move_th) * dt
        self.y += self.vx * math.sin(move_th) * dt

        q = tft.quaternion_from_euler(0, 0, self.th)

        # Gazebo 朝向（加 gazebo_yaw_offset 补偿 Gazebo 与 RViz 的显示差异）
        q_gz = tft.quaternion_from_euler(0, 0, move_th)
        model_state = ModelState()
        model_state.model_name = self.model_name
        model_state.pose.position.x = self.x
        model_state.pose.position.y = self.y
        model_state.pose.position.z = 0.0
        model_state.pose.orientation.x = q_gz[0]
        model_state.pose.orientation.y = q_gz[1]
        model_state.pose.orientation.z = q_gz[2]
        model_state.pose.orientation.w = q_gz[3]
        model_state.reference_frame = 'world'
        try:
            self.set_state(model_state)
            # 每 100 帧打一次日志，确认朝向
            if not hasattr(self, '_log_count'):
                self._log_count = 0
            self._log_count += 1
            if self._log_count % 100 == 0:
                import math as m
                roll, pitch, yaw = tft.euler_from_quaternion([q[0], q[1], q[2], q[3]])
                roll_gz, pitch_gz, yaw_gz = tft.euler_from_quaternion([q_gz[0], q_gz[1], q_gz[2], q_gz[3]])
                log(f"位置: x={self.x:.3f}, y={self.y:.3f}, TF heading={yaw*180/m.pi:.1f}°, Gazebo heading={yaw_gz*180/m.pi:.1f}°")
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
    parser.add_argument('--gazebo-yaw-offset', type=float, default=0.0,
                        help='仅对 Gazebo model_state 生效的朝向偏移，用于补偿 Gazebo 与 RViz 的显示差异')
    args, unknown = parser.parse_known_args()

    log(f"参数: ns={args.gazebo_namespace}, init_yaw_offset={args.init_yaw_offset}, gazebo_yaw_offset={args.gazebo_yaw_offset}")
    log(f"未解析参数: {unknown}")

    try:
        GazeboMapper(args.gazebo_namespace, args.init_yaw_offset, args.gazebo_yaw_offset)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        log(f"致命错误: {e}")
        raise
