#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
send_mpc_speed.py
订阅 /cmd_vel → 差速模型换算左右轮速度 → 串口发送给电机驱动板

差速模型:
    v_left  = v - ω * wheel_base / 2
    v_right = v + ω * wheel_base / 2

串口协议 (文本格式, 方便 MCU 解析):
    V <left_mps> <right_mps>\r\n
    例如: V 0.15 0.12\r\n

注意: 波特率必须与 read_uart.py 一致（默认 57600），
      否则会冲掉 read_uart 的串口配置。

用法 (由 run_debug.sh 自动启动):
    rosrun encoder_tools send_mpc_speed.py

独立测试:
    rosrun encoder_tools send_mpc_speed.py _port:=/dev/ttyTHS0 _baud:=57600
"""

import rospy
import serial
import termios
from geometry_msgs.msg import Twist


class MpcSpeedSender:
    def __init__(self):
        # ── 物理参数（与 encoder_odom.py 保持一致） ──
        self.wheel_radius = rospy.get_param("~wheel_radius", 0.1065)     # m
        self.wheel_base   = rospy.get_param("~wheel_base", 0.245 * 2.05) # m ≈ 0.50225

        # ── 串口配置 ──
        port = rospy.get_param("~port", "/dev/ttyTHS0")
        baud = rospy.get_param("~baud", 57600)
        self.serial_enabled = False

        try:
            self.ser = serial.Serial(port=port, baudrate=baud, timeout=0.1)
            fd = self.ser.fileno()
            tty = termios.tcgetattr(fd)
            tty[0] = 0
            tty[1] = 0
            tty[2] = tty[2] & ~termios.PARENB
            tty[2] = tty[2] & ~termios.CSTOPB
            tty[2] = tty[2] & ~termios.CSIZE
            tty[2] = tty[2] | termios.CS8
            tty[2] = tty[2] & ~termios.CRTSCTS
            tty[2] = tty[2] | termios.CREAD
            tty[2] = tty[2] & ~termios.HUPCL
            tty[3] = 0
            termios.tcsetattr(fd, termios.TCSANOW, tty)
            self.serial_enabled = True
            rospy.loginfo(f"[send_mpc] 串口已打开: {port} @ {baud} (raw mode)")
        except Exception as e:
            rospy.logwarn(f"[send_mpc] 串口打开失败 (仅打印日志，不发送): {e}")
            rospy.logwarn("[send_mpc] 检查串口是否存在或权限是否正确")
            self.ser = None

        # ── 订阅 cmd_vel ──
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_cb, queue_size=1)

        rospy.loginfo("=" * 55)
        rospy.loginfo("  send_mpc_speed 已启动")
        rospy.loginfo(f"  wheel_radius = {self.wheel_radius:.4f} m")
        rospy.loginfo(f"  wheel_base   = {self.wheel_base:.4f} m")
        rospy.loginfo(f"  差速模型: v_l = v - ω·B/2,  v_r = v + ω·B/2")
        rospy.loginfo(f"  串口协议: V <left_mps> <right_mps>\\r\\n")
        rospy.loginfo(f"  {'串口已连接, 数据将发送到物理车' if self.serial_enabled else '串口未连接, 仅打印日志'}")
        rospy.loginfo("=" * 55)

        # 计数器
        self.msg_count = 0

    def cmd_vel_cb(self, msg):
        self.msg_count += 1
        v = msg.linear.x
        w = msg.angular.z

        # ── 差速模型: v, ω → 左右轮线速度 ──
        half_base = self.wheel_base / 2.0
        v_left  = v - w * half_base
        v_right = v + w * half_base

        # ── 每秒打印一次明细 ──
        rospy.loginfo_throttle(
            1.0,
            f"\n"
            f"  ┌── cmd_vel #{self.msg_count} ──────────────────┐\n"
            f"  │  v (线速度)        = {v:>8.3f}  m/s       │\n"
            f"  │  ω (角速度)        = {w:>8.3f}  rad/s     │\n"
            f"  │                                          │\n"
            f"  │  v_left  (左轮)    = {v_left:>8.3f}  m/s       │\n"
            f"  │  v_right (右轮)    = {v_right:>8.3f}  m/s       │\n"
            f"  └──────────────────────────────────────────┘"
        )

        # ── 串口发送 ──
        if self.ser is not None and self.ser.is_open:
            # 文本协议:  V <left_mps> <right_mps>\r\n
            # 例如 MPU 收到 "V 0.15 -0.02\r\n" 后用 sscanf 即可解析
            cmd_str = f"V {v_left:.3f} {v_right:.3f}\r\n"
            try:
                self.ser.write(cmd_str.encode("utf-8"))
                rospy.loginfo_throttle(1.0, f"  [串口发送] {cmd_str.strip()}")
            except serial.SerialException as e:
                rospy.logerr_throttle(3.0, f"[send_mpc] 串口写入失败: {e}")

    def shutdown(self):
        if self.ser is not None and self.ser.is_open:
            # 停止时发送零速度
            try:
                self.ser.write(b"V 0.000 0.000\r\n")
                rospy.loginfo("[send_mpc] 发送零速度，关闭串口")
            except Exception:
                pass
            self.ser.close()


if __name__ == "__main__":
    rospy.init_node("send_mpc_speed")
    node = MpcSpeedSender()
    rospy.on_shutdown(node.shutdown)
    rospy.spin()
