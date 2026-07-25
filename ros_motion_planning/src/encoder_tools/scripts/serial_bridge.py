#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
serial_bridge.py
合并 read_uart + send_mpc_speed 到同一个串口节点。

功能:
  1) 订阅 /cmd_vel → 差速模型 → 串口发送 "l:左轮,r:右轮\r\n"
  2) 读取串口 → 解析 MCU 返回的 ticks → 发布 /wheel_ticks
  3) 过滤掉命令回显，只保留 tick 数据

MCU 预期输出格式 (可配置):
  每行: "ltick:123 rtick:456"
  或逗号分隔: "v_left,v_right,encoder_count"

用法 (由 run_debug.sh 自动启动):
    rosrun encoder_tools serial_bridge.py
"""

import re
import threading
import rospy
import serial
import termios
from std_msgs.msg import Int64MultiArray
from geometry_msgs.msg import Twist


class SerialBridge:
    def __init__(self):
        port = rospy.get_param("~port", "/dev/ttyTHS0")
        baud = rospy.get_param("~baud", 57600)

        # ── 物理参数（差速模型用） ──
        self.wheel_radius = rospy.get_param("~wheel_radius", 0.1065)
        self.wheel_base   = rospy.get_param("~wheel_base", 0.45)
        self.cmd_timeout = rospy.get_param("~cmd_timeout", 0.3)
        self.zero_repeat_period = rospy.get_param("~zero_repeat_period", 0.2)

        # ── 打开串口（只开一次） ──
        self.ser = serial.Serial(port=port, baudrate=baud, timeout=0.1)
        self.write_lock = threading.Lock()

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

        self.ser.reset_input_buffer()
        rospy.loginfo(f"[bridge] 串口已打开: {port} @ {baud} (raw mode)")

        # ── Tick 发布 ──
        self.tick_pub = rospy.Publisher("/wheel_ticks", Int64MultiArray, queue_size=10)

        # ── 订阅 cmd_vel ──
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_cb, queue_size=1)

        # ── 解析模式 ──
        # 模式1: ltick:123 rtick:456（read_uart 原格式）
        self.pattern_ltick_rtick = re.compile(r'ltick:\s*(-?\d+)\s+rtick:\s*(-?\d+)')
        # 模式2: 逗号分隔三值 (v_left,v_right,encoder_count) — 根据 MCU 实际输出调整
        self.pattern_csv3 = re.compile(r'(-?\d+\.?\d*)\s*,\s*(-?\d+\.?\d*)\s*,\s*(-?\d+\.?\d*)')
        # 回显过滤: 匹配 "target_l:..." 或 "l:...,r:..." 等命令回显
        self.pattern_echo = re.compile(r'target_[lr]|^l:-?\d|^V\s')

        # ── 统计 ──
        self.tick_count = 0
        self.cmd_count = 0
        self.last_cmd_time = rospy.Time.now()
        self.last_zero_send_time = rospy.Time(0)
        self.last_sent_zero = True
        rospy.Timer(rospy.Duration(0.1), self.cmd_timeout_cb)

        # ── 斜率检查（防 MCU 数据乱码导致位置跳变） ──
        self.last_ltick = None
        self.last_rtick = None
        self.delta_hist_ltick = []
        self.delta_hist_rtick = []

        rospy.loginfo("[bridge] serial_bridge 已启动 (读+写合并)")
        rospy.loginfo(f"[bridge] wheel_base={self.wheel_base:.4f}, wheel_radius={self.wheel_radius:.4f}")

    # ── 写入: /cmd_vel → 左右轮速度 → 串口 ──
    # 只在单轮速度接近 0 时补一点死区速度，避免静摩擦；不压缩上层给出的角速度。
    def cmd_vel_cb(self, msg):
        self.cmd_count += 1
        self.last_cmd_time = rospy.Time.now()
        v = msg.linear.x
        w = msg.angular.z

        half_base = self.wheel_base / 2.0
        v_left  = v - w * half_base
        v_right = v + w * half_base

        # 防单轮停转: 轮速接近 0 时补到 min_speed，不改变上层的转向意图。
        min_speed = rospy.get_param("~min_wheel_speed", 0.01)
        if abs(v) > min_speed or abs(w) > 1e-3:
            original_left = v_left
            original_right = v_right

            if 0.0 <= abs(v_left) < min_speed:
                v_left = min_speed * self._wheel_deadband_sign(original_left, -w)
            if 0.0 <= abs(v_right) < min_speed:
                v_right = min_speed * self._wheel_deadband_sign(original_right, w)

            if v_left != original_left or v_right != original_right:
                rospy.loginfo_throttle(1.0,
                    f"[bridge] 单轮死区补偿: l {original_left:.3f}->{v_left:.3f}, "
                    f"r {original_right:.3f}->{v_right:.3f}, 保留输入 v={v:.3f} ω={w:.3f}")

        cmd_str = self._write_wheel_command(v_left, v_right)
        self.last_sent_zero = abs(v) < 1e-4 and abs(w) < 1e-4
        if self.last_sent_zero:
            self.last_zero_send_time = self.last_cmd_time

        rospy.loginfo_throttle(1.0,
            f"[bridge] cmd #{self.cmd_count}: v={v:.3f} ω={w:.3f} → "
            f"l={v_left:.3f} r={v_right:.3f}  [串口发送] {cmd_str.strip()}")

    def cmd_timeout_cb(self, event):
        now = rospy.Time.now()
        if (now - self.last_cmd_time).to_sec() <= self.cmd_timeout:
            return
        if self.last_sent_zero and (now - self.last_zero_send_time).to_sec() < self.zero_repeat_period:
            return

        cmd_str = self._write_wheel_command(0.0, 0.0)
        self.last_sent_zero = True
        self.last_zero_send_time = now
        rospy.loginfo_throttle(
            1.0,
            f"[bridge] cmd_vel 超时 {self.cmd_timeout:.2f}s，主动发送零速度  [串口发送] {cmd_str.strip()}",
        )

    def _write_wheel_command(self, v_left, v_right):
        cmd_str = f"l:{v_left:.3f},r:{v_right:.3f}\r\n"
        try:
            with self.write_lock:
                if self.ser and self.ser.is_open:
                    self.ser.write(cmd_str.encode("utf-8"))
        except serial.SerialException as e:
            rospy.logerr_throttle(3.0, f"[bridge] 串口写入失败: {e}")
        return cmd_str

    def _wheel_deadband_sign(self, speed, turn_preference):
        if speed > 0:
            return 1.0
        if speed < 0:
            return -1.0
        if turn_preference > 0:
            return 1.0
        if turn_preference < 0:
            return -1.0
        return 1.0

    # ── 读取: 串口 → tick 解析 → /wheel_ticks ──
    def run(self):
        buf = b''
        while not rospy.is_shutdown():
            try:
                c = self.ser.read(1)
                if not c:
                    continue
                if c == b'\n':
                    line = buf.decode('utf-8', errors='ignore').strip()
                    buf = b''

                    if not line:
                        continue

                    # 过滤回显
                    if self.pattern_echo.search(line):
                        continue

                    # 尝试匹配 ltick:123 rtick:456
                    m = self.pattern_ltick_rtick.search(line)
                    if m:
                        ltick = int(m.group(1))
                        rtick = int(m.group(2))
                        self._publish_tick(ltick, rtick, line)
                        continue

                    # 尝试匹配逗号分隔三值 — 如果是 tick 数据则发布
                    m = self.pattern_csv3.match(line)
                    if m:
                        # 第三列可能是累积编码器值，需要根据实际 MCU 协议调整
                        # 目前先跳过，需要用户确认 MCU 输出格式
                        rospy.loginfo_throttle(5.0, f"[bridge] 收到 MCU 数据(未解析): {line}")
                        continue

                    # 未知格式
                    rospy.logwarn_throttle(5.0, f"[bridge] 无法匹配格式，丢弃: {line}")
                else:
                    buf += c

            except serial.SerialException as e:
                rospy.logerr(f"[bridge] 串口读取错误: {e}")
                break
            except Exception as e:
                rospy.logwarn(f"[bridge] 异常: {e}")

    def _check_sanity(self, ltick, rtick):
        """自适应斜率阈值：检查每帧变化量是否合理，防乱码跳变"""
        if self.last_ltick is not None:
            delta = ltick - self.last_ltick
            if not self._delta_ok(delta, self.delta_hist_ltick, "ltick"):
                return False

        if self.last_rtick is not None:
            delta = rtick - self.last_rtick
            if not self._delta_ok(delta, self.delta_hist_rtick, "rtick"):
                return False

        return True

    def _delta_ok(self, delta, history, name):
        """自适应检查 delta 是否合理"""
        if len(history) >= 3:
            avg = sum(history) / len(history)
            max_allowed = max(abs(avg) * 8, 30000)
            if abs(delta) > max_allowed:
                rospy.logwarn(f"[bridge] {name} 斜率异常: {delta} (平均 delta={avg:.0f}, 阈值={max_allowed:.0f})")
                return False
        else:
            if abs(delta) > 100000:
                rospy.logwarn(f"[bridge] {name} 首帧斜率异常: {delta}")
                return False

        history.append(delta)
        if len(history) > 10:
            history.pop(0)
        return True

    def _publish_tick(self, ltick, rtick, raw_line):
        self.tick_count += 1

        # 斜率检查：过滤异常跳变
        if not self._check_sanity(ltick, rtick):
            return

        # 更新 last 值
        self.last_ltick = ltick
        self.last_rtick = rtick

        # 发布
        msg = Int64MultiArray()
        msg.data = [ltick, rtick]
        self.tick_pub.publish(msg)
        rospy.loginfo_throttle(1.0, f"[bridge] tick #{self.tick_count}: ltick={ltick}, rtick={rtick}")

    def shutdown(self):
        if self.ser and self.ser.is_open:
            with self.write_lock:
                self.ser.write(b"l:0.000,r:0.000\r\n")
            self.ser.close()
        rospy.loginfo("[bridge] 串口已关闭")


if __name__ == "__main__":
    rospy.init_node("serial_bridge")
    node = SerialBridge()
    rospy.on_shutdown(node.shutdown)
    node.run()
