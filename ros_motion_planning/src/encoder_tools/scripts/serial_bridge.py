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
import os
import time
import rospy
import serial
import termios
from std_msgs.msg import Int64MultiArray
from geometry_msgs.msg import Twist, PoseStamped


class SerialBridge:
    def __init__(self):
        port = rospy.get_param("~port", "/dev/ttyTHS0")
        baud = rospy.get_param("~baud", 57600)

        # ── 物理参数（差速模型用） ──
        self.wheel_radius = rospy.get_param("~wheel_radius", 0.1065)
        self.wheel_base   = rospy.get_param("~wheel_base", 0.45)
        self.cmd_timeout = rospy.get_param("~cmd_timeout", 0.3)
        self.zero_repeat_period = rospy.get_param("~zero_repeat_period", 0.2)
        self.stop_burst_count = rospy.get_param("~stop_burst_count", 3)
        self.stop_burst_gap = rospy.get_param("~stop_burst_gap", 0.02)
        self.stop_brake_enabled = rospy.get_param("~stop_brake_enabled", False)
        self.stop_brake_right_speed = rospy.get_param("~stop_brake_right_speed", -0.08)
        self.stop_brake_duration = rospy.get_param("~stop_brake_duration", 0.4)
        self.pre_stop_brake_enabled = rospy.get_param("~pre_stop_brake_enabled", True)
        self.pre_stop_linear_threshold = rospy.get_param("~pre_stop_linear_threshold", 0.045)
        self.pre_stop_angular_threshold = rospy.get_param("~pre_stop_angular_threshold", 0.25)
        self.pre_stop_right_threshold = rospy.get_param("~pre_stop_right_threshold", 0.09)
        self.pre_stop_brake_cooldown = rospy.get_param("~pre_stop_brake_cooldown", 2.0)
        self.terminal_stop_hold_duration = rospy.get_param("~terminal_stop_hold_duration", 1.2)
        self.terminal_decel_log_linear_threshold = rospy.get_param(
            "~terminal_decel_log_linear_threshold", 0.13
        )

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
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_cb, queue_size=1)
        rospy.Subscriber("/goal_rotated", PoseStamped, self.goal_cb, queue_size=1)

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
        self.last_pre_stop_brake_time = rospy.Time(0)
        self.terminal_stop_hold_until = rospy.Time(0)
        self.terminal_stop_latched = False
        self.terminal_decel_logged = False
        self.goal_count = 0
        rospy.Timer(rospy.Duration(0.1), self.cmd_timeout_cb)

        # ── 斜率检查（防 MCU 数据乱码导致位置跳变） ──
        self.last_ltick = None
        self.last_rtick = None
        self.delta_hist_ltick = []
        self.delta_hist_rtick = []
        self.last_stop_warn_time = rospy.Time(0)

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

        is_zero_cmd = abs(v) < 1e-4 and abs(w) < 1e-4

        if self._should_log_terminal_decel_send(v, is_zero_cmd):
            self.terminal_decel_logged = True
            rospy.loginfo(
                f"[bridge] TERMINAL_DECEL_SEND_START: stamp={self.last_cmd_time.to_sec():.3f} "
                f"cmd_v={v:.3f} cmd_w={w:.3f} wheel_l={v_left:.3f} wheel_r={v_right:.3f}"
            )

        if self.terminal_stop_latched:
            cmd_str = self._write_wheel_command(0.0, 0.0)
            self.last_sent_zero = True
            self.last_zero_send_time = self.last_cmd_time
            rospy.loginfo_throttle(0.5,
                f"[bridge] 终点停车锁存中，只发送零速度，拦截上层命令 v={v:.3f} ω={w:.3f}  "
                f"[串口发送] {cmd_str.strip()}")
            return

        if self._in_terminal_stop_hold():
            cmd_str = self._write_wheel_command(0.0, 0.0)
            self.last_sent_zero = True
            self.last_zero_send_time = self.last_cmd_time
            rospy.loginfo_throttle(0.5,
                f"[bridge] 终点停车保持中，拦截上层残余命令 v={v:.3f} ω={w:.3f}  "
                f"[串口发送] {cmd_str.strip()}")
            return

        should_brake = is_zero_cmd and not self.last_sent_zero
        should_pre_brake = self._should_pre_stop_brake(v, w, v_right, is_zero_cmd)
        if should_brake:
            self.terminal_stop_latched = True
            self.terminal_stop_hold_until = (
                self.last_cmd_time + rospy.Duration(self.terminal_stop_hold_duration)
            )

        if should_pre_brake:
            self.terminal_stop_hold_until = (
                self.last_cmd_time + rospy.Duration(self.terminal_stop_hold_duration)
            )
            self.terminal_stop_latched = True
            self.last_pre_stop_brake_time = self.last_cmd_time
            cmd_str = self._write_wheel_command(0.0, 0.0)
            self.last_sent_zero = True
            self.last_zero_send_time = self.last_cmd_time
            rospy.loginfo(
                f"[bridge] 终点低速段进入停车保持 stamp={self.last_cmd_time.to_sec():.3f} "
                f"hold={self.terminal_stop_hold_duration:.2f}s，"
                f"拦截原命令 v={v:.3f} ω={w:.3f} → l={v_left:.3f} r={v_right:.3f}  "
                f"[串口发送] {cmd_str.strip()}"
            )
            return
        cmd_str = self._write_wheel_command(v_left, v_right, brake_right=False)
        self.last_sent_zero = is_zero_cmd
        if self.last_sent_zero:
            self.last_zero_send_time = self.last_cmd_time

        rospy.loginfo_throttle(1.0,
            f"[bridge] cmd #{self.cmd_count}: v={v:.3f} ω={w:.3f} → "
            f"l={v_left:.3f} r={v_right:.3f}  [串口发送] {cmd_str.strip()}")

    def goal_cb(self, msg):
        self.goal_count += 1
        if self.terminal_stop_latched:
            rospy.loginfo(
                f"[bridge] 收到新 goal #{self.goal_count}，解除终点停车锁存，允许重新接收速度命令"
            )
        else:
            rospy.loginfo_throttle(
                1.0,
                f"[bridge] 收到新 goal #{self.goal_count}，当前未锁存"
            )
        self.terminal_stop_latched = False
        self.terminal_stop_hold_until = rospy.Time(0)
        self.terminal_decel_logged = False
        self.last_sent_zero = True
        self.last_zero_send_time = rospy.Time.now()

    def _should_log_terminal_decel_send(self, v, is_zero_cmd):
        if self.terminal_decel_logged or is_zero_cmd:
            return False
        return 0.0 < abs(v) <= self.terminal_decel_log_linear_threshold

    def _should_pre_stop_brake(self, v, w, v_right, is_zero_cmd):
        if not self.pre_stop_brake_enabled or is_zero_cmd:
            return False
        if self.last_sent_zero:
            return False
        if v <= 0.0 or v > self.pre_stop_linear_threshold:
            return False
        if abs(w) > self.pre_stop_angular_threshold:
            return False
        if not (0.0 <= v_right <= self.pre_stop_right_threshold):
            return False
        now = rospy.Time.now()
        return (now - self.last_pre_stop_brake_time).to_sec() >= self.pre_stop_brake_cooldown

    def _in_terminal_stop_hold(self):
        return rospy.Time.now() < self.terminal_stop_hold_until

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

    def _write_wheel_command(self, v_left, v_right, brake_right=False,
                             brake_reason="停车前右轮反向刹车"):
        cmd_str = f"l:{v_left:.3f},r:{v_right:.3f}\r\n"
        cmd_bytes = cmd_str.encode("utf-8")
        try:
            with self.write_lock:
                if self.ser and self.ser.is_open:
                    if abs(v_left) < 1e-4 and abs(v_right) < 1e-4:
                        if self.stop_brake_enabled and brake_right:
                            self._send_right_brake(brake_reason)

                        stop_packets = [
                            b"l:0.000,r:0.000\r\n",
                            b"l:0,r:0\r\n",
                            b"r:0.000,l:0.000\r\n",
                            b"V 0.000 0.000\r\n",
                        ]
                        for _ in range(max(1, self.stop_burst_count)):
                            for packet in stop_packets:
                                self._write_serial_bytes(packet)
                            time.sleep(self.stop_burst_gap)
                        rospy.loginfo_throttle(
                            1.0,
                            "[bridge] 零速度兼容发送字节: "
                            + ", ".join(repr(packet) for packet in stop_packets)
                        )
                    else:
                        self._write_serial_bytes(cmd_bytes)
        except serial.SerialException as e:
            rospy.logerr_throttle(3.0, f"[bridge] 串口写入失败: {e}")
        except OSError as e:
            rospy.logerr_throttle(3.0, f"[bridge] 串口底层写入失败: {e}")
        return cmd_str

    def _write_serial_bytes(self, data):
        fd = self.ser.fileno()
        os.write(fd, data)
        termios.tcdrain(fd)

    def _send_right_brake(self, reason):
        if not (self.ser and self.ser.is_open):
            return
        brake_packet = (
            f"l:0.000,r:{self.stop_brake_right_speed:.3f}\r\n"
        ).encode("utf-8")
        self._write_serial_bytes(brake_packet)
        rospy.loginfo(
            f"[bridge] {reason}: {brake_packet!r}, "
            f"duration={self.stop_brake_duration:.2f}s"
        )
        time.sleep(max(0.0, self.stop_brake_duration))

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
        prev_ltick = self.last_ltick
        prev_rtick = self.last_rtick

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

        if self.last_sent_zero:
            now = rospy.Time.now()
            if (now - self.last_cmd_time).to_sec() > 0.5 and (now - self.last_stop_warn_time).to_sec() > 1.0:
                delta_l = 0 if prev_ltick is None else ltick - prev_ltick
                delta_r = 0 if prev_rtick is None else rtick - prev_rtick
                rospy.logwarn(
                    f"[bridge] 已持续发送零速度，但编码器仍在变化: "
                    f"Δl={delta_l}, Δr={delta_r}，"
                    f"若右轮仍转，多半是底层右轮停机执行/驱动问题"
                )
                self.last_stop_warn_time = now

    def shutdown(self):
        if self.ser and self.ser.is_open:
            self._write_wheel_command(0.0, 0.0)
            self.ser.close()
        rospy.loginfo("[bridge] 串口已关闭")


if __name__ == "__main__":
    rospy.init_node("serial_bridge")
    node = SerialBridge()
    rospy.on_shutdown(node.shutdown)
    node.run()
