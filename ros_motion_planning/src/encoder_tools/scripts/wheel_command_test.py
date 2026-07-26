#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
wheel_command_test.py

独立串口测试左右轮命令执行情况。运行前请停止 serial_bridge.py，
否则两个节点会抢同一个串口。

默认测试序列:
  1) l:0.000,r:-0.050
  2) l:-0.050,r:0.000
  3) l:-0.050,r:-0.050
  4) l:0.000,r:0.000

用法:
  rosrun encoder_tools wheel_command_test.py
  rosrun encoder_tools wheel_command_test.py _port:=/dev/ttyTHS0 _speed:=-0.05 _step_duration:=2.0
"""

import math
import os
import re
import threading
import time
import termios

import rospy
import serial


class WheelCommandTest:
    def __init__(self):
        self.port = rospy.get_param("~port", "/dev/ttyTHS0")
        self.baud = rospy.get_param("~baud", 57600)
        self.speed = rospy.get_param("~speed", -0.05)
        self.step_duration = rospy.get_param("~step_duration", 2.0)
        self.zero_duration = rospy.get_param("~zero_duration", 2.0)
        self.send_period = rospy.get_param("~send_period", 0.1)
        self.wheel_radius = rospy.get_param("~wheel_radius", 0.1065)

        pluse = 1.04190106 * 360.0 / (500.0 * 4 * 91)
        self.dist_per_tick = (pluse / 360.0) * 2.0 * math.pi * self.wheel_radius

        self.pattern_ltick_rtick = re.compile(r"ltick:\s*(-?\d+)\s+rtick:\s*(-?\d+)")
        self.pattern_echo = re.compile(r"target_[lr]|^l:-?\d|^V\s")

        self.last_ltick = None
        self.last_rtick = None
        self.last_tick_time = None
        self.latest_actual_left = None
        self.latest_actual_right = None
        self.latest_tick_age = None
        self.running = True
        self.write_lock = threading.Lock()

        self.ser = serial.Serial(port=self.port, baudrate=self.baud, timeout=0.05)
        self._set_raw_mode()
        self.ser.reset_input_buffer()

        rospy.loginfo(
            "[wheel_test] 串口已打开: %s @ %d, dist_per_tick=%.8f m",
            self.port,
            self.baud,
            self.dist_per_tick,
        )
        rospy.logwarn("[wheel_test] 请确认 serial_bridge.py 已停止，否则测试结果不可信")

        self.reader = threading.Thread(target=self._read_loop)
        self.reader.daemon = True
        self.reader.start()

    def _set_raw_mode(self):
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

    def _read_loop(self):
        buf = b""
        while self.running and not rospy.is_shutdown():
            try:
                c = self.ser.read(1)
                if not c:
                    continue
                if c != b"\n":
                    buf += c
                    continue

                line = buf.decode("utf-8", errors="ignore").strip()
                buf = b""
                if not line or self.pattern_echo.search(line):
                    continue

                match = self.pattern_ltick_rtick.search(line)
                if not match:
                    rospy.loginfo_throttle(2.0, "[wheel_test] 未解析串口行: %s", line)
                    continue

                self._handle_ticks(int(match.group(1)), int(match.group(2)))
            except Exception as exc:
                rospy.logwarn_throttle(1.0, "[wheel_test] 串口读取异常: %s", exc)

    def _handle_ticks(self, ltick, rtick):
        now = rospy.Time.now()
        if self.last_ltick is None:
            self.last_ltick = ltick
            self.last_rtick = rtick
            self.last_tick_time = now
            rospy.loginfo("[wheel_test] 第一帧 tick: ltick=%d rtick=%d", ltick, rtick)
            return

        dt = (now - self.last_tick_time).to_sec()
        if dt <= 0:
            return

        dl_tick = ltick - self.last_ltick
        dr_tick = rtick - self.last_rtick
        v_left = dl_tick * self.dist_per_tick / dt
        v_right = dr_tick * self.dist_per_tick / dt

        self.last_ltick = ltick
        self.last_rtick = rtick
        self.last_tick_time = now
        self.latest_actual_left = v_left
        self.latest_actual_right = v_right
        self.latest_tick_age = 0.0

        rospy.loginfo(
            "[wheel_test] tick ltick=%d rtick=%d Δl=%d Δr=%d actual_l=%.3f actual_r=%.3f dt=%.3f",
            ltick,
            rtick,
            dl_tick,
            dr_tick,
            v_left,
            v_right,
            dt,
        )

    def _write_command(self, left, right):
        cmd = f"l:{left:.3f},r:{right:.3f}\r\n"
        with self.write_lock:
            os.write(self.ser.fileno(), cmd.encode("utf-8"))
            termios.tcdrain(self.ser.fileno())
        return cmd.strip()

    def _actual_text(self):
        if self.latest_actual_left is None or self.last_tick_time is None:
            return "actual_l=NA actual_r=NA"
        age = (rospy.Time.now() - self.last_tick_time).to_sec()
        return (
            f"actual_l={self.latest_actual_left:.3f} "
            f"actual_r={self.latest_actual_right:.3f} age={age:.3f}s"
        )

    def run(self):
        steps = [
            ("right_reverse_only", 0.0, self.speed, self.step_duration),
            ("left_reverse_only", self.speed, 0.0, self.step_duration),
            ("both_reverse", self.speed, self.speed, self.step_duration),
            ("final_zero", 0.0, 0.0, self.zero_duration),
        ]

        rospy.logwarn("[wheel_test] 2 秒后开始测试，请抬起车轮或确保周围安全")
        rospy.sleep(2.0)

        for name, left, right, duration in steps:
            start = rospy.Time.now()
            rospy.logwarn(
                "[wheel_test] STEP_START %s duration=%.2fs target_l=%.3f target_r=%.3f %s",
                name,
                duration,
                left,
                right,
                self._actual_text(),
            )
            while not rospy.is_shutdown() and (rospy.Time.now() - start).to_sec() < duration:
                sent = self._write_command(left, right)
                rospy.logwarn_throttle(
                    0.5,
                    "[wheel_test] STEP %s send=%s %s",
                    name,
                    sent,
                    self._actual_text(),
                )
                rospy.sleep(self.send_period)

            rospy.logwarn("[wheel_test] STEP_DONE %s %s", name, self._actual_text())

        for _ in range(10):
            sent = self._write_command(0.0, 0.0)
            rospy.loginfo("[wheel_test] stop send=%s %s", sent, self._actual_text())
            rospy.sleep(0.1)

    def shutdown(self):
        self.running = False
        try:
            for _ in range(5):
                self._write_command(0.0, 0.0)
                time.sleep(0.05)
        except Exception:
            pass
        if self.ser and self.ser.is_open:
            self.ser.close()
        rospy.loginfo("[wheel_test] 串口已关闭")


if __name__ == "__main__":
    rospy.init_node("wheel_command_test")
    node = WheelCommandTest()
    rospy.on_shutdown(node.shutdown)
    node.run()
