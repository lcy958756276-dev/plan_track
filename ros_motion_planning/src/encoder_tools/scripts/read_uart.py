#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import serial
import re
import termios

from std_msgs.msg import Int64MultiArray


class EncoderReader:
    def __init__(self):
        port = rospy.get_param("~port", "/dev/ttyTHS0")
        baud = rospy.get_param("~baud", 57600)

        self.ser = serial.Serial(
            port=port,
            baudrate=baud,
            timeout=0.5
        )

        # ── 强制 raw 模式：关闭所有终端处理 ──
        fd = self.ser.fileno()
        tty = termios.tcgetattr(fd)
        tty[0] = 0  # iflag = 0
        tty[1] = 0  # oflag = 0
        tty[2] = tty[2] & ~termios.PARENB   # 无校验
        tty[2] = tty[2] & ~termios.CSTOPB   # 1位停止位
        tty[2] = tty[2] & ~termios.CSIZE    # 清除数据位
        tty[2] = tty[2] | termios.CS8       # 8位数据
        tty[2] = tty[2] & ~termios.CRTSCTS  # 关闭硬件流控
        tty[2] = tty[2] | termios.CREAD     # 使能接收
        tty[2] = tty[2] & ~termios.HUPCL    # 关闭时不断开DTR
        tty[3] = 0  # lflag = 0
        termios.tcsetattr(fd, termios.TCSANOW, tty)

        self.ser.reset_input_buffer()

        self.pub = rospy.Publisher(
            "/wheel_ticks",
            Int64MultiArray,
            queue_size=10
        )

        self.pattern = re.compile(
            r'ltick:(-?\d+)\s+rtick:(-?\d+)'
        )

        # 上一帧的数值，用于缺位检测
        self.last_ltick = None
        self.last_rtick = None

        rospy.loginfo("Open serial: %s (raw mode)", port)

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

                    match = self.pattern.search(line)

                    if match:
                        ltick_str = match.group(1)
                        rtick_str = match.group(2)

                        # 确保提取到的 tick 值只有数字和负号
                        if re.match(r'^-?\d+$', ltick_str) and re.match(r'^-?\d+$', rtick_str):
                            ltick = int(ltick_str)
                            rtick = int(rtick_str)

                            # ── 缺位滤波器 ──
                            # 缺位错误：数字少一位 → 值突然缩小约 10 倍
                            # 用相对比例检测，不依赖速度（任何速度下都不会一帧内变化 80% 以上）
                            if self._check_sanity(ltick, rtick):
                                msg = Int64MultiArray()
                                msg.data = [ltick, -rtick]
                                self.pub.publish(msg)

                                self.last_ltick = ltick
                                self.last_rtick = rtick

                                rospy.loginfo_throttle(
                                    1.0,
                                    f"ltick={ltick}, rtick={rtick}"
                                )
                        else:
                            rospy.logwarn_throttle(
                                5.0,
                                f"tick 值含非法字符，丢弃: ltick_str={ltick_str}, rtick_str={rtick_str}"
                            )
                    else:
                        rospy.logwarn_throttle(
                            5.0,
                            f"无法匹配格式，丢弃: {line}"
                        )
                else:
                    buf += c

            except Exception as e:
                rospy.logwarn(str(e))

    def _check_sanity(self, ltick, rtick):
        """检查 tick 值是否合理，过滤缺位错误"""
        # ltick 检查（始终正数，正向递增，反向递减）
        if self.last_ltick is not None and self.last_ltick > 10000:
            # 缺位错误：新值 < 旧值的 20%（至少缩小 5 倍）
            if ltick < self.last_ltick * 0.2:
                rospy.logwarn(f"ltick 缺位过滤: {self.last_ltick} → {ltick}")
                return False

        # rtick 检查（正向时负数且绝对值递增，反向时绝对值递减）
        # 缺位错误：绝对值缩小到 20% 以下（如 -203098 → -20309）
        # 注：不过滤符号翻转，因为倒车过零点是正常现象
        if self.last_rtick is not None and abs(self.last_rtick) > 10000:
            if abs(rtick) < abs(self.last_rtick) * 0.2:
                rospy.logwarn(f"rtick 缺位过滤: {self.last_rtick} → {rtick}")
                return False

        return True


if __name__ == "__main__":
    rospy.init_node("encoder_reader")

    node = EncoderReader()
    node.run()
