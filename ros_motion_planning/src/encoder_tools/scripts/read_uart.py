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

        # 上一帧的数值，用于斜率检测
        self.last_ltick = None
        self.last_rtick = None
        # 历史 delta 记录，用于自适应阈值
        self.delta_hist_ltick = []
        self.delta_hist_rtick = []

        # 原始数据明细日志
        try:
            self.raw_log = open("/tmp/read_uart_raw.log", "w")
            self.raw_log.write("ros_time\tltick\trtick\tstatus\treason\n")
        except Exception:
            self.raw_log = None

        rospy.loginfo("Open serial: %s (raw mode)", port)

    def _log_raw(self, stamp, ltick, rtick, status, reason):
        if self.raw_log is not None:
            t = stamp.to_sec() if hasattr(stamp, 'to_sec') else rospy.Time.now().to_sec()
            self.raw_log.write(f"{t:.3f}\t{ltick}\t{rtick}\t{status}\t{reason}\n")

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

                        if re.match(r'^-?\d+$', ltick_str) and re.match(r'^-?\d+$', rtick_str):
                            ltick = int(ltick_str)
                            rtick = int(rtick_str)

                            now = rospy.Time.now()

                            if self._check_sanity(ltick, rtick):
                                msg = Int64MultiArray()
                                msg.data = [ltick, -rtick]
                                self.pub.publish(msg)

                                self.last_ltick = ltick
                                self.last_rtick = rtick

                                self._log_raw(now, ltick, rtick, "OK", "")
                                rospy.loginfo_throttle(1.0, f"ltick={ltick}, rtick={rtick}")
                            else:
                                self._log_raw(now, ltick, rtick, "FILTER", "slope")
                        else:
                            self._log_raw(rospy.Time.now(), ltick_str, rtick_str, "FILTER", "chars")
                            rospy.logwarn_throttle(5.0, f"tick 值含非法字符，丢弃: ltick_str={ltick_str}, rtick_str={rtick_str}")
                    else:
                        rospy.logwarn_throttle(5.0, f"无法匹配格式，丢弃: {line}")
                else:
                    buf += c

            except Exception as e:
                rospy.logwarn(str(e))

    def _check_sanity(self, ltick, rtick):
        """自适应斜率阈值：检查每帧变化量是否合理"""
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
                rospy.logwarn(f"{name} 斜率异常: {delta} (平均 delta={avg:.0f}, 阈值={max_allowed:.0f})")
                return False
        else:
            if abs(delta) > 100000:
                rospy.logwarn(f"{name} 首帧斜率异常: {delta}")
                return False

        history.append(delta)
        if len(history) > 10:
            history.pop(0)
        return True


if __name__ == "__main__":
    rospy.init_node("encoder_reader")

    node = EncoderReader()
    node.run()
