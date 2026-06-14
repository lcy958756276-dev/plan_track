#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import serial
import re

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
        self.ser.reset_input_buffer()

        self.pub = rospy.Publisher(
            "/wheel_ticks",
            Int64MultiArray,
            queue_size=10
        )

        self.pattern = re.compile(
            r'ltick:(-?\d+)\s+rtick:(-?\d+)'
        )

        self.discard_count = 0

        rospy.loginfo("Open serial: %s", port)

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

                            msg = Int64MultiArray()
                            msg.data = [ltick, -rtick]

                            self.pub.publish(msg)

                            rospy.loginfo_throttle(
                                1.0,
                                f"ltick={ltick}, rtick={rtick}"
                            )
                        else:
                            self.discard_count += 1
                            rospy.logwarn_throttle(
                                5.0,
                                f"tick 值含非法字符，丢弃: ltick_str={ltick_str}, rtick_str={rtick_str}"
                            )
                    else:
                        self.discard_count += 1
                        rospy.logwarn_throttle(
                            5.0,
                            f"无法匹配格式，丢弃: {line}"
                        )
                else:
                    buf += c

            except Exception as e:
                rospy.logwarn(str(e))


if __name__ == "__main__":
    rospy.init_node("encoder_reader")

    node = EncoderReader()
    node.run()
