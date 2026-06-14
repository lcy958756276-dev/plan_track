#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import serial
import re

from std_msgs.msg import Int64MultiArray


class EncoderReader:
    def __init__(self):
        port = rospy.get_param("~port", "/dev/ttyTHS0")
        baud = rospy.get_param("~baud", 115200)

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

        rospy.loginfo("Open serial: %s", port)
        rospy.loginfo("调试模式：原始数据直接打印")

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

                    # 调试：直接打印每行原始数据
                    if line:
                        rospy.loginfo("RAW: %s", line)

                else:
                    buf += c

            except Exception as e:
                rospy.logwarn(str(e))


if __name__ == "__main__":
    rospy.init_node("encoder_reader")

    node = EncoderReader()
    node.run()
