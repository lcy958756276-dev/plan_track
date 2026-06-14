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
            timeout=0.1
        )

        self.pub = rospy.Publisher(
            "/wheel_ticks",
            Int64MultiArray,
            queue_size=10
        )

        self.pattern = re.compile(
            r'ltick:(-?\d+)\s+rtick:(-?\d+)'
        )

        rospy.loginfo("Open serial: %s", port)

    def run(self):
        while not rospy.is_shutdown():
            try:
                line = self.ser.readline().decode(
                    'utf-8',
                    errors='ignore'
                ).strip()

                if not line:
                    continue

                match = self.pattern.search(line)

                if match:
                    ltick = int(match.group(1))
                    rtick = int(match.group(2))

                    msg = Int64MultiArray()
                    msg.data = [ltick, -rtick]

                    self.pub.publish(msg)

                    rospy.loginfo(
                        f"ltick={ltick}, rtick={rtick}"
                    )

            except Exception as e:
                rospy.logwarn(str(e))


if __name__ == "__main__":
    rospy.init_node("encoder_reader")

    node = EncoderReader()
    node.run()