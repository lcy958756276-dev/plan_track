#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
send_uart.py
键盘输入 A/B/C → 串口透传给电机驱动板

用法:
    rosrun encoder_tools send_uart.py
    # 或直接 python send_uart.py
"""

import sys
import serial
import rospy


class KeyboardToSerial:
    def __init__(self):
        port = rospy.get_param("~port", "/dev/ttyTHS0")
        baud = rospy.get_param("~baud", 115200)

        try:
            self.ser = serial.Serial(port=port, baudrate=baud, timeout=0.1)
            rospy.loginfo(f"串口已打开: {port} @ {baud}")
        except serial.SerialException as e:
            rospy.logerr(f"无法打开串口 {port}: {e}")
            sys.exit(1)

    def run(self):
        rospy.loginfo("等待键盘输入: A / B / C (q 退出)")
        while not rospy.is_shutdown():
            try:
                # 读取一个字符 (Python 3)
                ch = input(">>> ").strip().upper()

                if not ch:
                    continue
                if ch == 'Q':
                    rospy.loginfo("退出")
                    break
                if ch not in ('A', 'B', 'C'):
                    rospy.logwarn(f"无效指令: {ch}，只接受 A/B/C")
                    continue

                # 透传发送
                cmd = ch + '\n'
                self.ser.write(cmd.encode('utf-8'))
                rospy.loginfo(f"发送: {ch}")

            except EOFError:
                break
            except KeyboardInterrupt:
                break
            except serial.SerialException as e:
                rospy.logerr(f"串口错误: {e}")
                break

        self.ser.close()


if __name__ == "__main__":
    rospy.init_node("keyboard_to_serial")
    node = KeyboardToSerial()
    node.run()
