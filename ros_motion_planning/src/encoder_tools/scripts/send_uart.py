#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
send_uart.py
键盘输入 A/B/C → 串口透传给电机驱动板

注意: 波特率必须与 read_uart.py 一致（默认 57600），
      否则打开串口时会冲掉 read_uart 的配置，导致数据中断。

用法:
    rosrun encoder_tools send_uart.py
    # 或直接 python send_uart.py
"""

import sys
import serial
import termios
import rospy


class KeyboardToSerial:
    def __init__(self):
        port = rospy.get_param("~port", "/dev/ttyTHS0")
        # 必须与 read_uart.py 一致，否则会冲掉对方的串口配置
        baud = rospy.get_param("~baud", 57600)

        try:
            self.ser = serial.Serial(port=port, baudrate=baud, timeout=0.1)

            # 同样应用 raw termios 模式，避免破坏 read_uart 的配置
            fd = self.ser.fileno()
            tty = termios.tcgetattr(fd)
            tty[0] = 0  # iflag = 0
            tty[1] = 0  # oflag = 0
            tty[2] = tty[2] & ~termios.PARENB
            tty[2] = tty[2] & ~termios.CSTOPB
            tty[2] = tty[2] & ~termios.CSIZE
            tty[2] = tty[2] | termios.CS8
            tty[2] = tty[2] & ~termios.CRTSCTS
            tty[2] = tty[2] | termios.CREAD
            tty[2] = tty[2] & ~termios.HUPCL
            tty[3] = 0  # lflag = 0
            termios.tcsetattr(fd, termios.TCSANOW, tty)

            rospy.loginfo(f"串口已打开: {port} @ {baud} (raw mode)")

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
                cmd = ch + '\r\n'
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
