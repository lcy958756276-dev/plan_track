#!/usr/bin/env python3
"""
简易键盘遥控小车，用于 gmapping 建图。
用法:  python3 key_teleop.py
    w=前进  s=后退  a=左转  d=右转
    空格=停止
"""

import sys
import select
import termios
import tty
import rospy
from geometry_msgs.msg import Twist

msg = """
简易键盘遥控
---------------------------
  w = 前进
  a = 左转    d = 右转
  s = 后退
  空格 = 停止

CTRL+C 退出
---------------------------
"""

key_bindings = {
    'w': (0.1, 0),       # 前进（慢速，便于建图）
    's': (-0.1, 0),      # 后退
    'a': (0, 0.2),       # 左转
    'd': (0, -0.2),      # 右转
    '1': (0.2, 0),       # 稍快前进
    '2': (0, 0.4),       # 稍快转弯
    ' ': (0, 0),          # 停止（空格）
}

def get_key():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    return key

if __name__ == '__main__':
    rospy.init_node('key_teleop')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    old_settings = termios.tcgetattr(sys.stdin)

    print(msg)
    try:
        while not rospy.is_shutdown():
            key = get_key()
            if key in key_bindings:
                twist = Twist()
                twist.linear.x = key_bindings[key][0]
                twist.angular.z = key_bindings[key][1]
                pub.publish(twist)
            elif key == '\x03':  # CTRL+C
                break
    except rospy.ROSInterruptException:
        pass
    finally:
        pub.publish(Twist())
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        print("已退出")
