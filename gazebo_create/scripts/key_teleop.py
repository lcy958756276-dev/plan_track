#!/usr/bin/env python3
"""
简易键盘遥控小车，用于 gmapping 建图。
替代 teleop_twist_keyboard，无需安装额外包。
用法:  python3 key_teleop.py
    i=前进  k=停止  j=左转  l=右转
    ,=后退  u=左前  o=右前
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
控制键:
    i         k
  j   l    ← 键位参考
    ,

  i = 前进    , = 后退
  j = 左转    l = 右转
  u = 左前    o = 右前
  k = 停止

CTRL+C 退出
"""

key_bindings = {
    'i': (0.2, 0),     # 前进
    ',': (-0.2, 0),    # 后退
    'j': (0, 0.5),     # 左转
    'l': (0, -0.5),    # 右转
    'u': (0.15, 0.3),  # 左前
    'o': (0.15, -0.3), # 右前
    'k': (0, 0),       # 停止
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
        # 停止
        pub.publish(Twist())
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        print("已退出")
