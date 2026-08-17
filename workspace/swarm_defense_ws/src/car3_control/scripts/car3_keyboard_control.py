#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""car3 麦轮键盘控制器(teleop 风格)
发布 Twist 到 /cmd_vel(planar_move 插件订阅),支持麦轮全向运动。

设计:按方向键 = 把该方向速度设成当前档位(长按不会失控),用 +/- 调节速度档。
按键:
  w / s : 前进 / 后退        (linear.x = ±当前线速度)
  a / d : 左移 / 右移        (linear.y = ±当前线速度, 麦轮横移)
  q / e : 左转 / 右转        (angular.z = ±当前角速度)
  = / + : 线速度 +0.1 m/s
  -     : 线速度 -0.1 m/s
  ]     : 角速度 +0.1 rad/s
  [     : 角速度 -0.1 rad/s
  空格  : 停止(清零)
  t     : 显示帮助
  CTRL-C: 退出(自动发布零速度)
每次按键都会打印当前指令,方便确认速度变化。
"""
import rospy
import sys
import select
import os
import termios
import tty
from geometry_msgs.msg import Twist

LIN_STEP = 0.1          # m/s 每次按键步进
ANG_STEP = 0.1          # rad/s 每次按键步进
MAX_LIN = 2.0
MAX_ANG = 3.0
RATE = 10.0

lin_speed = 0.3         # 当前线速度档位 (m/s, 建图时低速更稳)
ang_speed = 0.5         # 当前角速度档位 (rad/s)


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def get_key(timeout=0.1):
    if select.select([sys.stdin], [], [], timeout)[0]:
        return os.read(sys.stdin.fileno(), 1).decode()
    return None


def report(tw):
    sys.stdout.write("\rlin.x=%.2f lin.y=%.2f ang.z=%.2f | 档位: 线=%.2f m/s 角=%.2f rad/s   \r" %
                     (tw.linear.x, tw.linear.y, tw.angular.z, lin_speed, ang_speed))
    sys.stdout.flush()


def main():
    global lin_speed, ang_speed
    rospy.init_node("car3_keyboard_control", anonymous=True)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    rate = rospy.Rate(RATE)

    tw = Twist()
    stop_pressed = False

    old_attr = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        rospy.loginfo("car3 键盘控制已启动  (w/s/a/d/q/e 运动, +/- 线速度, ]/[ 角速度, 空格 停止, t 帮助)")
        print()
        report(tw)

        while not rospy.is_shutdown():
            ch = get_key()
            if ch is not None:
                if ch == "w":
                    tw.linear.x = lin_speed
                elif ch == "s":
                    tw.linear.x = -lin_speed
                elif ch == "a":
                    tw.linear.y = lin_speed
                elif ch == "d":
                    tw.linear.y = -lin_speed
                elif ch == "q":
                    tw.angular.z = ang_speed
                elif ch == "e":
                    tw.angular.z = -ang_speed
                elif ch in ("=", "+"):
                    lin_speed = clamp(lin_speed + LIN_STEP, 0.0, MAX_LIN)
                elif ch == "-":
                    lin_speed = clamp(lin_speed - LIN_STEP, 0.0, MAX_LIN)
                elif ch == "]":
                    ang_speed = clamp(ang_speed + ANG_STEP, 0.0, MAX_ANG)
                elif ch == "[":
                    ang_speed = clamp(ang_speed - ANG_STEP, 0.0, MAX_ANG)
                elif ch == " ":
                    stop_pressed = True
                elif ch == "t":
                    print("\n[帮助]  w/s 前后  a/d 横移  q/e 旋转  +=/- 线速度  ]/[ 角速度  空格 停止")
                else:
                    continue
                if stop_pressed:
                    tw = Twist()
                    stop_pressed = False
                    print("\n[停止]")
                report(tw)

            pub.publish(tw)
            rate.sleep()

    except KeyboardInterrupt:
        pass
    finally:
        try:
            pub.publish(Twist())
        except Exception:
            pass
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)
        print("\n[已退出,已发布零速度]")
        sys.stdout.flush()


if __name__ == "__main__":
    main()
