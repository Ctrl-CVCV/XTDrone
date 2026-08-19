#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""V1 keyboard teleop shared module: run_teleop(car) drives /carN/cmd_vel.
Same keys/speeds as the single-car keyboard controller (mecanum omnidirectional):
  w/s 前进/后退  a/d 左移/右移  q/e 左转/右转
  =/+ 线速度档+0.1  - 线速度档-0.1  ] 角速度档+0.1  [ 角速度档-0.1
  空格 停止  t 帮助  CTRL-C 退出(自动发零速度)
"""
import rospy
import sys
import select
import os
import termios
import tty
from geometry_msgs.msg import Twist

LIN_STEP = 0.1
ANG_STEP = 0.1
MAX_LIN = 2.0
MAX_ANG = 3.0
RATE = 10.0


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def get_key(timeout=0.1):
    if select.select([sys.stdin], [], [], timeout)[0]:
        return os.read(sys.stdin.fileno(), 1).decode()
    return None


def report(tw, lin_speed, ang_speed):
    sys.stdout.write("\rlin.x=%.2f lin.y=%.2f ang.z=%.2f | 档位: 线=%.2f m/s 角=%.2f rad/s   \r" %
                     (tw.linear.x, tw.linear.y, tw.angular.z, lin_speed, ang_speed))
    sys.stdout.flush()


def run_teleop(car):
    rospy.init_node("keyboard_%s" % car, anonymous=True)
    pub = rospy.Publisher("/%s/cmd_vel" % car, Twist, queue_size=1)
    rate = rospy.Rate(RATE)

    lin_speed = 0.5
    ang_speed = 0.5
    tw = Twist()
    stop_pressed = False

    old_attr = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        rospy.loginfo("%s 键盘控制已启动 (w/s/a/d/q/e 运动, +/- 线速度, ]/[ 角速度, 空格 停止, t 帮助)",
                      car)
        print()
        report(tw, lin_speed, ang_speed)

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
                report(tw, lin_speed, ang_speed)

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
        print("\n[%s 已退出,已发布零速度]" % car)
        sys.stdout.flush()
