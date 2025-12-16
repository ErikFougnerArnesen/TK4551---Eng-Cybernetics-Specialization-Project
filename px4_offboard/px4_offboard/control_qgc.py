#!/usr/bin/env python3

__author__ = "Erik Fougner Arnesen"
__contact__ = "erik.f.arnesen@gmail.com"

import sys

import geometry_msgs.msg
import rclpy

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty


msg = """
Keyboard teleop for Offboard velocity commands.

This node ONLY publishes /offboard_velocity_cmd.
Arming, takeoff, and flight modes are handled in QGroundControl.

Controls:
  w/s : Up/Down (local z)
  a/d : Yaw left/right
  Arrow keys : Move in x/y
  q/z : Increase/decrease linear speed
  e/c : Increase/decrease yaw rate
  x   : Reset integrated command to 0
  CTRL-C to quit
"""

moveBindings = {
    'w': (0, 0, 1, 0),
    's': (0, 0, -1, 0),
    'a': (0, 0, 0, -1),
    'd': (0, 0, 0, 1),
    '\x1b[A': (0, 1, 0, 0),
    '\x1b[B': (0, -1, 0, 0),
    '\x1b[C': (1, 0, 0, 0),
    '\x1b[D': (-1, 0, 0, 0),
}

speedBindings = {
    'q': (1.1, 1.0),
    'z': (0.9, 1.0),
    'e': (1.0, 1.1),
    'c': (1.0, 0.9),
}


def getKey(settings):
    if sys.platform == 'win32':
        return msvcrt.getwch()

    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    if key == '\x1b':
        key += sys.stdin.read(2)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed, turn):
    return f"currently: speed {speed:.2f}, yaw_rate {turn:.2f}"


def main():
    if sys.platform != 'win32':
        settings = termios.tcgetattr(sys.stdin)
    else:
        settings = None

    rclpy.init()

    qos_profile = QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1
    )

    node = rclpy.create_node('control_qgc')
    pub = node.create_publisher(geometry_msgs.msg.Twist, '/offboard_velocity_cmd', qos_profile)

    speed = 0.5
    turn = 0.2

    x = y = z = th = 0.0
    x_val = y_val = z_val = yaw_val = 0.0

    try:
        print(msg)
        print(vels(speed, turn))
        while True:
            key = getKey(settings)

            if key in moveBindings:
                x, y, z, th = moveBindings[key]
            elif key in speedBindings:
                speed *= speedBindings[key][0]
                turn *= speedBindings[key][1]
                print(vels(speed, turn))
                continue
            elif key == 'x':
                x_val = y_val = z_val = yaw_val = 0.0
                print('Reset command integrators to 0')
                continue
            elif key == '\x03':
                break
            else:
                x = y = z = th = 0.0

            twist = geometry_msgs.msg.Twist()
            x_val = (x * speed) + x_val
            y_val = (y * speed) + y_val
            z_val = (z * speed) + z_val
            yaw_val = (th * turn) + yaw_val

            twist.linear.x = float(x_val)
            twist.linear.y = float(y_val)
            twist.linear.z = float(z_val)
            twist.angular.z = float(yaw_val)

            pub.publish(twist)

    finally:
        pub.publish(geometry_msgs.msg.Twist())
        if sys.platform != 'win32' and settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
