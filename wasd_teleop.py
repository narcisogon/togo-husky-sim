#!/usr/bin/env python3
"""Simple WASD teleop for a Clearpath Husky/A300 sim.

Publishes geometry_msgs/msg/TwistStamped to a Clearpath cmd_vel topic.
"""

import argparse
import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import TwistStamped


HELP = """
WASD Teleop
----------
w / s      forward / backward
a / d      rotate left / rotate right
space      stop
+ / -      increase / decrease linear speed
] / [      increase / decrease angular speed
q          quit
"""


def get_key(timeout=0.1):
    readable, _, _ = select.select([sys.stdin], [], [], timeout)
    if readable:
        return sys.stdin.read(1)
    return None


def clamp(value, low, high):
    return max(low, min(high, value))


def main():
    parser = argparse.ArgumentParser(description="WASD teleop for Clearpath cmd_vel")
    parser.add_argument("--topic", default="/a300_0000/cmd_vel", help="TwistStamped topic to publish")
    parser.add_argument("--linear", type=float, default=0.25, help="Initial linear speed in m/s")
    parser.add_argument("--angular", type=float, default=0.6, help="Initial angular speed in rad/s")
    parser.add_argument("--rate", type=float, default=20.0, help="Publish rate while a key is held")
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("wasd_teleop")
    pub = node.create_publisher(TwistStamped, args.topic, 10)

    linear_speed = args.linear
    angular_speed = args.angular
    linear = 0.0
    angular = 0.0

    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())

    print(HELP)
    print(f"Publishing to: {args.topic}")
    print(f"Linear: {linear_speed:.2f} m/s | Angular: {angular_speed:.2f} rad/s")

    try:
        while rclpy.ok():
            key = get_key(timeout=1.0 / args.rate)

            if key == "q":
                break
            if key == "w":
                linear = linear_speed
                angular = 0.0
            elif key == "s":
                linear = -linear_speed
                angular = 0.0
            elif key == "a":
                linear = 0.0
                angular = angular_speed
            elif key == "d":
                linear = 0.0
                angular = -angular_speed
            elif key == " ":
                linear = 0.0
                angular = 0.0
            elif key == "+" or key == "=":
                linear_speed = clamp(linear_speed + 0.05, 0.05, 1.5)
                print(f"Linear speed: {linear_speed:.2f} m/s")
            elif key == "-" or key == "_":
                linear_speed = clamp(linear_speed - 0.05, 0.05, 1.5)
                print(f"Linear speed: {linear_speed:.2f} m/s")
            elif key == "]":
                angular_speed = clamp(angular_speed + 0.1, 0.1, 2.5)
                print(f"Angular speed: {angular_speed:.2f} rad/s")
            elif key == "[":
                angular_speed = clamp(angular_speed - 0.1, 0.1, 2.5)
                print(f"Angular speed: {angular_speed:.2f} rad/s")
            elif key is not None:
                linear = 0.0
                angular = 0.0

            msg = TwistStamped()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.header.frame_id = "base_link"
            msg.twist.linear.x = linear
            msg.twist.angular.z = angular
            pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.0)
    finally:
        stop = TwistStamped()
        stop.header.stamp = node.get_clock().now().to_msg()
        stop.header.frame_id = "base_link"
        pub.publish(stop)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        rclpy.shutdown()
        print("\nStopped.")


if __name__ == "__main__":
    main()
