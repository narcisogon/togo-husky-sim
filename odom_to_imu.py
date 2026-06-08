#!/usr/bin/env python3
"""Publish a simulated IMU stream from platform odometry.

This is a development fallback for Clearpath Gazebo configs where the IMU
interface exists but no Gazebo IMU bridge is publishing. It is not a substitute
for a true physics IMU model, but it gives LiDAR-inertial stacks a live
sensor_msgs/Imu topic for integration testing.
"""

import argparse
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu


class OdomToImu(Node):
    def __init__(self, odom_topic, imu_topic, frame_id):
        super().__init__('odom_to_imu')
        self.imu_topic = imu_topic
        self.frame_id = frame_id
        self.prev_time = None
        self.prev_vx = 0.0
        self.prev_vy = 0.0
        self.prev_vz = 0.0

        self.pub = self.create_publisher(Imu, imu_topic, 10)
        self.sub = self.create_subscription(Odometry, odom_topic, self.on_odom, 10)
        self.get_logger().info(f'Subscribed to odom: {odom_topic}')
        self.get_logger().info(f'Publishing simulated IMU: {imu_topic}')

    def on_odom(self, odom):
        now = rclpy.time.Time.from_msg(odom.header.stamp)
        vx = odom.twist.twist.linear.x
        vy = odom.twist.twist.linear.y
        vz = odom.twist.twist.linear.z

        ax = 0.0
        ay = 0.0
        az = 9.80665
        if self.prev_time is not None:
            dt = (now - self.prev_time).nanoseconds * 1e-9
            if dt > 1e-6 and math.isfinite(dt):
                ax = (vx - self.prev_vx) / dt
                ay = (vy - self.prev_vy) / dt
                az = (vz - self.prev_vz) / dt + 9.80665

        self.prev_time = now
        self.prev_vx = vx
        self.prev_vy = vy
        self.prev_vz = vz

        imu = Imu()
        imu.header.stamp = odom.header.stamp
        imu.header.frame_id = self.frame_id
        imu.orientation = odom.pose.pose.orientation
        imu.angular_velocity = odom.twist.twist.angular
        imu.linear_acceleration.x = ax
        imu.linear_acceleration.y = ay
        imu.linear_acceleration.z = az

        # Mark covariance as approximate, but usable by consumers that require it.
        imu.orientation_covariance = [0.05, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.1]
        imu.angular_velocity_covariance = [0.02, 0.0, 0.0, 0.0, 0.02, 0.0, 0.0, 0.0, 0.02]
        imu.linear_acceleration_covariance = [0.2, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 0.3]
        self.pub.publish(imu)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--odom-topic', default='/a300_0000/platform/odom')
    parser.add_argument('--imu-topic', default='/a300_0000/sensors/imu_0/data')
    parser.add_argument('--frame-id', default='imu_0')
    args = parser.parse_args()

    rclpy.init()
    node = OdomToImu(args.odom_topic, args.imu_topic, args.frame_id)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


