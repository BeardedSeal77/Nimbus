#!/usr/bin/env python3
"""
Simple script to listen to Gazebo topics from WSL2
Run this INSIDE WSL2 to see what Gazebo is actually publishing
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
import time

class GazeboTopicListener(Node):
    def __init__(self):
        super().__init__('gazebo_topic_listener')

        # Subscribe to all Gazebo topics
        self.status_sub = self.create_subscription(
            String, '/tello/status', self.status_callback, 10)

        self.cmd_vel_sub = self.create_subscription(
            Twist, '/tello/cmd_vel', self.cmd_vel_callback, 10)

        self.odom_sub = self.create_subscription(
            Odometry, '/tello/odom', self.odom_callback, 10)

        self.image_sub = self.create_subscription(
            Image, '/tello/camera/image_raw', self.image_callback, 10)

        self.get_logger().info('Listening for Gazebo topics...')

        # Timer to show we're alive
        self.timer = self.create_timer(5.0, self.heartbeat)
        self.msg_count = 0

    def status_callback(self, msg):
        self.msg_count += 1
        self.get_logger().info(f'[{self.msg_count}] STATUS: {msg.data}')

    def cmd_vel_callback(self, msg):
        self.msg_count += 1
        self.get_logger().info(f'[{self.msg_count}] CMD_VEL: linear.x={msg.linear.x}, angular.z={msg.angular.z}')

    def odom_callback(self, msg):
        self.msg_count += 1
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.get_logger().info(f'[{self.msg_count}] ODOM: x={x:.2f}, y={y:.2f}')

    def image_callback(self, msg):
        self.msg_count += 1
        self.get_logger().info(f'[{self.msg_count}] IMAGE: {msg.width}x{msg.height}, encoding={msg.encoding}')

    def heartbeat(self):
        self.get_logger().info(f'Still listening... (received {self.msg_count} messages so far)')

def main():
    print("=" * 50)
    print("GAZEBO TOPIC LISTENER")
    print("Run this inside WSL2 while Gazebo is running")
    print("=" * 50)

    rclpy.init()
    listener = GazeboTopicListener()

    try:
        print("Listening for Gazebo topics for 30 seconds...")
        start_time = time.time()
        while time.time() - start_time < 30.0:
            rclpy.spin_once(listener, timeout_sec=1.0)

        print(f"\nDone! Received {listener.msg_count} total messages")

    except KeyboardInterrupt:
        print(f"\nStopped. Received {listener.msg_count} total messages")
    finally:
        listener.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()