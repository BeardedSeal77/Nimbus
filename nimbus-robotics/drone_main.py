#!/usr/bin/env python3
"""
Drone Main Application
Handles drone control, flight operations, video capture, and ROS2 publishing for the Nimbus system.
"""


import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class DroneNode(Node):
    def __init__(self):
        super().__init__('drone_node')
        self.command_sub = self.create_subscription(
            String,
            '/drone_command',
            self.command_callback,
            10
        )
        self.status_pub = self.create_publisher(
            String,
            '/drone_status',
            10
        )
        self.get_logger().info('Drone node started, waiting for commands...')

    def command_callback(self, msg):
        if msg.data == 'green_light':
            self.get_logger().info('Received green light from web UI, starting movement sequence')
            # left 0.5m, up 0.5m, right 0.5m, down 0.5m (back to origin)
            moves = [
                ('left', 0.5),
                ('up', 0.5),
                ('right', 0.5),
                ('down', 0.5)
            ]
            for direction, distance in moves:
                self.simulate_move(direction, distance)
                status_msg = String()
                status_msg.data = f'Moved {direction} {distance} meters'
                self.status_pub.publish(status_msg)
                self.get_logger().info(f'Published status: {status_msg.data}')

    def simulate_move(self, direction, distance):
        # Simulate movement
        # For now, just log and sleep 1 second per move to simulate time taken
        self.get_logger().info(f'Simulating {direction} movement of {distance} meters...')
        time.sleep(1)

def main(args=None):
    rclpy.init(args=args)
    drone_node = DroneNode()
    rclpy.spin(drone_node)
    drone_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()