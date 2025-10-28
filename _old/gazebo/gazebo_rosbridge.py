#!/usr/bin/env python3
"""
Nimbus Gazebo-ROS2 WebSocket Bridge
Bridges topics between Gazebo (WSL2) and ROS2 (Docker) via WebSocket
Treats them as separate machines on the network
"""

import rclpy
from rclpy.node import Node
import websocket
import json
import threading
import time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image

class GazeboRosbridgeNode(Node):
    def __init__(self):
        super().__init__('gazebo_rosbridge')

        # WebSocket connection to Docker ROS2
        self.rosbridge_url = "ws://192.168.8.102:9090"
        self.ws = None
        self.connect_websocket()

        # ROS2 subscriptions (from Gazebo)
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/tello/cmd_vel', self.cmd_vel_callback, 10)

        # ROS2 publishers (to Gazebo)
        self.odom_pub = self.create_publisher(Odometry, '/tello/odom', 10)
        self.image_pub = self.create_publisher(Image, '/tello/camera/image_raw', 10)

        self.get_logger().info('🌐 Gazebo-ROS2 WebSocket Bridge Started')
        self.get_logger().info(f'🔗 Connected to: {self.rosbridge_url}')

    def connect_websocket(self):
        """Connect to rosbridge WebSocket"""
        try:
            self.ws = websocket.WebSocket()
            self.ws.connect(self.rosbridge_url)
            self.get_logger().info('✅ WebSocket connected to Docker ROS2')

            # Start WebSocket listener thread
            self.ws_thread = threading.Thread(target=self.websocket_listener, daemon=True)
            self.ws_thread.start()

        except Exception as e:
            self.get_logger().error(f'❌ WebSocket connection failed: {e}')

    def websocket_listener(self):
        """Listen for messages from Docker ROS2"""
        while True:
            try:
                message = self.ws.recv()
                data = json.loads(message)
                self.handle_websocket_message(data)
            except Exception as e:
                self.get_logger().error(f'WebSocket receive error: {e}')
                time.sleep(1)

    def handle_websocket_message(self, data):
        """Handle incoming WebSocket messages from Docker ROS2"""
        if data.get('op') == 'publish':
            topic = data.get('topic')
            msg_data = data.get('msg', {})

            if topic == '/tello/odom':
                # Convert and publish odometry
                self.get_logger().info('📡 Received odometry from Docker ROS2')

            elif topic == '/tello/camera/image_raw':
                # Convert and publish image
                self.get_logger().info('📡 Received camera image from Docker ROS2')

    def cmd_vel_callback(self, msg):
        """Forward cmd_vel from Gazebo to Docker ROS2"""
        websocket_msg = {
            "op": "publish",
            "topic": "/tello/cmd_vel",
            "msg": {
                "linear": {
                    "x": msg.linear.x,
                    "y": msg.linear.y,
                    "z": msg.linear.z
                },
                "angular": {
                    "x": msg.angular.x,
                    "y": msg.angular.y,
                    "z": msg.angular.z
                }
            }
        }

        try:
            self.ws.send(json.dumps(websocket_msg))
            self.get_logger().info('📤 Sent cmd_vel to Docker ROS2')
        except Exception as e:
            self.get_logger().error(f'Failed to send cmd_vel: {e}')

def main():
    rclpy.init()

    try:
        bridge = GazeboRosbridgeNode()
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()