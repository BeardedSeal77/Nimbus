#!/usr/bin/env python3
"""
Drone Main Application
Handles drone control, flight operations, video capture, and ROS2 publishing for the Nimbus system.
"""

import os
import logging
import asyncio
import threading
import cv2
from flask import Flask, jsonify, request
from mavsdk import System
from mavsdk.action import ActionError
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Logging setup (outputs to terminal/console)
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Flask app for receiving commands from web UI
app = Flask(__name__)

# Node for publishing to MR
class DroneROSNode(Node):
    def __init__(self, drone_system):
        super().__init__('drone_main_node')
        self.bridge = CvBridge()
        self.drone_system = drone_system  # MAVSDK system for control
        self.video_publisher = self.create_publisher(Image, '/drone/camera/image_raw', 10)
        self.output_publisher = self.create_publisher(String, '/drone/outputs', 10)
        self.cap = cv2.VideoCapture(0)  # Change to drone camera, e.g., 'rtsp://drone_ip:port/video'
        if not self.cap.isOpened():
            self.publish_output('Error: Failed to open camera!')
        # Timer for video publishing (~30 FPS)
        self.video_timer = self.create_timer(0.033, self.publish_video)
        self.get_logger().info('ROS2 node initialized for video and outputs')

    def publish_video(self):
        ret, frame = self.cap.read()
        if ret:
            try:
                img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                self.video_publisher.publish(img_msg)
            except Exception as e:
                self.publish_output(f'Error publishing video: {str(e)}')
        else:
            self.publish_output('Warning: Failed to capture video frame')

    def publish_output(self, message: str):
        logger.info(message)  # Print to terminal
        msg = String()
        msg.data = message
        self.output_publisher.publish(msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

# MAVSDK async functions for drone control
async def connect_drone():
    drone = System()
    await drone.connect(system_address=os.environ.get('DRONE_ADDRESS', 'udp://:14540'))  # Env var for flexibility
    async for state in drone.core.connection_state():
        if state.is_connected:
            logger.info("Connected to drone!")
            return drone
    raise Exception("Failed to connect to drone")

async def arm_drone(drone):
    try:
        await drone.action.arm()
        logger.info("Drone armed")
    except ActionError as e:
        logger.error(f"Arming failed: {e}")

async def takeoff_drone(drone, altitude: float):
    await arm_drone(drone)
    try:
        await drone.action.set_takeoff_altitude(altitude)
        await drone.action.takeoff()
        logger.info(f"Lifting off to {altitude}m")
    except ActionError as e:
        logger.error(f"Takeoff failed: {e}")

async def land_drone(drone):
    try:
        await drone.action.land()
        logger.info("Landing initiated")
    except ActionError as e:
        logger.error(f"Landing failed: {e}")

async def navigate_drone(drone, target_pose: dict):
    # Assuming target_pose has 'latitude', 'longitude', 'altitude', 'heading' (adapt as needed)
    try:
        lat = target_pose.get('latitude', 0.0)
        lon = target_pose.get('longitude', 0.0)
        alt = target_pose.get('altitude', 10.0)
        heading = target_pose.get('heading', 0.0)
        await drone.action.goto_location(lat, lon, alt, heading)
        logger.info(f"Navigating to pose: {target_pose}")
    except ActionError as e:
        logger.error(f"Navigation failed: {e}")

# Flask endpoints to receive commands from web UI (and trigger MAVSDK + publish outputs)
@app.route('/health', methods=['GET'])
def health_check():
    return jsonify({'status': 'healthy', 'service': 'nimbus-drone'})

@app.route('/drone/status', methods=['GET'])
def drone_status():
    return jsonify({
        'service': 'Drone/Robotics',
        'status': 'running',
        'features': ['Flight Control', 'Navigation', 'Hardware Interface']
    })

@app.route('/drone/takeoff', methods=['POST'])
def takeoff():
    data = request.get_json() or {}
    altitude = data.get('altitude', 2.0)
    ros_node.publish_output(f"Takeoff command received - altitude: {altitude}m")
    asyncio.run(takeoff_drone(ros_node.drone_system, altitude))
    return jsonify({
        'status': 'success',
        'message': f'Takeoff initiated to {altitude}m',
        'altitude': altitude
    })

@app.route('/drone/land', methods=['POST'])
def land():
    ros_node.publish_output("Land command received")
    asyncio.run(land_drone(ros_node.drone_system))
    return jsonify({
        'status': 'success',
        'message': 'Landing sequence initiated'
    })

@app.route('/drone/navigate', methods=['POST'])
def navigate():
    data = request.get_json() or {}
    target_pose = data.get('target_pose', {})
    ros_node.publish_output(f"Navigation command: {target_pose}")
    asyncio.run(navigate_drone(ros_node.drone_system, target_pose))
    return jsonify({
        'status': 'success',
        'message': 'Navigation started',
        'target': target_pose
    })

# Main function to start everything
def main():
    global ros_node
    # Start
    rclpy.init()
    # Connect to drone
    loop = asyncio.get_event_loop()
    drone_system = loop.run_until_complete(connect_drone())
    # Create ROS2 node with drone system
    ros_node = DroneROSNode(drone_system)
    # Run Flask in a separate thread (non-blocking)
    flask_thread = threading.Thread(target=app.run, kwargs={'host': '0.0.0.0', 'port': int(os.environ.get('PORT', 5004)), 'debug': False})
    flask_thread.start()
    # Spin ROS2 node
    try:
        rclpy.spin(ros_node)
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

