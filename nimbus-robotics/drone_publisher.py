#!/usr/bin/env python3
"""
Drone Main Application
Handles drone control, flight operations, video capture, and ROS2 publishing for the Nimbus system.
"""

import rclpy
from rclpy.node import Node
import cv2
import base64
import websocket
import json
import time

ROS2_WS_URL = "ws://localhost:9090"
WIDTH = 640
HEIGHT = 480

class DroneVideoPublisher(Node):
    def __init__(self):
        super().__init__('drone_video_publisher')
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error('Failed to open camera!')
            return
        self.ws = websocket.WebSocketApp(
            ROS2_WS_URL,
            on_open=self.on_open,
            on_message=self.on_message,
            on_error=self.on_error,
            on_close=self.on_close
        )
        self.running = True
        self.ws_thread = self.ws.run_forever()

    def on_open(self, ws):
        self.get_logger().info('WebSocket opened, starting video stream...')

    def on_message(self, ws, message):
        pass  # Not needed for publisher

    def on_error(self, ws, error):
        self.get_logger().error(f'WebSocket error: {error}')

    def on_close(self, ws, close_status_code, close_msg):
        self.get_logger().info('WebSocket closed')
        self.running = False

    def run(self):
        while self.running:
            ret, frame = self.cap.read()
            if ret:
                # Resize and encode as JPEG
                frame = cv2.resize(frame, (WIDTH, HEIGHT))
                _, jpeg_buffer = cv2.imencode('.jpg', frame)
                jpeg_bytes = jpeg_buffer.tobytes()
                b64data = base64.b64encode(jpeg_bytes).decode('utf-8')

                publish_msg = {
                    "op": "publish",
                    "topic": "/drone_video",
                    "msg": {
                        "header": {"stamp": {"sec": int(time.time()), "nanosec": 0}},
                        "height": HEIGHT,
                        "width": WIDTH,
                        "encoding": "jpeg",
                        "is_bigendian": 0,
                        "step": 0,
                        "data": b64data
                    }
                }
                self.ws.send(json.dumps(publish_msg))
            time.sleep(0.033)  # ~30 FPS

    def destroy_node(self):
        self.cap.release()
        self.ws.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    publisher = DroneVideoPublisher()
    publisher.run()
    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()