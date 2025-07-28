#!/usr/bin/env python3
"""
Camera Publisher for ROS2
Publishes Camo Studio camera feed to ROS2 topics via rosbridge
"""

import cv2
import time
import base64
import json
import websocket
import threading
import logging
from typing import Optional
import sys
import os

# Add project root to path
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(project_root)
from helpers.phone_camera import PhoneCamera

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class CameraPublisher:
    """
    Publishes camera feed to ROS2 via rosbridge websocket
    """
    
    def __init__(self, ros2_host: str = "localhost", ros2_port: int = 9090):
        self.ros2_host = ros2_host
        self.ros2_port = ros2_port
        self.ws_url = f"ws://{ros2_host}:{ros2_port}"
        
        # WebSocket connection
        self.ws = None
        self.ws_connected = False
        
        # Camera
        self.camera = None
        self.is_publishing = False
        
        # Stats
        self.frame_count = 0
        self.publish_count = 0
        self.start_time = None
        
        # Threading
        self.publish_thread = None
        self.ws_thread = None
        
    def connect_to_ros2(self) -> bool:
        """Connect to ROS2 via rosbridge websocket"""
        try:
            logger.info(f"Connecting to ROS2 at {self.ws_url}")
            
            self.ws = websocket.WebSocketApp(
                self.ws_url,
                on_open=self._on_ws_open,
                on_message=self._on_ws_message,
                on_error=self._on_ws_error,
                on_close=self._on_ws_close
            )
            
            # Start WebSocket in separate thread
            self.ws_thread = threading.Thread(target=self.ws.run_forever, daemon=True)
            self.ws_thread.start()
            
            # Wait for connection
            timeout = 10
            while not self.ws_connected and timeout > 0:
                time.sleep(0.1)
                timeout -= 0.1
            
            if self.ws_connected:
                logger.info("Connected to ROS2 rosbridge")
                self._advertise_topics()
                return True
            else:
                logger.error("Failed to connect to ROS2 rosbridge")
                return False
                
        except Exception as e:
            logger.error(f"Error connecting to ROS2: {e}")
            return False
    
    def _on_ws_open(self, ws):
        """WebSocket connection opened"""
        self.ws_connected = True
        logger.info("WebSocket connection opened")
    
    def _on_ws_message(self, ws, message):
        """Handle WebSocket messages"""
        try:
            msg = json.loads(message)
            # Handle ROS2 messages/responses here if needed
            logger.debug(f"ROS2 message: {msg}")
        except json.JSONDecodeError:
            logger.warning(f"Invalid JSON message: {message}")
    
    def _on_ws_error(self, ws, error):
        """WebSocket error"""
        logger.error(f"WebSocket error: {error}")
        self.ws_connected = False
    
    def _on_ws_close(self, ws, close_status_code, close_msg):
        """WebSocket connection closed"""
        logger.info("WebSocket connection closed")
        self.ws_connected = False
    
    def _advertise_topics(self):
        """Advertise ROS2 topics"""
        # Advertise camera image topic
        advertise_msg = {
            "op": "advertise",
            "topic": "/nimbus/camera/image_raw",
            "type": "sensor_msgs/Image"
        }
        self._send_ros_message(advertise_msg)
        
        # Advertise camera info topic
        advertise_info_msg = {
            "op": "advertise", 
            "topic": "/nimbus/camera/camera_info",
            "type": "sensor_msgs/CameraInfo"
        }
        self._send_ros_message(advertise_info_msg)
        
        logger.info("Topics advertised to ROS2")
    
    def _send_ros_message(self, message: dict):
        """Send message to ROS2 via websocket"""
        if self.ws_connected and self.ws:
            try:
                self.ws.send(json.dumps(message))
            except Exception as e:
                logger.error(f"Error sending ROS message: {e}")
    
    def setup_camera(self) -> bool:
        """Set up Camo Studio camera"""
        logger.info("Setting up Camo Studio camera...")
        
        self.camera = PhoneCamera()
        success = self.camera.connect_usb_camera(0)  # Camera 0 = Camo Studio
        
        if success:
            info = self.camera.get_camera_info()
            logger.info(f"Camera connected: {info['width']}x{info['height']} @ {info['fps']}fps")
            return True
        else:
            logger.error("Failed to connect to Camo Studio")
            return False
    
    def start_publishing(self):
        """Start publishing camera feed to ROS2"""
        if not self.ws_connected:
            logger.error("Not connected to ROS2")
            return False
        
        if not self.camera:
            logger.error("Camera not set up")
            return False
        
        logger.info("Starting camera publishing to ROS2...")
        self.is_publishing = True
        self.start_time = time.time()
        
        # Start publishing thread
        self.publish_thread = threading.Thread(target=self._publish_loop, daemon=True)
        self.publish_thread.start()
        
        return True
    
    def _publish_loop(self):
        """Main publishing loop"""
        logger.info("Camera publishing loop started")
        
        while self.is_publishing and self.ws_connected:
            try:
                # Get frame from camera
                ret, frame = self.camera.get_frame()
                if not ret or frame is None:
                    logger.warning("Failed to get camera frame")
                    time.sleep(0.1)
                    continue
                
                self.frame_count += 1
                
                # Publish every 3rd frame to avoid overwhelming ROS2
                if self.frame_count % 3 == 0:
                    self._publish_frame(frame)
                    self.publish_count += 1
                
                # Control frame rate (10 FPS for ROS2)
                time.sleep(0.1)
                
            except Exception as e:
                logger.error(f"Error in publish loop: {e}")
                time.sleep(1)
    
    def _publish_frame(self, frame):
        """Publish single frame to ROS2"""
        try:
            # Convert frame to ROS2 Image message
            height, width, channels = frame.shape
            
            # Encode frame as JPEG for efficiency
            _, encoded_frame = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            frame_data = base64.b64encode(encoded_frame).decode('utf-8')
            
            # Create ROS2 Image message
            timestamp = time.time()
            
            ros_image_msg = {
                "op": "publish",
                "topic": "/nimbus/camera/image_raw",
                "msg": {
                    "header": {
                        "stamp": {
                            "sec": int(timestamp),
                            "nanosec": int((timestamp % 1) * 1e9)
                        },
                        "frame_id": "camera_frame"
                    },
                    "height": height,
                    "width": width,
                    "encoding": "jpeg",  # Using JPEG encoding for efficiency
                    "is_bigendian": False,
                    "step": len(encoded_frame),
                    "data": frame_data
                }
            }
            
            # Send to ROS2
            self._send_ros_message(ros_image_msg)
            
            # Also publish camera info
            self._publish_camera_info(timestamp)
            
        except Exception as e:
            logger.error(f"Error publishing frame: {e}")
    
    def _publish_camera_info(self, timestamp: float):
        """Publish camera info to ROS2"""
        if not self.camera:
            return
        
        info = self.camera.get_camera_info()
        
        camera_info_msg = {
            "op": "publish",
            "topic": "/nimbus/camera/camera_info", 
            "msg": {
                "header": {
                    "stamp": {
                        "sec": int(timestamp),
                        "nanosec": int((timestamp % 1) * 1e9)
                    },
                    "frame_id": "camera_frame"
                },
                "height": info['height'],
                "width": info['width'],
                "distortion_model": "plumb_bob",
                "d": [0.0, 0.0, 0.0, 0.0, 0.0],  # No distortion for now
                "k": [info['width']/2, 0.0, info['width']/2,   # Simple camera matrix
                      0.0, info['height']/2, info['height']/2,
                      0.0, 0.0, 1.0],
                "r": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],  # Identity
                "p": [info['width']/2, 0.0, info['width']/2, 0.0,      # Projection matrix
                      0.0, info['height']/2, info['height']/2, 0.0,
                      0.0, 0.0, 1.0, 0.0]
            }
        }
        
        self._send_ros_message(camera_info_msg)
    
    def stop_publishing(self):
        """Stop publishing camera feed"""
        logger.info("Stopping camera publishing...")
        self.is_publishing = False
        
        if self.publish_thread and self.publish_thread.is_alive():
            self.publish_thread.join(timeout=5)
        
        # Show stats
        if self.start_time:
            elapsed = time.time() - self.start_time
            logger.info(f"Published {self.publish_count} frames in {elapsed:.1f}s")
    
    def cleanup(self):
        """Clean up resources"""
        logger.info("Cleaning up camera publisher...")
        
        self.stop_publishing()
        
        if self.camera:
            self.camera.disconnect()
        
        if self.ws:
            self.ws.close()
        
        logger.info("Cleanup completed")

def main():
    """Main entry point"""
    publisher = CameraPublisher()
    
    try:
        # Set up camera
        if not publisher.setup_camera():
            return
        
        # Connect to ROS2
        if not publisher.connect_to_ros2():
            return
        
        # Start publishing
        if not publisher.start_publishing():
            return
        
        logger.info("Camera publisher running... Press Ctrl+C to stop")
        
        # Keep running
        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:
        logger.info("Shutting down camera publisher...")
    finally:
        publisher.cleanup()

if __name__ == "__main__":
    main()