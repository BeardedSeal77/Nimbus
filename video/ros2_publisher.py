"""
ROS2 Publisher Module
Publishes video frames to ROS2 container via websocket
"""

import cv2
import time
import base64
import json
import websocket
import threading
import logging
import numpy as np
from typing import Optional, Dict
from datetime import datetime
from config import PERFORMANCE_SETTINGS

logger = logging.getLogger(__name__)

class ROS2VideoPublisher:
    """
    ROS2 video publisher that sends frames to ROS2 container via websocket
    """
    
    def __init__(self, ros2_host: str = "localhost", ros2_port: int = 9090):
        self.ros2_host = ros2_host
        self.ros2_port = ros2_port
        self.ws_url = f"ws://{ros2_host}:{ros2_port}"
        
        # WebSocket connection
        self.ws = None
        self.ws_connected = False
        self.connection_thread = None
        
        # Publishing state
        self.is_publishing = False
        self.frames_published = 0
        self.publish_start_time = None
        
        # Performance tracking
        self.last_publish_time = 0
        self.publish_errors = 0
        self.connection_attempts = 0
        
        # Settings optimized for performance
        self.jpeg_quality = 70  # Reduce quality for speed
        self.publish_rate = PERFORMANCE_SETTINGS['max_fps']
        
    def connect(self) -> bool:
        """
        Connect to ROS2 via websocket
        """
        self.connection_attempts += 1
        
        try:
            self.ws = websocket.WebSocketApp(
                self.ws_url,
                on_open=self._on_ws_open,
                on_message=self._on_ws_message,
                on_error=self._on_ws_error,
                on_close=self._on_ws_close
            )
            
            # Start WebSocket in background thread
            self.connection_thread = threading.Thread(target=self.ws.run_forever, daemon=True)
            self.connection_thread.start()
            
            # Wait for connection
            timeout = 10
            while not self.ws_connected and timeout > 0:
                time.sleep(0.1)
                timeout -= 0.1
            
            if self.ws_connected:
                logger.info("Connected to ROS2 rosbridge")
                self._advertise_topic()
                return True
            else:
                logger.error("Failed to connect to ROS2 rosbridge")
                return False
                
        except Exception as e:
            return False
    
    def _on_ws_open(self, ws):
        """WebSocket connection opened"""
        self.ws_connected = True
    
    def _on_ws_message(self, ws, message):
        """Handle WebSocket messages from ROS2"""
        try:
            msg = json.loads(message)
            # Handle any ROS2 responses if needed (logging removed for performance)
        except json.JSONDecodeError:
            pass  # Skip logging for performance
        except Exception as e:
            pass  # Skip logging for performance
    
    def _on_ws_error(self, ws, error):
        """WebSocket error"""
        self.ws_connected = False
    
    def _on_ws_close(self, ws, close_status_code, close_msg):
        """WebSocket connection closed"""
        self.ws_connected = False
    
    def _advertise_topic(self):
        """Advertise the camera topic to ROS2"""
        advertise_msg = {
            "op": "advertise",
            "topic": "/camera/image_raw",
            "type": "sensor_msgs/Image"
        }
        self._send_ros_message(advertise_msg)
        logger.info("Camera topic advertised: /camera/image_raw")
    
    def _send_ros_message(self, message: dict):
        """Send message to ROS2 via websocket"""
        if self.ws_connected and self.ws:
            try:
                self.ws.send(json.dumps(message))
            except Exception as e:
                self.publish_errors += 1
    
    def start_publishing(self) -> bool:
        """
        Start publishing mode
        """
        if not self.ws_connected:
            logger.error("Cannot start publishing: not connected to ROS2")
            return False
        
        self.is_publishing = True
        self.publish_start_time = time.time()
        self.frames_published = 0
        
        logger.info("Started ROS2 video publishing")
        return True
    
    def stop_publishing(self):
        """
        Stop publishing mode
        """
        self.is_publishing = False
        
        # Stats available via get_stats() if needed
    
    def publish_frame(self, frame: np.ndarray) -> bool:
        """
        Publish a single frame to ROS2
        
        Args:
            frame: OpenCV frame to publish
            
        Returns:
            bool: True if published successfully
        """
        if not self.is_publishing or not self.ws_connected:
            return False
        
        # Simplified rate limiting
        current_time = time.time()
        min_interval = 1.0 / self.publish_rate
        if current_time - self.last_publish_time < min_interval:
            return True  # Skip this frame to maintain rate
        
        try:
            # Encode frame as JPEG
            encode_param = [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
            _, encoded_frame = cv2.imencode('.jpg', frame, encode_param)
            frame_data = base64.b64encode(encoded_frame).decode('utf-8')
            
            # Get frame properties
            height, width = frame.shape[:2]
            timestamp = current_time
            
            # Create ROS2 Image message
            ros_image_msg = {
                "op": "publish",
                "topic": "/camera/image_raw",
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
                    "encoding": "jpeg",
                    "is_bigendian": 0,
                    "step": len(encoded_frame),
                    "data": frame_data
                }
            }
            
            # Send to ROS2
            self._send_ros_message(ros_image_msg)
            
            # Update statistics
            self.frames_published += 1
            self.last_publish_time = current_time
            
            # No logging for maximum performance
            
            return True
            
        except Exception as e:
            self.publish_errors += 1
            return False
    
    def get_stats(self) -> Dict:
        """
        Get publishing statistics
        """
        elapsed = 0
        fps = 0
        
        if self.publish_start_time:
            elapsed = time.time() - self.publish_start_time
            fps = self.frames_published / elapsed if elapsed > 0 else 0
        
        return {
            'connected': self.ws_connected,
            'publishing': self.is_publishing,
            'frames_published': self.frames_published,
            'publish_errors': self.publish_errors,
            'connection_attempts': self.connection_attempts,
            'publish_fps': fps,
            'uptime': elapsed,
            'target_fps': self.publish_rate,
            'jpeg_quality': self.jpeg_quality
        }
    
    def disconnect(self):
        """
        Disconnect from ROS2 and cleanup
        """
        # Stop publishing
        self.stop_publishing()
        
        # Close WebSocket
        if self.ws:
            self.ws.close()
        
        # Wait for connection thread
        if self.connection_thread and self.connection_thread.is_alive():
            self.connection_thread.join(timeout=2.0)
        
        self.ws_connected = False
    
    def set_quality(self, quality: int):
        """
        Set JPEG compression quality (1-100)
        """
        self.jpeg_quality = max(1, min(100, quality))
    
    def set_publish_rate(self, fps: float):
        """
        Set target publishing rate
        """
        self.publish_rate = max(1, min(60, fps))
    
    def is_ready(self) -> bool:
        """
        Check if publisher is ready to publish frames
        """
        return self.ws_connected and self.is_publishing