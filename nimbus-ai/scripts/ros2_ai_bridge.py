#!/usr/bin/env python3
"""
ROS2 AI Bridge
Connects AI.py to ROS2 system via rosbridge websocket
Receives camera frames and publishes AI processing results
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
import sys
import os

# Add project root to path
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(project_root)

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class ROS2AIBridge:
    """
    Bridge between ROS2 and AI.py processing
    Subscribes to camera feed and publishes AI results
    """
    
    def __init__(self, ros2_host: str = "localhost", ros2_port: int = 9090):
        self.ros2_host = ros2_host  
        self.ros2_port = ros2_port
        self.ws_url = f"ws://{ros2_host}:{ros2_port}"
        
        # WebSocket connection
        self.ws = None
        self.ws_connected = False
        
        # AI System (will try to initialize)
        self.ai_system = None
        self.ai_available = False
        
        # Processing stats
        self.frames_received = 0
        self.frames_processed = 0
        self.start_time = None
        
        # Threading
        self.ws_thread = None
        self.is_running = False
        
    def initialize_ai_system(self):
        """Initialize AI system if dependencies are available"""
        try:
            from AI import get_ai_system
            self.ai_system = get_ai_system()
            self.ai_system.start_continuous_slam()
            self.ai_available = True
            logger.info("AI system initialized successfully")
            return True
        except ImportError as e:
            logger.warning(f"AI system not available: {e}")
            logger.info("Running in pass-through mode")
            self.ai_available = False
            return False
    
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
                self._setup_subscriptions()
                self._advertise_ai_topics()
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
        """Handle WebSocket messages from ROS2"""
        try:
            msg = json.loads(message)
            
            # Handle camera image messages
            if msg.get('topic') == '/nimbus/camera/image_raw':
                self._handle_camera_frame(msg)
            
            # Handle other ROS2 messages
            elif 'topic' in msg:
                logger.debug(f"ROS2 message on {msg['topic']}")
                
        except json.JSONDecodeError:
            logger.warning(f"Invalid JSON message: {message}")
        except Exception as e:
            logger.error(f"Error handling ROS2 message: {e}")
    
    def _on_ws_error(self, ws, error):
        """WebSocket error"""
        logger.error(f"WebSocket error: {error}")
        self.ws_connected = False
    
    def _on_ws_close(self, ws, close_status_code, close_msg):
        """WebSocket connection closed"""
        logger.info("WebSocket connection closed")
        self.ws_connected = False
    
    def _setup_subscriptions(self):
        """Subscribe to ROS2 topics"""
        # Subscribe to camera feed
        subscribe_msg = {
            "op": "subscribe",
            "topic": "/nimbus/camera/image_raw",
            "type": "sensor_msgs/Image"
        }
        self._send_ros_message(subscribe_msg)
        
        logger.info("Subscribed to camera feed")
    
    def _advertise_ai_topics(self):
        """Advertise AI result topics"""
        # Advertise AI processing results
        topics = [
            ("/nimbus/ai/slam_pose", "geometry_msgs/PoseStamped"),
            ("/nimbus/ai/object_detection", "vision_msgs/Detection2DArray"),
            ("/nimbus/ai/processing_status", "std_msgs/String"),
            ("/nimbus/ai/processed_image", "sensor_msgs/Image")
        ]
        
        for topic, msg_type in topics:
            advertise_msg = {
                "op": "advertise",
                "topic": topic,
                "type": msg_type
            }
            self._send_ros_message(advertise_msg)
        
        logger.info("AI result topics advertised")
    
    def _send_ros_message(self, message: dict):
        """Send message to ROS2 via websocket"""
        if self.ws_connected and self.ws:
            try:
                self.ws.send(json.dumps(message))
            except Exception as e:
                logger.error(f"Error sending ROS message: {e}")
    
    def _handle_camera_frame(self, ros_msg):
        """Handle incoming camera frame from ROS2"""
        try:
            self.frames_received += 1
            
            if not self.start_time:
                self.start_time = time.time()
            
            # Extract frame data
            msg_data = ros_msg.get('msg', {})
            
            # Decode image data
            frame = self._decode_ros_image(msg_data)
            if frame is None:
                logger.warning("Failed to decode camera frame")
                return
            
            # Process frame through AI system
            ai_result = self._process_frame_with_ai(frame)
            
            # Publish AI results back to ROS2
            self._publish_ai_results(ai_result, msg_data.get('header', {}))
            
            self.frames_processed += 1
            
            # Log stats periodically
            if self.frames_processed % 30 == 0:
                elapsed = time.time() - self.start_time
                fps = self.frames_processed / elapsed if elapsed > 0 else 0
                logger.info(f"AI Bridge: {self.frames_processed} frames processed @ {fps:.1f} FPS")
            
        except Exception as e:
            logger.error(f"Error handling camera frame: {e}")
    
    def _decode_ros_image(self, image_msg: dict) -> Optional[np.ndarray]:
        """Decode ROS Image message to OpenCV frame"""
        try:
            # Get image properties
            width = image_msg.get('width', 0)
            height = image_msg.get('height', 0)
            encoding = image_msg.get('encoding', '')
            data = image_msg.get('data', '')
            
            if not data:
                return None
            
            # Decode base64 data
            image_data = base64.b64decode(data)
            
            if encoding == 'jpeg':
                # Decode JPEG
                nparr = np.frombuffer(image_data, np.uint8)
                frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                return frame
            else:
                # Handle other encodings (BGR8, RGB8, etc.)
                logger.warning(f"Unsupported encoding: {encoding}")
                return None
                
        except Exception as e:
            logger.error(f"Error decoding ROS image: {e}")
            return None
    
    def _process_frame_with_ai(self, frame: np.ndarray) -> Dict:
        """Process frame through AI system"""
        if not self.ai_available or not self.ai_system:
            # No AI processing available, return empty result
            return {
                'camera_pose': None,
                'bounding_box': None,
                'object_pose': None,
                'processing_status': 'ai_not_available'
            }
        
        try:
            # Process through AI.py
            result = self.ai_system.process_frame(frame)
            
            # Get current state
            state = self.ai_system.get_state()
            
            return {
                'camera_pose': result.get('camera_pose'),
                'bounding_box': result.get('bounding_box'), 
                'object_pose': result.get('object_pose'),
                'processing_status': state.get('processing_status', 'unknown'),
                'frame_shape': frame.shape
            }
            
        except Exception as e:
            logger.error(f"AI processing error: {e}")
            return {
                'camera_pose': None,
                'bounding_box': None,
                'object_pose': None,
                'processing_status': f'error: {str(e)}'
            }
    
    def _publish_ai_results(self, ai_result: Dict, header: Dict):
        """Publish AI processing results to ROS2"""
        try:
            timestamp = time.time()
            
            # Publish SLAM pose if available
            camera_pose = ai_result.get('camera_pose')
            if camera_pose and 'position' in camera_pose:
                pose_msg = {
                    "op": "publish",
                    "topic": "/nimbus/ai/slam_pose",
                    "msg": {
                        "header": {
                            "stamp": {
                                "sec": int(timestamp),
                                "nanosec": int((timestamp % 1) * 1e9)
                            },
                            "frame_id": "camera_frame"
                        },
                        "pose": {
                            "position": camera_pose['position'],
                            "orientation": camera_pose.get('orientation', 
                                                        {"x": 0, "y": 0, "z": 0, "w": 1})
                        }
                    }
                }
                self._send_ros_message(pose_msg)
            
            # Publish object detection if available
            bounding_box = ai_result.get('bounding_box')
            if bounding_box:
                detection_msg = {
                    "op": "publish",
                    "topic": "/nimbus/ai/object_detection",
                    "msg": {
                        "header": {
                            "stamp": {
                                "sec": int(timestamp),
                                "nanosec": int((timestamp % 1) * 1e9)
                            },
                            "frame_id": "camera_frame"
                        },
                        "detections": [{
                            "bbox": {
                                "center": {
                                    "x": bounding_box.get('x', 0) + bounding_box.get('width', 0) / 2,
                                    "y": bounding_box.get('y', 0) + bounding_box.get('height', 0) / 2
                                },
                                "size_x": bounding_box.get('width', 0),
                                "size_y": bounding_box.get('height', 0)
                            },
                            "results": [{
                                "id": bounding_box.get('object_name', 'unknown'),
                                "score": bounding_box.get('confidence', 0.0)
                            }]
                        }]
                    }
                }
                self._send_ros_message(detection_msg)
            
            # Always publish processing status
            status_msg = {
                "op": "publish", 
                "topic": "/nimbus/ai/processing_status",
                "msg": {
                    "data": ai_result.get('processing_status', 'unknown')
                }
            }
            self._send_ros_message(status_msg)
            
        except Exception as e:
            logger.error(f"Error publishing AI results: {e}")
    
    def start_bridge(self):
        """Start the ROS2 AI bridge"""
        logger.info("Starting ROS2 AI Bridge...")
        
        # Initialize AI system
        self.initialize_ai_system()
        
        # Connect to ROS2
        if not self.connect_to_ros2():
            return False
        
        self.is_running = True
        self.start_time = time.time()
        
        logger.info("ROS2 AI Bridge running...")
        logger.info("Waiting for camera frames from ROS2...")
        
        return True
    
    def stop_bridge(self):
        """Stop the bridge"""
        logger.info("Stopping ROS2 AI Bridge...")
        
        self.is_running = False
        
        if self.ai_system and self.ai_available:
            self.ai_system.stop_continuous_slam()
        
        if self.ws:
            self.ws.close()
        
        # Show final stats
        if self.start_time:
            elapsed = time.time() - self.start_time
            logger.info(f"Bridge stats: {self.frames_received} frames received, "
                       f"{self.frames_processed} processed in {elapsed:.1f}s")
        
        logger.info("ROS2 AI Bridge stopped")

def main():
    """Main entry point"""
    bridge = ROS2AIBridge()
    
    try:
        if not bridge.start_bridge():
            return
        
        logger.info("ROS2 AI Bridge running... Press Ctrl+C to stop")
        
        # Keep running
        while bridge.is_running and bridge.ws_connected:
            time.sleep(1)
            
    except KeyboardInterrupt:
        logger.info("Shutting down ROS2 AI Bridge...")
    finally:
        bridge.stop_bridge()

if __name__ == "__main__":
    main()