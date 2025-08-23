"""
ROS2 Connection Service
Handles WebSocket connection to ROS2 rosbridge and camera feed
"""

import threading
import time
import json
import base64
import logging
import numpy as np
import cv2
import websocket

logger = logging.getLogger(__name__)

class ROS2Service:
    def __init__(self):
        self.app = None
        self.thread = None
        self.running = False
        
        # WebSocket connection
        self.ws = None
        self.ws_connected = False
        
        # Statistics
        self.frames_received = 0
        self.start_time = None
        
    def start_service(self, app):
        """Start the ROS2 connection service"""
        self.app = app
        self.running = True
        
        # Start connection thread
        self.thread = threading.Thread(target=self._connection_loop, daemon=True)
        self.thread.start()
        
        app.config['ROS2_SERVICE_RUNNING'] = True
        logger.info("ROS2 service started")
        
    def stop_service(self):
        """Stop the ROS2 connection service"""
        self.running = False
        
        # Close WebSocket connection
        if self.ws:
            self.ws.close()
        
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=2)
        
        if self.app:
            self.app.config['ROS2_SERVICE_RUNNING'] = False
        logger.info("ROS2 service stopped")
    
    def _connection_loop(self):
        """Main connection loop with reconnection logic"""
        while self.running:
            try:
                if not self.ws_connected:
                    self._connect_to_ros2()
                time.sleep(1)  # Check connection every second
            except Exception as e:
                logger.error(f"ROS2 connection loop error: {e}")
                time.sleep(5)  # Wait before retry
    
    def _connect_to_ros2(self):
        """Connect to ROS2 via rosbridge websocket"""
        if not self.app:
            return
            
        host = self.app.config['ROS2_HOST']
        port = self.app.config['ROS2_PORT']
        ws_url = f"ws://{host}:{port}"
        
        try:
            logger.info(f"Connecting to ROS2 at {ws_url}")
            
            self.ws = websocket.WebSocketApp(
                ws_url,
                on_open=self._on_ws_open,
                on_message=self._on_ws_message,
                on_error=self._on_ws_error,
                on_close=self._on_ws_close
            )
            
            # Run WebSocket (this blocks until connection closes)
            self.ws.run_forever()
            
        except Exception as e:
            logger.error(f"Error connecting to ROS2: {e}")
            time.sleep(5)  # Wait before retry
    
    def _on_ws_open(self, ws):
        """WebSocket connection opened"""
        self.ws_connected = True
        logger.info("WebSocket connection established")
        self._setup_subscriptions()
    
    def _on_ws_message(self, ws, message):
        """Handle WebSocket messages from ROS2"""
        try:
            msg = json.loads(message)
            
            # Handle camera image messages
            if msg.get('topic') == '/camera/image_raw':
                self._handle_camera_frame(msg)
            
        except json.JSONDecodeError:
            logger.debug("Invalid JSON message")
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
        """Subscribe to ROS2 camera topic"""
        subscribe_msg = {
            "op": "subscribe",
            "topic": "/camera/image_raw",
            "type": "sensor_msgs/Image"
        }
        self._send_ros_message(subscribe_msg)
        logger.info("Subscribed to /camera/image_raw")
    
    def _send_ros_message(self, message):
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
            
            # Extract and decode frame
            msg_data = ros_msg.get('msg', {})
            frame = self._decode_ros_image(msg_data)
            
            if frame is not None:
                # Send frame to AI service for processing
                from services.ai_service import add_frame_for_processing
                add_frame_for_processing(frame)
                
                # Send frame to display service
                from services.display_service import update_current_frame
                update_current_frame(frame)
                
                # Log stats periodically
                if self.frames_received % 30 == 0:
                    elapsed = time.time() - self.start_time
                    fps = self.frames_received / elapsed if elapsed > 0 else 0
                    logger.info(f"Received {self.frames_received} frames @ {fps:.1f} FPS")
            
        except Exception as e:
            logger.error(f"Error handling camera frame: {e}")
    
    def _decode_ros_image(self, image_msg):
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
                logger.debug(f"Unsupported encoding: {encoding}")
                return None
                
        except Exception as e:
            logger.error(f"Error decoding ROS image: {e}")
            return None
    
    def get_connection_status(self):
        """Get connection status info"""
        return {
            'connected': self.ws_connected,
            'frames_received': self.frames_received,
            'uptime': time.time() - self.start_time if self.start_time else 0
        }

# Global service instance
ros2_service_instance = ROS2Service()

def start_service(app):
    """Start the ROS2 service"""
    ros2_service_instance.start_service(app)

def stop_service():
    """Stop the ROS2 service"""
    ros2_service_instance.stop_service()

def get_connection_status():
    """Get connection status"""
    return ros2_service_instance.get_connection_status()