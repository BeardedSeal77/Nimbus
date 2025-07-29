#!/usr/bin/env python3
"""
AI.py - Nimbus AI Main System
Subscribes to ROS2 video feed, processes frames, and displays results
Pipeline: Camo -> ROS2 -> This Script -> Processing -> Display
"""

import cv2
import time
import base64
import json
import websocket
import threading
import logging
import numpy as np
import sys
import os
from typing import Optional, Dict, Any
from datetime import datetime

# Add project paths
project_root = os.path.dirname(os.path.abspath(__file__))
sys.path.append(project_root)

# Import nimbus-ai helpers and scripts
from helpers.display import ProcessedDisplay

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class NimbusAISystem:
    """
    Main Nimbus AI System
    Subscribes to ROS2 video feed and displays processed frames
    """
    
    def __init__(self, ros2_host: str = "localhost", ros2_port: int = 9090):
        self.ros2_host = ros2_host
        self.ros2_port = ros2_port
        self.ws_url = f"ws://{ros2_host}:{ros2_port}"
        
        # WebSocket connection
        self.ws = None
        self.ws_connected = False
        
        # Display system
        self.display = ProcessedDisplay()
        
        # Processing stats
        self.frames_received = 0
        self.frames_displayed = 0
        self.start_time = None
        
        # Current frame data
        self.current_frame = None
        self.current_ai_result = {}
        
        # Threading
        self.ws_thread = None
        self.is_running = False
        self.display_thread = None
        
        # AI processing placeholder (we'll add real processing later)
        self.ai_processing_enabled = False
        
    def connect_to_ros2(self) -> bool:
        """Connect to ROS2 via rosbridge websocket"""
        try:
            logger.info(f"🔗 Connecting to ROS2 at {self.ws_url}")
            
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
                logger.info("✅ Connected to ROS2 rosbridge")
                self._setup_subscriptions()
                return True
            else:
                logger.error("❌ Failed to connect to ROS2 rosbridge")
                return False
                
        except Exception as e:
            logger.error(f"❌ Error connecting to ROS2: {e}")
            return False
    
    def _on_ws_open(self, ws):
        """WebSocket connection opened"""
        self.ws_connected = True
        logger.info("🔗 WebSocket connection established")
    
    def _on_ws_message(self, ws, message):
        """Handle WebSocket messages from ROS2"""
        try:
            msg = json.loads(message)
            
            # Handle camera image messages
            if msg.get('topic') == '/camera/image_raw':
                self._handle_camera_frame(msg)
            
        except json.JSONDecodeError:
            logger.debug(f"Invalid JSON message")
        except Exception as e:
            logger.error(f"Error handling ROS2 message: {e}")
    
    def _on_ws_error(self, ws, error):
        """WebSocket error"""
        logger.error(f"❌ WebSocket error: {error}")
        self.ws_connected = False
    
    def _on_ws_close(self, ws, close_status_code, close_msg):
        """WebSocket connection closed"""
        logger.info("🔌 WebSocket connection closed")
        self.ws_connected = False
    
    def _setup_subscriptions(self):
        """Subscribe to ROS2 camera topic"""
        subscribe_msg = {
            "op": "subscribe",
            "topic": "/camera/image_raw",
            "type": "sensor_msgs/Image"
        }
        self._send_ros_message(subscribe_msg)
        logger.info("📡 Subscribed to /camera/image_raw")
    
    def _send_ros_message(self, message: dict):
        """Send message to ROS2 via websocket"""
        if self.ws_connected and self.ws:
            try:
                self.ws.send(json.dumps(message))
            except Exception as e:
                logger.error(f"❌ Error sending ROS message: {e}")
    
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
                # Process frame (placeholder for now)
                ai_result = self._process_frame(frame)
                
                # Update current frame data for display thread
                self.current_frame = frame.copy()
                self.current_ai_result = ai_result
                
                # Log stats periodically
                if self.frames_received % 30 == 0:
                    elapsed = time.time() - self.start_time
                    fps = self.frames_received / elapsed if elapsed > 0 else 0
                    logger.info(f"📊 Received {self.frames_received} frames @ {fps:.1f} FPS")
            
        except Exception as e:
            logger.error(f"❌ Error handling camera frame: {e}")
    
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
                logger.debug(f"Unsupported encoding: {encoding}")
                return None
                
        except Exception as e:
            logger.error(f"❌ Error decoding ROS image: {e}")
            return None
    
    def _process_frame(self, frame: np.ndarray) -> Dict[str, Any]:
        """
        Process frame through AI pipeline
        Currently a placeholder - will add real AI processing later
        """
        # Placeholder AI processing result
        ai_result = {
            'camera_pose': None,
            'bounding_box': None,
            'object_pose': None,
            'processing_status': 'pass_through',
            'frame_shape': frame.shape,
            'timestamp': datetime.now()
        }
        
        # TODO: Add real AI processing here
        # This is where we'll integrate:
        # - SLAM processing
        # - Object detection
        # - Depth estimation
        # - Intent processing
        
        return ai_result
    
    def _display_loop(self):
        """Main display loop - runs in separate thread"""
        logger.info("🖥️  Starting display loop...")
        
        while self.is_running:
            try:
                if self.current_frame is not None:
                    # Create display frame with overlays
                    elapsed = time.time() - self.start_time if self.start_time else 0
                    display_frame = self.display.create_display_frame(
                        self.current_frame, 
                        self.current_ai_result, 
                        self.frames_displayed, 
                        elapsed
                    )
                    
                    # Show frame
                    cv2.imshow('Nimbus AI - Live Video Feed', display_frame)
                    
                    self.frames_displayed += 1
                    
                    # Handle key presses
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'):
                        logger.info("🛑 Quit requested by user")
                        self.stop()
                        break
                    elif key == ord('s'):
                        logger.info("📊 Stats - Frames received: {}, displayed: {}".format(
                            self.frames_received, self.frames_displayed))
                    elif key == ord('r'):
                        logger.info("🔄 Reset requested")
                        self.frames_received = 0
                        self.frames_displayed = 0
                        self.start_time = time.time()
                
                time.sleep(0.033)  # ~30 FPS display rate
                
            except Exception as e:
                logger.error(f"❌ Display loop error: {e}")
                time.sleep(0.1)
        
        cv2.destroyAllWindows()
        logger.info("🖥️  Display loop stopped")
    
    def start(self):
        """Start the AI system"""
        logger.info("=" * 60)
        logger.info("🚀 NIMBUS AI SYSTEM STARTING")
        logger.info("=" * 60)
        logger.info("Pipeline: Camo -> ROS2 -> AI.py -> Processing -> Display")
        logger.info("Currently in pass-through mode (no AI processing)")
        logger.info("")
        
        # Connect to ROS2
        if not self.connect_to_ros2():
            logger.error("❌ Failed to connect to ROS2")
            return False
        
        # Start display thread
        self.is_running = True
        self.display_thread = threading.Thread(target=self._display_loop, daemon=True)
        self.display_thread.start()
        
        logger.info("✅ Nimbus AI System running")
        logger.info("📹 Waiting for video feed from ROS2...")
        logger.info("🎮 Controls: 'q' = quit, 's' = stats, 'r' = reset")
        logger.info("")
        
        return True
    
    def stop(self):
        """Stop the AI system"""
        logger.info("🛑 Stopping Nimbus AI System...")
        
        self.is_running = False
        
        # Close WebSocket connection
        if self.ws:
            self.ws.close()
        
        # Wait for threads to finish
        if self.display_thread and self.display_thread.is_alive():
            self.display_thread.join(timeout=2)
        
        # Show final stats
        if self.start_time:
            elapsed = time.time() - self.start_time
            logger.info(f"📊 Final Stats:")
            logger.info(f"   Frames received: {self.frames_received}")
            logger.info(f"   Frames displayed: {self.frames_displayed}")
            logger.info(f"   Runtime: {elapsed:.1f}s")
            if elapsed > 0:
                logger.info(f"   Average FPS: {self.frames_received / elapsed:.1f}")
        
        logger.info("✅ Nimbus AI System stopped")
    
    def run(self):
        """Main run method - keeps system alive"""
        try:
            if not self.start():
                return
            
            # Keep main thread alive
            while self.is_running and self.ws_connected:
                time.sleep(1)
                
        except KeyboardInterrupt:
            logger.info("🛑 Keyboard interrupt received")
        finally:
            self.stop()

def main():
    """Main entry point following AI-old.py pattern"""
    logger.info("🤖 Initializing Nimbus AI System...")
    
    # Initialize AI system
    ai_system = NimbusAISystem()
    
    try:
        # Run the system
        ai_system.run()
        
    except Exception as e:
        logger.error(f"❌ Fatal error: {e}")
    finally:
        logger.info("👋 Nimbus AI System shutdown complete")

if __name__ == "__main__":
    main()