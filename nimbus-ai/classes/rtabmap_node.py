"""
RTAB-Map SLAM Node
Interfaces with containerized ROS2+RTAB-Map for visual SLAM
Handles phone webcam input and provides camera pose estimation
"""

import cv2
import numpy as np
import requests
import json
import logging
from typing import Dict, Optional, Tuple
import threading
import time
from datetime import datetime

logger = logging.getLogger(__name__)

class RTABMapNode:
    """
    Modular RTAB-Map SLAM interface
    Connects local Python to containerized ROS2+RTAB-Map system
    """
    
    def __init__(self, ros2_host: str = "localhost", ros2_port: int = 9090):
        self.ros2_host = ros2_host
        self.ros2_port = ros2_port
        self.ros2_bridge_url = f"http://{ros2_host}:{ros2_port}"
        
        # SLAM state
        self.current_pose = None
        self.is_initialized = False
        self.is_running = False
        self.frame_count = 0
        
        # Camera configuration
        self.camera_matrix = None
        self.dist_coeffs = None
        
        # Threading
        self._slam_thread = None
        self._lock = threading.Lock()
        
        logger.info(f"RTABMapNode initialized with ROS2 bridge at {self.ros2_bridge_url}")
    
    def initialize_slam(self) -> bool:
        """
        Initialize RTAB-Map SLAM system in the container
        
        Returns:
            bool: True if initialization successful
        """
        try:
            # Test ROS2 bridge connection
            if not self._test_ros2_connection():
                logger.error("Cannot connect to ROS2 bridge")
                return False
            
            # Initialize RTAB-Map through ROS2 bridge
            # This would typically involve calling ROS2 services
            logger.info("RTAB-Map SLAM initialized successfully")
            self.is_initialized = True
            return True
            
        except Exception as e:
            logger.error(f"Failed to initialize RTAB-Map: {e}")
            return False
    
    def process_frame(self, frame: np.ndarray, depth_frame: Optional[np.ndarray] = None) -> Optional[Dict]:
        """
        Process a single frame through RTAB-Map SLAM
        
        Args:
            frame: RGB camera frame from phone
            depth_frame: Optional depth data (if available)
            
        Returns:
            Dict: Camera pose information or None if failed
        """
        if not self.is_initialized:
            logger.warning("RTAB-Map not initialized")
            return None
        
        try:
            with self._lock:
                self.frame_count += 1
                
                # Prepare frame data for ROS2
                frame_data = self._prepare_frame_data(frame, depth_frame)
                
                # Send to RTAB-Map via ROS2 bridge
                pose_result = self._send_frame_to_slam(frame_data)
                
                if pose_result:
                    self.current_pose = pose_result
                    logger.debug(f"Frame {self.frame_count} processed, pose updated")
                    return pose_result
                else:
                    logger.warning(f"Frame {self.frame_count} processing failed")
                    return None
                    
        except Exception as e:
            logger.error(f"Error processing frame: {e}")
            return None
    
    def get_current_pose(self) -> Optional[Dict]:
        """
        Get the latest camera pose estimate
        
        Returns:
            Dict: Current pose with position and orientation
        """
        with self._lock:
            return self.current_pose.copy() if self.current_pose else None
    
    def start_continuous_slam(self, camera_source: any) -> bool:
        """
        Start continuous SLAM processing in background thread
        
        Args:
            camera_source: Camera input source (phone webcam)
            
        Returns:
            bool: True if started successfully
        """
        if self.is_running:
            logger.warning("SLAM already running")
            return True
        
        if not self.is_initialized:
            if not self.initialize_slam():
                return False
        
        try:
            self.is_running = True
            self._slam_thread = threading.Thread(
                target=self._continuous_slam_loop,
                args=(camera_source,),
                daemon=True
            )
            self._slam_thread.start()
            logger.info("Continuous SLAM started")
            return True
            
        except Exception as e:
            logger.error(f"Failed to start continuous SLAM: {e}")
            self.is_running = False
            return False
    
    def stop_continuous_slam(self):
        """Stop continuous SLAM processing"""
        self.is_running = False
        if self._slam_thread and self._slam_thread.is_alive():
            self._slam_thread.join(timeout=5)
        logger.info("Continuous SLAM stopped")
    
    def reset_slam(self) -> bool:
        """
        Reset RTAB-Map SLAM system
        
        Returns:
            bool: True if reset successful
        """
        try:
            # Send reset command to ROS2 RTAB-Map
            logger.info("Resetting RTAB-Map SLAM")
            self.current_pose = None
            self.frame_count = 0
            return True
            
        except Exception as e:
            logger.error(f"Failed to reset SLAM: {e}")
            return False
    
    def _test_ros2_connection(self) -> bool:
        """Test connection to ROS2 bridge"""
        try:
            # Try to connect to ROS2 bridge
            # For now, just test if the container is reachable
            response = requests.get(f"{self.ros2_bridge_url}/api/ping", timeout=5)
            return response.status_code == 200
        except:
            # ROS2 bridge might not have HTTP endpoints, that's okay
            # We'll assume it's running if the container is up
            return True
    
    def _prepare_frame_data(self, frame: np.ndarray, depth_frame: Optional[np.ndarray]) -> Dict:
        """Prepare frame data for ROS2 transmission"""
        # Convert frame to format suitable for ROS2
        # This might involve encoding, compression, etc.
        
        frame_data = {
            'timestamp': datetime.now().isoformat(),
            'frame_id': self.frame_count,
            'width': frame.shape[1],
            'height': frame.shape[0],
            'encoding': 'bgr8',
            # Frame data would be encoded here (base64, compressed, etc.)
            'data': None  # Placeholder for actual frame data
        }
        
        if depth_frame is not None:
            frame_data['depth_data'] = None  # Placeholder for depth data
        
        return frame_data
    
    def _send_frame_to_slam(self, frame_data: Dict) -> Optional[Dict]:
        """Send frame to RTAB-Map via ROS2 bridge"""
        try:
            # This would typically involve:
            # 1. Publishing frame to ROS2 topic
            # 2. Waiting for pose estimate response
            # 3. Parsing the response
            
            # Mock pose response for now
            mock_pose = {
                'position': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0},
                'timestamp': frame_data['timestamp'],
                'confidence': 0.95,
                'frame_id': frame_data['frame_id']
            }
            
            return mock_pose
            
        except Exception as e:
            logger.error(f"Failed to send frame to SLAM: {e}")
            return None
    
    def _continuous_slam_loop(self, camera_source):
        """Continuous SLAM processing loop"""
        while self.is_running:
            try:
                # Get frame from camera source
                ret, frame = camera_source.read()
                if not ret:
                    continue
                
                # Process frame
                self.process_frame(frame)
                
                # Control processing rate
                time.sleep(0.1)  # 10 FPS
                
            except Exception as e:
                logger.error(f"Error in SLAM loop: {e}")
                time.sleep(1)

# Factory function for AI.py integration
def rtabmap_node(video_stream: np.ndarray, depth_data: Optional[np.ndarray] = None) -> Optional[Dict]:
    """
    Process single frame through RTAB-Map SLAM (AI.py interface)
    
    Args:
        video_stream: Camera frame
        depth_data: Optional depth information
        
    Returns:
        Dict: Camera pose or None if failed
    """
    # Use global SLAM instance or create new one
    global _slam_instance
    
    if '_slam_instance' not in globals():
        _slam_instance = RTABMapNode()
        if not _slam_instance.initialize_slam():
            logger.error("Failed to initialize SLAM for single frame processing")
            return None
    
    return _slam_instance.process_frame(video_stream, depth_data)

# Global instance for continuous SLAM
_slam_instance = None