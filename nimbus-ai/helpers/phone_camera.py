"""
Phone Camera Interface
Handles phone webcam input for the Nimbus AI system
Supports various phone webcam apps and IP camera sources
"""

import cv2
import numpy as np
import logging
from typing import Optional, Tuple, Dict
import threading
import time
from datetime import datetime

logger = logging.getLogger(__name__)

class PhoneCamera:
    """
    Phone webcam interface for Nimbus AI
    Supports multiple connection methods for phone cameras
    """
    
    def __init__(self):
        self.cap = None
        self.is_connected = False
        self.is_streaming = False
        self.current_frame = None
        self.frame_count = 0
        self.connection_type = None
        
        # Threading for continuous capture
        self._capture_thread = None
        self._lock = threading.Lock()
        
        # Camera info
        self.width = 0
        self.height = 0
        self.fps = 0
        
    def connect_ip_camera(self, ip_url: str) -> bool:
        """
        Connect to phone via IP camera app
        
        Common phone camera apps:
        - DroidCam: http://192.168.1.xxx:4747/video
        - IP Webcam: http://192.168.1.xxx:8080/video
        - iVCam: Various URLs
        
        Args:
            ip_url: IP camera URL from phone app
            
        Returns:
            bool: True if connected successfully
        """
        try:
            logger.info(f"Connecting to IP camera: {ip_url}")
            self.cap = cv2.VideoCapture(ip_url)
            
            if not self.cap.isOpened():
                logger.error("Failed to open IP camera stream")
                return False
            
            # Test frame capture
            ret, frame = self.cap.read()
            if not ret:
                logger.error("Failed to read frame from IP camera")
                self.cap.release()
                return False
            
            # Get camera properties
            self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            self.fps = int(self.cap.get(cv2.CAP_PROP_FPS))
            
            self.is_connected = True
            self.connection_type = "ip_camera"
            
            logger.info(f"IP camera connected: {self.width}x{self.height} @ {self.fps}fps")
            return True
            
        except Exception as e:
            logger.error(f"Error connecting to IP camera: {e}")
            return False
    
    def connect_usb_camera(self, device_id: int = 0) -> bool:
        """
        Connect to phone via USB (if supported)
        
        Args:
            device_id: USB camera device ID
            
        Returns:
            bool: True if connected successfully
        """
        try:
            logger.info(f"Connecting to USB camera: device {device_id}")
            self.cap = cv2.VideoCapture(device_id)
            
            if not self.cap.isOpened():
                logger.error(f"Failed to open USB camera device {device_id}")
                return False
            
            # Test frame capture
            ret, frame = self.cap.read()
            if not ret:
                logger.error("Failed to read frame from USB camera")
                self.cap.release()
                return False
            
            # Get camera properties
            self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            self.fps = int(self.cap.get(cv2.CAP_PROP_FPS))
            
            self.is_connected = True
            self.connection_type = "usb_camera"
            
            logger.info(f"USB camera connected: {self.width}x{self.height} @ {self.fps}fps")
            return True
            
        except Exception as e:
            logger.error(f"Error connecting to USB camera: {e}")
            return False
    
    def auto_connect(self) -> bool:
        """
        Auto-detect and connect to available camera
        Tries common IP camera URLs and USB devices
        
        Returns:
            bool: True if any camera connected
        """
        # Common IP camera URLs to try
        common_urls = [
            "http://192.168.1.100:4747/video",  # DroidCam default
            "http://192.168.1.100:8080/video",  # IP Webcam default
            "http://192.168.0.100:4747/video",  # Alternative subnet
            "http://192.168.0.100:8080/video",
        ]
        
        # Try IP cameras first
        for url in common_urls:
            if self.connect_ip_camera(url):
                logger.info(f"Auto-connected to IP camera: {url}")
                return True
        
        # Try USB cameras
        for device_id in range(3):  # Try first 3 USB devices
            if self.connect_usb_camera(device_id):
                logger.info(f"Auto-connected to USB camera: device {device_id}")
                return True
        
        logger.error("Auto-connect failed: No cameras found")
        return False
    
    def start_streaming(self) -> bool:
        """
        Start continuous frame capture in background thread
        
        Returns:
            bool: True if streaming started
        """
        if not self.is_connected:
            logger.error("Cannot start streaming: Camera not connected")
            return False
        
        if self.is_streaming:
            logger.warning("Streaming already active")
            return True
        
        try:
            self.is_streaming = True
            self._capture_thread = threading.Thread(
                target=self._capture_loop,
                daemon=True
            )
            self._capture_thread.start()
            logger.info("Camera streaming started")
            return True
            
        except Exception as e:
            logger.error(f"Failed to start streaming: {e}")
            self.is_streaming = False
            return False
    
    def stop_streaming(self):
        """Stop continuous frame capture"""
        self.is_streaming = False
        if self._capture_thread and self._capture_thread.is_alive():
            self._capture_thread.join(timeout=5)
        logger.info("Camera streaming stopped")
    
    def get_frame(self) -> Optional[Tuple[bool, np.ndarray]]:
        """
        Get single frame from camera
        
        Returns:
            Tuple: (success, frame) or (False, None) if failed
        """
        if not self.is_connected:
            return False, None
        
        try:
            ret, frame = self.cap.read()
            if ret:
                with self._lock:
                    self.frame_count += 1
                    self.current_frame = frame.copy()
                return ret, frame
            else:
                logger.warning("Failed to capture frame")
                return False, None
                
        except Exception as e:
            logger.error(f"Error capturing frame: {e}")
            return False, None
    
    def get_latest_frame(self) -> Optional[np.ndarray]:
        """
        Get the latest frame from continuous streaming
        
        Returns:
            np.ndarray: Latest frame or None if not available
        """
        with self._lock:
            return self.current_frame.copy() if self.current_frame is not None else None
    
    def get_camera_info(self) -> Dict:
        """
        Get camera information
        
        Returns:
            Dict: Camera properties and status
        """
        return {
            'connected': self.is_connected,
            'streaming': self.is_streaming,
            'connection_type': self.connection_type,
            'width': self.width,
            'height': self.height,
            'fps': self.fps,
            'frame_count': self.frame_count
        }
    
    def disconnect(self):
        """Disconnect from camera and cleanup"""
        self.stop_streaming()
        
        if self.cap:
            self.cap.release()
            self.cap = None
        
        self.is_connected = False
        self.current_frame = None
        self.frame_count = 0
        
        logger.info("Camera disconnected")
    
    def _capture_loop(self):
        """Continuous frame capture loop"""
        while self.is_streaming and self.is_connected:
            try:
                ret, frame = self.cap.read()
                if ret:
                    with self._lock:
                        self.frame_count += 1
                        self.current_frame = frame.copy()
                else:
                    logger.warning("Frame capture failed in streaming loop")
                    time.sleep(0.1)
                
                # Control capture rate (don't overwhelm the system)
                time.sleep(1/30)  # ~30 FPS max
                
            except Exception as e:
                logger.error(f"Error in capture loop: {e}")
                time.sleep(1)
        
        logger.info("Capture loop ended")

# Convenience functions for easy integration
def create_phone_camera(ip_url: Optional[str] = None) -> PhoneCamera:
    """
    Create and connect phone camera
    
    Args:
        ip_url: Specific IP camera URL, or None for auto-detection
        
    Returns:
        PhoneCamera: Connected camera instance
    """
    camera = PhoneCamera()
    
    if ip_url:
        success = camera.connect_ip_camera(ip_url)
    else:
        success = camera.auto_connect()
    
    if success:
        logger.info("Phone camera created and connected")
        return camera
    else:
        logger.error("Failed to create phone camera connection")
        return camera  # Return anyway, user can try manual connection

def get_common_phone_camera_urls() -> list:
    """
    Get list of common phone camera app URLs to try
    
    Returns:
        list: Common IP camera URLs
    """
    return [
        # DroidCam
        "http://192.168.1.100:4747/video",
        "http://192.168.0.100:4747/video",
        "http://10.0.0.100:4747/video",
        
        # IP Webcam
        "http://192.168.1.100:8080/video",
        "http://192.168.0.100:8080/video",
        "http://10.0.0.100:8080/video",
        
        # Alternative ports
        "http://192.168.1.100:8081/video",
        "http://192.168.1.100:9000/video",
    ]