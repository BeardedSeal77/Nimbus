"""
Video Capture Module
Handles connecting to and capturing frames from Camo Studio camera
"""

import cv2
import time
import threading
import logging
from typing import Optional, Tuple, Dict
import numpy as np
from config import CAMERA_SETTINGS, PERFORMANCE_SETTINGS

logger = logging.getLogger(__name__)

class CameraCapture:
    """
    Camera capture class that handles connection and frame retrieval
    """
    
    def __init__(self):
        self.cap = None
        self.is_connected = False
        self.is_capturing = False
        
        # Current frame data
        self.current_frame = None
        self.frame_count = 0
        self.dropped_frames = 0
        
        # Camera properties
        self.width = 0
        self.height = 0
        self.fps = 0
        self.backend_name = ""
        
        # Threading
        self.capture_thread = None
        self.frame_lock = threading.Lock()
        
        # Performance tracking
        self.last_frame_time = 0
        self.connection_start_time = 0
        
    def connect(self) -> bool:
        """
        Attempt to connect to Camo Studio camera
        Tries multiple backends and indices
        """
        logger.info("Attempting to connect to Camo Studio camera...")
        self.connection_start_time = time.time()
        
        # Get backend constants
        backends = []
        for backend_name in CAMERA_SETTINGS['backends']:
            try:
                backend_value = eval(backend_name)
                backends.append((backend_name, backend_value))
            except:
                logger.debug(f"Backend {backend_name} not available")
        
        # If no specific backends, try default
        if not backends:
            backends = [("DEFAULT", cv2.CAP_ANY)]
        
        # Try each combination of backend and index
        for backend_name, backend_value in backends:
            logger.info(f"Trying backend: {backend_name}")
            
            for index in CAMERA_SETTINGS['usb_indices']:
                logger.info(f"  Testing camera index {index}...")
                
                try:
                    # Create VideoCapture with specific backend
                    cap = cv2.VideoCapture(index, backend_value)
                    
                    if not cap.isOpened():
                        logger.debug(f"    Index {index} failed to open")
                        cap.release()
                        continue
                    
                    # Set buffer size to reduce latency
                    cap.set(cv2.CAP_PROP_BUFFERSIZE, PERFORMANCE_SETTINGS['buffer_size'])
                    
                    # Try to read a frame
                    ret, frame = cap.read()
                    if not ret or frame is None:
                        logger.debug(f"    Index {index} failed to read frame")
                        cap.release()
                        continue
                    
                    # Success! Store the connection
                    self.cap = cap
                    self.backend_name = backend_name
                    
                    # Get camera properties
                    self.width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                    self.height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                    self.fps = int(cap.get(cv2.CAP_PROP_FPS)) or 30  # Default to 30 if unknown
                    
                    # Optimize settings
                    self._optimize_camera_settings()
                    
                    self.is_connected = True
                    connection_time = time.time() - self.connection_start_time
                    
                    logger.info(f"Connected to camera!")
                    logger.info(f"   Backend: {backend_name}")
                    logger.info(f"   Index: {index}")
                    logger.info(f"   Resolution: {self.width}x{self.height}")
                    logger.info(f"   FPS: {self.fps}")
                    logger.info(f"   Connection time: {connection_time:.2f}s")
                    
                    return True
                    
                except Exception as e:
                    logger.debug(f"    Error with index {index}: {e}")
                    if 'cap' in locals():
                        cap.release()
                    continue
        
        logger.error("Failed to connect to any camera")
        return False
    
    def _optimize_camera_settings(self):
        """
        Optimize camera settings for best performance
        """
        if not self.cap:
            return
        
        # Try to set optimal resolution
        for target_width, target_height in CAMERA_SETTINGS['resolutions']:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, target_width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, target_height)
            
            actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            
            if actual_width == target_width and actual_height == target_height:
                self.width = actual_width
                self.height = actual_height
                logger.info(f"Set resolution to {target_width}x{target_height}")
                break
        
        # Try to set optimal FPS
        for target_fps in CAMERA_SETTINGS['fps_targets']:
            self.cap.set(cv2.CAP_PROP_FPS, target_fps)
            actual_fps = int(self.cap.get(cv2.CAP_PROP_FPS))
            
            if actual_fps >= target_fps * 0.8:  # Within 80% is good enough
                self.fps = actual_fps
                logger.info(f"Set FPS to {actual_fps}")
                break
        
        # Additional optimizations
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Minimize latency
        
        # Disable auto-exposure and auto-focus if possible (may not work on all cameras)
        try:
            self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)  # Manual exposure
        except:
            pass
            
        try:
            self.cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)  # Disable autofocus
        except:
            pass
    
    def start_capture(self) -> bool:
        """
        Start continuous frame capture in background thread
        """
        if not self.is_connected:
            logger.error("Cannot start capture: not connected to camera")
            return False
        
        if self.is_capturing:
            logger.warning("Capture already running")
            return True
        
        try:
            self.is_capturing = True
            self.capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
            self.capture_thread.start()
            logger.info("Started continuous capture")
            return True
            
        except Exception as e:
            logger.error(f"Failed to start capture: {e}")
            self.is_capturing = False
            return False
    
    def stop_capture(self):
        """
        Stop continuous frame capture
        """
        self.is_capturing = False
        if self.capture_thread:
            self.capture_thread.join(timeout=2.0)
        logger.info("Stopped continuous capture")
    
    def get_frame(self) -> Tuple[bool, Optional[np.ndarray]]:
        """
        Get the latest captured frame
        """
        with self.frame_lock:
            if self.current_frame is not None:
                return True, self.current_frame.copy()
            else:
                return False, None
    
    def get_single_frame(self) -> Tuple[bool, Optional[np.ndarray]]:
        """
        Capture a single frame directly (not from continuous capture)
        """
        if not self.is_connected or not self.cap:
            return False, None
        
        try:
            ret, frame = self.cap.read()
            if ret and frame is not None:
                return True, frame
            else:
                return False, None
                
        except Exception as e:
            logger.error(f"Error capturing single frame: {e}")
            return False, None
    
    def get_stats(self) -> Dict:
        """
        Get capture statistics
        """
        return {
            'connected': self.is_connected,
            'capturing': self.is_capturing,
            'backend': self.backend_name,
            'resolution': f"{self.width}x{self.height}",
            'fps': self.fps,
            'frames_captured': self.frame_count,
            'frames_dropped': self.dropped_frames,
            'uptime': time.time() - self.connection_start_time if self.connection_start_time else 0
        }
    
    def disconnect(self):
        """
        Disconnect from camera and cleanup
        """
        logger.info("Disconnecting from camera...")
        
        self.stop_capture()
        
        if self.cap:
            self.cap.release()
            self.cap = None
        
        self.is_connected = False
        self.current_frame = None
        self.frame_count = 0
        
        logger.info("Camera disconnected")
    
    def _capture_loop(self):
        """
        Continuous capture loop (runs in background thread)
        """
        logger.info("Capture loop started")
        frame_interval = 1.0 / PERFORMANCE_SETTINGS['max_fps']  # Match Camo's 30fps
        
        while self.is_capturing and self.is_connected:
            try:
                start_time = time.time()
                
                ret, frame = self.cap.read()
                if ret and frame is not None:
                    with self.frame_lock:
                        self.current_frame = frame
                        self.frame_count += 1
                        self.last_frame_time = time.time()
                        
                    # Debug: Log every 100 frames to see if camera is working
                    if self.frame_count % 100 == 0:
                        logger.info(f"Camera captured {self.frame_count} frames")
                else:
                    self.dropped_frames += 1
                    if self.dropped_frames % 10 == 1:  # Log first few dropped frames
                        logger.warning(f"Dropped frame #{self.dropped_frames}")
                
                # Control frame rate
                elapsed = time.time() - start_time
                sleep_time = max(0, frame_interval - elapsed)
                if sleep_time > 0:
                    time.sleep(sleep_time)
                
            except Exception as e:
                logger.error(f"Error in capture loop: {e}")
                time.sleep(0.1)
        
        logger.info("Capture loop ended")