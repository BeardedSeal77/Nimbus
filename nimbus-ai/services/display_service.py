"""
Display Service
Handles OpenCV window display in background thread
"""

import threading
import time
import logging
import cv2
import numpy as np
from datetime import datetime

logger = logging.getLogger(__name__)

class DisplayService:
    def __init__(self):
        self.app = None
        self.thread = None
        self.running = False
        
        # Current frame data
        self.current_frame = None
        self.current_ai_result = {}
        self.frame_lock = threading.Lock()
        
        # Display system
        self.display = None
        
        # Statistics
        self.frames_displayed = 0
        
    def start_service(self, app):
        """Start the display service"""
        self.app = app
        
        # Initialize display system
        self._initialize_display()
        
        self.running = True
        self.thread = threading.Thread(target=self._display_loop, daemon=True)
        self.thread.start()
        
        app.config['DISPLAY_SERVICE_RUNNING'] = True
        logger.info("Display service started")
        
    def stop_service(self):
        """Stop the display service"""
        self.running = False
        
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=2)
        
        # Clean up OpenCV windows
        cv2.destroyAllWindows()
        
        if self.app:
            self.app.config['DISPLAY_SERVICE_RUNNING'] = False
        logger.info("Display service stopped")
    
    def _initialize_display(self):
        """Initialize the display system"""
        try:
            from helpers.display import ProcessedDisplay
            self.display = ProcessedDisplay()
            logger.info("Display system loaded")
        except ImportError as e:
            logger.warning(f"Display system not available: {e}")
    
    def update_current_frame(self, frame):
        """Update current frame (called by ROS2 service)"""
        with self.frame_lock:
            self.current_frame = frame.copy() if frame is not None else None
    
    def _display_loop(self):
        """Main display loop"""
        logger.info("Starting display loop...")
        
        while self.running and self.app:
            try:
                with self.frame_lock:
                    frame = self.current_frame.copy() if self.current_frame is not None else None
                
                if frame is not None and self.display:
                    # Get latest AI results
                    from services.ai_service import get_latest_detection_result
                    detection_result = get_latest_detection_result()
                    
                    # Create AI result for display
                    ai_result = self._create_ai_result(detection_result)
                    
                    # Create display frame with overlays
                    display_frame = self.display.create_display_frame(
                        frame, 
                        ai_result, 
                        self.frames_displayed, 
                        time.time()  # Simple elapsed time
                    )
                    
                    # Show frame
                    cv2.imshow('Nimbus AI - Live Video Feed', display_frame)
                    self.frames_displayed += 1
                    
                    # Handle key presses
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'):
                        logger.info("Quit requested by user")
                        self._request_shutdown()
                        break
                    elif key == ord('s'):
                        logger.info(f"Stats - Frames displayed: {self.frames_displayed}")
                    elif key == ord('r'):
                        logger.info("Reset requested")
                        self._reset_stats()
                
                time.sleep(0.033)  # ~30 FPS display rate
                
            except Exception as e:
                logger.error(f"Display loop error: {e}")
                time.sleep(0.1)
        
        cv2.destroyAllWindows()
        logger.info("Display loop stopped")
    
    def _create_ai_result(self, detection_result):
        """Create AI result dictionary for display"""
        return {
            'camera_pose': None,
            'bounding_box': detection_result.get('bounding_box'),
            'object_pose': None,
            'processing_status': detection_result.get('processing_status', 'live_stream'),
            'frame_shape': self.current_frame.shape if self.current_frame is not None else None,
            'timestamp': datetime.now(),
            'target_object': self.app.config['GLOBAL_OBJECT'],
            'intent': self.app.config['GLOBAL_INTENT'],
            'target_distance': detection_result.get('target_distance'),
            'depth_result': detection_result.get('depth_result'),
            'depth_frames_collected': detection_result.get('depth_frames_collected'),
            'depth_frames_needed': detection_result.get('depth_frames_needed'),
            'global_get_dist': self.app.config['GLOBAL_GET_DIST']
        }
    
    def _request_shutdown(self):
        """Request application shutdown"""
        # In a Flask app, we can't easily shutdown from a thread
        # This would need to be handled by the main Flask process
        logger.info("Shutdown requested from display service")
        self.running = False
    
    def _reset_stats(self):
        """Reset display statistics"""
        self.frames_displayed = 0
        logger.info("Display stats reset")
    
    def get_display_stats(self):
        """Get display statistics"""
        return {
            'frames_displayed': self.frames_displayed,
            'display_running': self.running
        }

# Global service instance
display_service_instance = DisplayService()

def start_service(app):
    """Start the display service"""
    display_service_instance.start_service(app)

def stop_service():
    """Stop the display service"""
    display_service_instance.stop_service()

def update_current_frame(frame):
    """Update current frame"""
    display_service_instance.update_current_frame(frame)

def get_display_stats():
    """Get display statistics"""
    return display_service_instance.get_display_stats()