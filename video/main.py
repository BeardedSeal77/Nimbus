#!/usr/bin/env python3
"""
Camo Studio Camera Test
Standalone Python program to test camera connection without ROS2
"""

import cv2
import time
import logging
import sys
import os
import threading
from concurrent.futures import ThreadPoolExecutor
from queue import Queue, Empty
from datetime import datetime

# Add current directory to path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from capture import CameraCapture
from ros2_publisher import ROS2VideoPublisher
from config import PERFORMANCE_SETTINGS

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.StreamHandler(),
        logging.FileHandler('camera_test.log')
    ]
)
logger = logging.getLogger(__name__)

class CameraToROS2App:
    """
    Main application class for publishing camera to ROS2
    """
    
    def __init__(self):
        self.capture = CameraCapture()
        self.publisher = ROS2VideoPublisher()
        self.running = False
        
        # Multi-threading components
        self.frame_queue = Queue(maxsize=5)  # Small queue to prevent lag
        self.encode_queue = Queue(maxsize=5)
        self.publish_queue = Queue(maxsize=5)
        
        # Thread pool for parallel processing
        self.thread_pool = ThreadPoolExecutor(
            max_workers=PERFORMANCE_SETTINGS['encode_threads'],
            thread_name_prefix='VideoEncoder'
        )
        
    def run(self):
        """
        Main application loop
        """
        logger.info("=" * 60)
        logger.info("CAMO STUDIO TO ROS2 PUBLISHER")
        logger.info("=" * 60)
        logger.info("Publishing camera feed to ROS2 container")
        logger.info("Pipeline: Camo Studio -> This App -> ROS2 -> AI.py")
        logger.info("")
        
        try:
            # Connect to camera
            if not self._connect_camera():
                return False
            
            # Connect to ROS2
            if not self._connect_ros2():
                return False
            
            # Start the multi-threaded processing pipeline
            self._start_threaded_pipeline()
            
            return True
            
        except KeyboardInterrupt:
            logger.info("Application interrupted by user")
            return True
        except Exception as e:
            logger.error(f"Application error: {e}")
            return False
        finally:
            self._cleanup()
    
    def _connect_camera(self) -> bool:
        """
        Connect to the camera
        """
        if not self.capture.connect():
            return False
        
        # Start continuous capture
        if not self.capture.start_capture():
            return False
        
        return True
    
    def _connect_ros2(self) -> bool:
        """
        Connect to ROS2
        """
        if not self.publisher.connect():
            return False
        
        # Start publishing
        if not self.publisher.start_publishing():
            return False
        
        return True
    
    def _start_threaded_pipeline(self):
        """
        Start multi-threaded video processing pipeline
        Capture -> Encode -> Publish (all on separate threads)
        """
        logger.info("Starting multi-threaded video pipeline...")
        self.running = True
        
        # Start pipeline threads
        threads = [
            threading.Thread(target=self._frame_capture_loop, name="FrameCapture", daemon=True),
            threading.Thread(target=self._frame_publisher_loop, name="FramePublisher", daemon=True)
        ]
        
        for thread in threads:
            thread.start()
            logger.info(f"Started thread: {thread.name}")
        
        # Keep main thread alive and monitor
        try:
            while self.running:
                time.sleep(1)
        except KeyboardInterrupt:
            logger.info("Stopping pipeline...")
            self.running = False
            
        # Wait for threads to finish
        for thread in threads:
            thread.join(timeout=2)
    
    def _frame_capture_loop(self):
        """Frame capture thread - high priority"""
        logger.info("Frame capture thread started")
        frame_count = 0
        
        while self.running:
            try:
                ret, frame = self.capture.get_frame()
                
                if ret and frame is not None:
                    # Try to put frame in queue (non-blocking)
                    try:
                        self.frame_queue.put_nowait(frame)
                        frame_count += 1
                        
                        if frame_count % 300 == 0:  # Log less frequently
                            logger.info(f"Captured {frame_count} frames")
                            
                    except:
                        # Queue full, skip this frame to prevent lag
                        pass
                
                # Precise timing for 30fps
                time.sleep(1/30)
                
            except Exception as e:
                time.sleep(0.01)
    
    def _frame_publisher_loop(self):
        """Frame publishing thread - processes and publishes frames"""
        logger.info("Frame publisher thread started")
        published_count = 0
        
        while self.running:
            try:
                # Get frame from queue (with timeout)
                frame = self.frame_queue.get(timeout=0.1)
                
                # Publish directly (ROS2Publisher handles its own threading)
                self.publisher.publish_frame(frame)
                published_count += 1
                
                if published_count % 300 == 0:  # Log less frequently
                    logger.info(f"Published {published_count} frames")
                
                self.frame_queue.task_done()
                
            except Empty:
                # No frame available, continue
                continue
            except Exception as e:
                time.sleep(0.01)
    
    def _log_stats(self):
        """
        Log current statistics
        """
        capture_stats = self.capture.get_stats()
        publisher_stats = self.publisher.get_stats()
        
        logger.info("Current Stats:")
        logger.info("  Camera:")
        logger.info(f"    Frames captured: {capture_stats['frames_captured']}")
        logger.info(f"    Frames dropped: {capture_stats['frames_dropped']}")
        logger.info(f"    Resolution: {capture_stats['resolution']}")
        logger.info(f"    Backend: {capture_stats['backend']}")
        
        logger.info("  ROS2 Publisher:")
        logger.info(f"    Frames published: {publisher_stats['frames_published']}")
        logger.info(f"    Publish errors: {publisher_stats['publish_errors']}")
        logger.info(f"    Publishing FPS: {publisher_stats['publish_fps']:.1f}")
        logger.info(f"    Target FPS: {publisher_stats['target_fps']}")
        logger.info(f"    JPEG quality: {publisher_stats['jpeg_quality']}")
    
    def _cleanup(self):
        """
        Clean up resources
        """
        try:
            # Stop the pipeline
            self.running = False
            
            # Shutdown thread pool
            if hasattr(self, 'thread_pool'):
                self.thread_pool.shutdown(wait=True, timeout=2)
            
            # Disconnect components
            self.capture.disconnect()
            self.publisher.disconnect()
        except Exception as e:
            pass

def main():
    """
    Main entry point
    """
    # Print system information
    print(f"Python version: {sys.version}")
    print(f"OpenCV version: {cv2.__version__}")
    print(f"Working directory: {os.getcwd()}")
    print()
    
    # Create and run the application
    app = CameraToROS2App()
    success = app.run()
    
    if success:
        logger.info("Camera to ROS2 publishing completed successfully")
    else:
        logger.error("Camera to ROS2 publishing failed")
        sys.exit(1)

if __name__ == "__main__":
    main()