#!/usr/bin/env python3
"""
Simple Direct Video Feed
Camo Studio → AI.py → Display (no complex nodes, just video)
"""

import cv2
import time
import numpy as np
import sys
import os
import logging

# Add project paths
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from helpers.phone_camera import PhoneCamera
from helpers.display import ProcessedDisplay

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class SimpleVideoFeed:
    """
    Simple video feed through AI.py without complex processing
    """
    
    def __init__(self):
        self.camera = None
        self.display = None
        self.is_running = False
        
        # Stats
        self.frame_count = 0
        self.start_time = None
        
    def setup_camera(self):
        """Set up Camo Studio camera"""
        logger.info("Setting up Camo Studio camera...")
        
        self.camera = PhoneCamera()
        success = self.camera.connect_usb_camera(0)  # Camera 0 = Camo Studio
        
        if success:
            info = self.camera.get_camera_info()
            logger.info(f"✅ Camera connected: {info['width']}x{info['height']} @ {info['fps']}fps")
            return True
        else:
            logger.error("❌ Failed to connect to Camo Studio")
            return False
    
    def setup_display(self):
        """Set up display helper"""
        self.display = ProcessedDisplay()
        logger.info("✅ Display system ready")
        return True
    
    def process_frame_simple(self, frame):
        """Simple frame processing - just pass through for now"""
        # This is where AI.py processing would happen
        # For now, just return empty results
        
        mock_ai_result = {
            'camera_pose': None,  # No SLAM yet
            'bounding_box': None,  # No object detection yet
            'object_pose': None,  # No 3D positioning yet
            'processing_status': 'video_only'
        }
        
        return mock_ai_result
    
    def run_video_feed(self):
        """Run the simple video feed"""
        logger.info("🚀 Starting Simple Video Feed...")
        
        # Setup components
        if not self.setup_camera():
            return False
        
        if not self.setup_display():
            return False
        
        logger.info("📹 Video feed starting...")
        logger.info("Controls: 'q' = quit, 's' = stats")
        
        self.is_running = True
        self.start_time = time.time()
        
        try:
            while self.is_running:
                # Get frame from camera
                ret, frame = self.camera.get_frame()
                if not ret or frame is None:
                    logger.warning("⚠️  Failed to get frame from camera")
                    continue
                
                self.frame_count += 1
                
                # Simple processing (bypass complex AI for now)
                ai_result = self.process_frame_simple(frame)
                
                # Create display frame
                display_frame = self.display.create_display_frame(
                    frame,
                    ai_result,
                    self.frame_count,
                    time.time() - self.start_time
                )
                
                # Add simple status overlay
                cv2.putText(display_frame, "NIMBUS AI - Simple Video Feed", 
                          (10, display_frame.shape[0] - 60), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                
                cv2.putText(display_frame, "AI Processing: DISABLED (video only)", 
                          (10, display_frame.shape[0] - 30), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
                
                # Show video feed
                cv2.imshow('Nimbus AI - Simple Video Feed', display_frame)
                
                # Handle keyboard input
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    logger.info("🛑 Stopping video feed...")
                    break
                elif key == ord('s'):
                    elapsed = time.time() - self.start_time
                    fps = self.frame_count / elapsed if elapsed > 0 else 0
                    logger.info(f"📊 Stats: {self.frame_count} frames @ {fps:.1f} FPS")
        
        except KeyboardInterrupt:
            logger.info("🛑 Video feed interrupted by user")
        
        finally:
            self.cleanup()
        
        return True
    
    def cleanup(self):
        """Clean up resources"""
        logger.info("🧹 Cleaning up...")
        
        self.is_running = False
        
        if self.camera:
            self.camera.disconnect()
            logger.info("📷 Camera disconnected")
        
        cv2.destroyAllWindows()
        
        # Show final stats
        if self.start_time:
            elapsed = time.time() - self.start_time
            avg_fps = self.frame_count / elapsed if elapsed > 0 else 0
            logger.info(f"📊 Final stats: {self.frame_count} frames in {elapsed:.1f}s (avg {avg_fps:.1f} FPS)")
        
        logger.info("✅ Cleanup completed")

def main():
    """Main entry point"""
    logger.info("=" * 60)
    logger.info("NIMBUS AI - SIMPLE VIDEO FEED")
    logger.info("=" * 60)
    logger.info("Pipeline: Camo Studio → Simple Processing → Display")
    logger.info("(Complex AI nodes disabled for now)")
    logger.info("")
    
    feed = SimpleVideoFeed()
    feed.run_video_feed()

if __name__ == "__main__":
    main()