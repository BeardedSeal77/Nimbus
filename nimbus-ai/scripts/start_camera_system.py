#!/usr/bin/env python3
"""
Nimbus Camera System Startup
Coordinates camera publisher and AI bridge for ROS2 integration
"""

import subprocess
import threading
import time
import logging
import signal
import sys
import os

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class NimbusCameraSystem:
    """
    Manages the complete camera → ROS2 → AI pipeline
    """
    
    def __init__(self):
        self.camera_process = None
        self.bridge_process = None
        self.is_running = False
        
    def start_camera_publisher(self):
        """Start the camera publisher in separate process"""
        try:
            logger.info("Starting camera publisher...")
            
            # Start camera publisher as subprocess
            self.camera_process = subprocess.Popen([
                sys.executable, 
                os.path.join(os.path.dirname(__file__), "camera_publisher.py")
            ], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            
            logger.info("Camera publisher started")
            return True
            
        except Exception as e:
            logger.error(f"Failed to start camera publisher: {e}")
            return False
    
    def start_ai_bridge(self):
        """Start the AI bridge in separate process"""
        try:
            logger.info("Starting AI bridge...")
            
            # Wait a moment for camera publisher to initialize
            time.sleep(2)
            
            # Start AI bridge as subprocess  
            self.bridge_process = subprocess.Popen([
                sys.executable,
                os.path.join(os.path.dirname(__file__), "ros2_ai_bridge.py")
            ], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            
            logger.info("AI bridge started")
            return True
            
        except Exception as e:
            logger.error(f"Failed to start AI bridge: {e}")
            return False
    
    def monitor_processes(self):
        """Monitor running processes and log output"""
        def monitor_process(process, name):
            """Monitor a single process"""
            try:
                while process and process.poll() is None:
                    output = process.stdout.readline()
                    if output:
                        logger.info(f"[{name}] {output.strip()}")
                    
                    error = process.stderr.readline()
                    if error:
                        logger.error(f"[{name}] {error.strip()}")
                        
            except Exception as e:
                logger.error(f"Error monitoring {name}: {e}")
        
        # Start monitoring threads
        if self.camera_process:
            camera_thread = threading.Thread(
                target=monitor_process, 
                args=(self.camera_process, "CAMERA"),
                daemon=True
            )
            camera_thread.start()
        
        if self.bridge_process:
            bridge_thread = threading.Thread(
                target=monitor_process,
                args=(self.bridge_process, "AI_BRIDGE"), 
                daemon=True
            )
            bridge_thread.start()
    
    def start_system(self):
        """Start the complete system"""
        logger.info("=" * 60)
        logger.info("STARTING NIMBUS CAMERA SYSTEM")
        logger.info("=" * 60)
        logger.info("Pipeline: Camo Studio → ROS2 → AI.py")
        logger.info("")
        
        # Check if ROS2 container is running
        logger.info("Make sure ROS2 container is running:")
        logger.info("  docker-compose up ros2-central")
        logger.info("")
        
        self.is_running = True
        
        # Start camera publisher
        if not self.start_camera_publisher():
            return False
        
        # Start AI bridge
        if not self.start_ai_bridge():
            return False
        
        # Start monitoring
        self.monitor_processes()
        
        logger.info("=" * 60)
        logger.info("NIMBUS CAMERA SYSTEM RUNNING")
        logger.info("=" * 60)
        logger.info("Components:")
        logger.info("  ✓ Camera Publisher: Camo Studio → ROS2")
        logger.info("  ✓ AI Bridge: ROS2 → AI.py → ROS2")  
        logger.info("")
        logger.info("ROS2 Topics:")
        logger.info("  → /nimbus/camera/image_raw (camera feed)")
        logger.info("  → /nimbus/ai/slam_pose (SLAM results)")
        logger.info("  → /nimbus/ai/object_detection (detections)")
        logger.info("  → /nimbus/ai/processing_status (AI status)")
        logger.info("")
        logger.info("Press Ctrl+C to stop system")
        
        return True
    
    def stop_system(self):
        """Stop the system"""
        logger.info("Stopping Nimbus Camera System...")
        
        self.is_running = False
        
        # Stop processes
        if self.camera_process:
            logger.info("Stopping camera publisher...")
            self.camera_process.terminate()
            try:
                self.camera_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.camera_process.kill()
        
        if self.bridge_process:
            logger.info("Stopping AI bridge...")
            self.bridge_process.terminate()
            try:
                self.bridge_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.bridge_process.kill()
        
        logger.info("Nimbus Camera System stopped")
    
    def run_system(self):
        """Run the system until interrupted"""
        if not self.start_system():
            logger.error("Failed to start system")
            return
        
        try:
            # Keep running
            while self.is_running:
                time.sleep(1)
                
                # Check if processes are still running
                if self.camera_process and self.camera_process.poll() is not None:
                    logger.error("Camera publisher stopped unexpectedly")
                    break
                
                if self.bridge_process and self.bridge_process.poll() is not None:
                    logger.error("AI bridge stopped unexpectedly")
                    break
                    
        except KeyboardInterrupt:
            logger.info("System interrupted by user")
        finally:
            self.stop_system()

def signal_handler(signum, frame):
    """Handle system signals"""
    logger.info("Received signal, shutting down...")
    sys.exit(0)

def main():
    """Main entry point"""
    # Set up signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # Create and run system
    system = NimbusCameraSystem()
    system.run_system()

if __name__ == "__main__":
    main()