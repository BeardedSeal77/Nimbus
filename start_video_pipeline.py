#!/usr/bin/env python3
"""
Nimbus Video Pipeline Startup Script
Starts AI.py first, waits for it to be ready, then starts video publisher
Runs both processes on separate threads with optimized CPU affinity
"""

import subprocess
import time
import threading
import os
import sys
import psutil
import logging
from pathlib import Path

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class VideosPipelineManager:
    """
    Manages the video pipeline with optimized threading and CPU affinity
    """
    
    def __init__(self):
        self.base_dir = Path(__file__).parent
        self.ai_process = None
        self.video_process = None
        self.running = False
        
        # CPU affinity settings for 14700KF (28 threads)
        # P-cores: 0-15 (8 cores, 16 threads)  
        # E-cores: 16-27 (12 cores, 12 threads)
        self.ai_cpu_cores = list(range(0, 8))      # AI.py gets P-cores 0-7 (8 threads)
        self.video_cpu_cores = list(range(8, 16))   # Video gets P-cores 8-15 (8 threads)
        
    def set_process_affinity(self, process, cpu_cores):
        """Set CPU affinity for a process"""
        try:
            psutil_process = psutil.Process(process.pid)
            psutil_process.cpu_affinity(cpu_cores)
            logger.info(f"Set process {process.pid} affinity to cores: {cpu_cores}")
        except Exception as e:
            logger.warning(f"Failed to set CPU affinity: {e}")
    
    def start_ai_process(self):
        """Start the AI.py process"""
        ai_script = self.base_dir / "nimbus-ai" / "AI.py"
        logger.info(f"Starting AI process: {ai_script}")
        
        try:
            # Start AI process
            self.ai_process = subprocess.Popen(
                [sys.executable, str(ai_script)],
                cwd=str(self.base_dir),
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                universal_newlines=True,
                bufsize=1  # Line buffered
            )
            
            # Set CPU affinity to P-cores 0-7
            self.set_process_affinity(self.ai_process, self.ai_cpu_cores)
            
            logger.info(f"AI process started with PID: {self.ai_process.pid}")
            
            # Monitor AI output in separate thread
            ai_monitor_thread = threading.Thread(
                target=self._monitor_process_output,
                args=(self.ai_process, "AI"),
                daemon=True
            )
            ai_monitor_thread.start()
            
            return True
            
        except Exception as e:
            logger.error(f"Failed to start AI process: {e}")
            return False
    
    def start_video_process(self):
        """Start the video publisher process"""
        video_script = self.base_dir / "video" / "main.py"
        logger.info(f"Starting Video process: {video_script}")
        
        try:
            # Start video process
            self.video_process = subprocess.Popen(
                [sys.executable, str(video_script)],
                cwd=str(self.base_dir / "video"),
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                universal_newlines=True,
                bufsize=1  # Line buffered
            )
            
            # Set CPU affinity to P-cores 8-15
            self.set_process_affinity(self.video_process, self.video_cpu_cores)
            
            logger.info(f"Video process started with PID: {self.video_process.pid}")
            
            # Monitor video output in separate thread
            video_monitor_thread = threading.Thread(
                target=self._monitor_process_output,
                args=(self.video_process, "VIDEO"),
                daemon=True
            )
            video_monitor_thread.start()
            
            return True
            
        except Exception as e:
            logger.error(f"Failed to start video process: {e}")
            return False
    
    def _monitor_process_output(self, process, name):
        """Monitor process output and log it"""
        try:
            for line in iter(process.stdout.readline, ''):
                if line.strip():
                    logger.info(f"[{name}] {line.strip()}")
        except Exception as e:
            logger.error(f"Error monitoring {name} output: {e}")
    
    def wait_for_ai_ready(self):
        """Wait 10 seconds for AI process to be ready"""
        logger.info("Waiting 10 seconds for AI.py to initialize...")
        
        for i in range(10, 0, -1):
            if not self.ai_process or self.ai_process.poll() is not None:
                logger.error("AI process died while waiting")
                return False
            
            logger.info(f"AI initializing... {i}s remaining")
            time.sleep(1)
        
        logger.info("AI process should be ready now")
        return True
    
    def start_pipeline(self):
        """Start the complete video pipeline"""
        logger.info("=" * 60)
        logger.info("STARTING NIMBUS VIDEO PIPELINE")
        logger.info("=" * 60)
        logger.info(f"CPU Info: {psutil.cpu_count()} cores, {psutil.cpu_count(logical=True)} threads")
        logger.info(f"AI.py will use cores: {self.ai_cpu_cores}")
        logger.info(f"Video publisher will use cores: {self.video_cpu_cores}")
        logger.info("")
        
        self.running = True
        
        # Step 1: Start AI process first
        if not self.start_ai_process():
            logger.error("Failed to start AI process")
            return False
        
        # Step 2: Wait 10 seconds for AI to be ready
        if not self.wait_for_ai_ready():
            logger.error("AI process not ready")
            self.stop_pipeline()
            return False
        
        # Step 3: Start video publisher
        if not self.start_video_process():
            logger.error("Failed to start video process")
            self.stop_pipeline()
            return False
        
        logger.info("✅ Video pipeline started successfully!")
        logger.info("Both processes are running with optimized CPU affinity")
        logger.info("Press Ctrl+C to stop the pipeline")
        
        return True
    
    def hard_reset_ai(self):
        """Perform a hard reset of the AI process"""
        logger.info("Performing hard reset of AI process...")
        
        if self.ai_process:
            logger.info("Force killing AI process...")
            self.ai_process.kill()
            try:
                self.ai_process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                pass
        
        # Wait a moment for cleanup
        time.sleep(2)
        
        # Restart AI process
        return self.start_ai_process()
    
    def stop_pipeline(self):
        """Stop both processes"""
        logger.info("Stopping video pipeline...")
        self.running = False
        
        if self.video_process:
            logger.info("Terminating video process...")
            self.video_process.terminate()
            try:
                self.video_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                logger.warning("Video process didn't terminate, killing it")
                self.video_process.kill()
        
        if self.ai_process:
            logger.info("Terminating AI process...")
            self.ai_process.terminate()
            try:
                self.ai_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                logger.warning("AI process didn't terminate, killing it")
                self.ai_process.kill()
        
        logger.info("Video pipeline stopped")
    
    def monitor_pipeline(self):
        """Monitor both processes and restart if needed"""
        while self.running:
            try:
                # Check if processes are still alive
                if self.ai_process and self.ai_process.poll() is not None:
                    logger.error("AI process died unexpectedly")
                    break
                
                if self.video_process and self.video_process.poll() is not None:
                    logger.error("Video process died unexpectedly")
                    break
                
                time.sleep(1)
                
            except KeyboardInterrupt:
                logger.info("Keyboard interrupt received")
                break
            except Exception as e:
                logger.error(f"Error monitoring pipeline: {e}")
                break
        
        self.stop_pipeline()

def main():
    """Main entry point"""
    pipeline = VideosPipelineManager()
    
    try:
        if pipeline.start_pipeline():
            pipeline.monitor_pipeline()
        else:
            logger.error("Failed to start pipeline")
            sys.exit(1)
            
    except KeyboardInterrupt:
        logger.info("Pipeline interrupted by user")
    except Exception as e:
        logger.error(f"Pipeline error: {e}")
        sys.exit(1)
    finally:
        pipeline.stop_pipeline()

if __name__ == "__main__":
    main()