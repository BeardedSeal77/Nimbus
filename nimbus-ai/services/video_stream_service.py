"""
Video Stream Service
Handles MJPEG video stream from Webots and provides frames for processing
"""

import threading
import time
import logging
import cv2

logger = logging.getLogger(__name__)

class VideoStreamService:
    def __init__(self):
        self.app = None
        self.thread = None
        self.running = False

        # Video capture
        self.cap = None
        self.connected = False

        # Statistics
        self.frames_received = 0
        self.start_time = None

    def start_service(self, app, stream_url='http://localhost:8080/video'):
        """Start the video stream service"""
        self.app = app
        self.stream_url = stream_url
        self.running = True

        # Start video capture thread
        self.thread = threading.Thread(target=self._video_loop, daemon=True)
        self.thread.start()

        app.config['VIDEO_STREAM_RUNNING'] = True
        logger.info(f"Video stream service started - connecting to {stream_url}")

    def stop_service(self):
        """Stop the video stream service"""
        self.running = False

        # Close video capture
        if self.cap:
            self.cap.release()

        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=2)

        if self.app:
            self.app.config['VIDEO_STREAM_RUNNING'] = False
        logger.info("Video stream service stopped")

    def _video_loop(self):
        """Main video capture loop"""
        retry_count = 0
        max_retries = 5

        while self.running and retry_count < max_retries:
            try:
                if not self.connected:
                    self._connect_to_stream()
                    if not self.connected:
                        retry_count += 1
                        if retry_count >= max_retries:
                            logger.error(f"Failed to connect after {max_retries} attempts. Stopping.")
                            self.running = False
                            break
                        time.sleep(2)  # Wait before retry
                else:
                    # Read frames
                    ret, frame = self.cap.read()
                    if ret:
                        self.frames_received += 1

                        if not self.start_time:
                            self.start_time = time.time()

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
                    else:
                        logger.warning("Failed to read frame from stream")
                        self.connected = False
                        time.sleep(1)

            except Exception as e:
                logger.error(f"Video stream loop error: {e}")
                self.connected = False
                retry_count += 1
                if retry_count >= max_retries:
                    logger.error(f"Failed to connect after {max_retries} attempts. Stopping.")
                    self.running = False
                    break
                time.sleep(2)

    def _connect_to_stream(self):
        """Connect to MJPEG video stream"""
        try:
            logger.info(f"Connecting to video stream at {self.stream_url}")

            self.cap = cv2.VideoCapture(self.stream_url)

            if self.cap.isOpened():
                self.connected = True
                logger.info("Video stream connection established")
            else:
                logger.error("Failed to open video stream")
                self.connected = False

        except Exception as e:
            logger.error(f"Error connecting to video stream: {e}")
            self.connected = False

    def get_connection_status(self):
        """Get connection status info"""
        return {
            'connected': self.connected,
            'frames_received': self.frames_received,
            'uptime': time.time() - self.start_time if self.start_time else 0
        }

# Global service instance
video_stream_service_instance = VideoStreamService()

def start_service(app, stream_url='http://localhost:8080/video'):
    """Start the video stream service"""
    video_stream_service_instance.start_service(app, stream_url)

def stop_service():
    """Stop the video stream service"""
    video_stream_service_instance.stop_service()

def get_connection_status():
    """Get connection status"""
    return video_stream_service_instance.get_connection_status()