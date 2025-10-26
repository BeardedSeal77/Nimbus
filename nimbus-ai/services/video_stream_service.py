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

        # Connection status
        self.connected = False

        # Statistics
        self.frames_received = 0
        self.start_time = None

    def start_service(self, app, stream_url=None):
        """Start the video stream service"""
        self.app = app
        self.running = True

        self.thread = threading.Thread(target=self._video_loop, daemon=True)
        self.thread.start()

        app.config['VIDEO_STREAM_RUNNING'] = True
        logger.info("Video stream service started - reading from shared state")

    def stop_service(self):
        """Stop the video stream service"""
        self.running = False

        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=2)

        if self.app:
            self.app.config['VIDEO_STREAM_RUNNING'] = False
        logger.info("Video stream service stopped")

    def _video_loop(self):
        """Main video capture loop - reads from shared state"""
        import numpy as np

        logger.info("Video loop started - reading from shared state")
        self.connected = True
        self.start_time = time.time()

        while self.running:
            try:
                shared_state = self.app.shared_state

                if shared_state and shared_state.get('raw_video_frame'):
                    jpeg_bytes = shared_state['raw_video_frame']

                    if jpeg_bytes:
                        jpeg_array = np.frombuffer(jpeg_bytes, dtype=np.uint8)
                        frame = cv2.imdecode(jpeg_array, cv2.IMREAD_COLOR)

                        if frame is not None:
                            self.frames_received += 1

                            from services.ai_service import add_frame_for_processing
                            add_frame_for_processing(frame)

                            from services.display_service import update_current_frame
                            update_current_frame(frame)

                            if self.frames_received % 30 == 0:
                                elapsed = time.time() - self.start_time
                                fps = self.frames_received / elapsed if elapsed > 0 else 0
                                logger.info(f"Received {self.frames_received} frames @ {fps:.1f} FPS")

                time.sleep(0.01)

            except Exception as e:
                logger.error(f"Video stream loop error: {e}")
                time.sleep(1)


    def get_connection_status(self):
        """Get connection status info"""
        return {
            'connected': self.connected,
            'frames_received': self.frames_received,
            'uptime': time.time() - self.start_time if self.start_time else 0
        }

# Global service instance
video_stream_service_instance = VideoStreamService()

def start_service(app, stream_url=None):
    """Start the video stream service"""
    video_stream_service_instance.start_service(app, stream_url)

def stop_service():
    """Stop the video stream service"""
    video_stream_service_instance.stop_service()

def get_connection_status():
    """Get connection status"""
    return video_stream_service_instance.get_connection_status()