import cv2
import time
import threading
import queue
import base64
import io
import requests
from PIL import Image
from djitellopy import Tello

# Make sure VideoPublisher code is already imported

WIDTH = 640
HEIGHT = 480
HUB_URL = "http://localhost:5000"  # change to your hub endpoint

# Initialize Tello
tello = Tello()
tello.connect()
tello.streamon()

class VideoPublisher:
    """Encodes video frames and publishes to hub via HTTP POST"""

    def __init__(self, hub_url):
        self.hub_url = hub_url
        self.encoder_thread = None
        self.running = False
        self.frame_queue = queue.Queue(maxsize=2)
        self.last_publish_time = 0
        self.publish_interval = 0.033  # 30 FPS max

    def start(self):
        """Start the encoding and publishing thread"""
        self.running = True

        # Start encoding thread
        self.encoder_thread = threading.Thread(target=self._encoding_loop, daemon=True)
        self.encoder_thread.start()

        print(f"Video publisher started - sending to {self.hub_url}/drone/video")

    def queue_frame(self, image_data, width, height):
        """Queue a frame for encoding (non-blocking, called from main loop)"""
        if not self.running:
            return

        try:
            # Non-blocking put - drops frame if queue full
            self.frame_queue.put_nowait((image_data, width, height))
        except queue.Full:
            pass  # Drop frame if encoder is behind

    def _encoding_loop(self):
        """Background thread that encodes and publishes frames"""
        while self.running:
            try:
                # Get frame from queue with timeout
                image_data, width, height = self.frame_queue.get(timeout=0.1)

                if Image is None:
                    continue

                # Rate limit publishing
                current_time = time.time()
                if current_time - self.last_publish_time < self.publish_interval:
                    continue

                # Convert BGRA to RGB
                img = Image.frombytes('RGBA', (width, height), image_data)
                img = img.convert('RGB')

                # Encode as JPEG
                buffer = io.BytesIO()
                img.save(buffer, format='JPEG', quality=85)
                jpeg_bytes = buffer.getvalue()

                # Publish to hub
                try:
                    requests.post(
                        f"{self.hub_url}/drone/video",
                        json={
                            'data': base64.b64encode(jpeg_bytes).decode('ascii'),
                            'timestamp': current_time,
                            'width': width,
                            'height': height
                        },
                        timeout=0.01  # 10ms timeout
                    )
                    self.last_publish_time = current_time
                except requests.exceptions.RequestException:
                    pass  # Silently fail if hub not available

            except queue.Empty:
                continue
            except Exception as e:
                print(f"Error encoding/publishing frame: {e}")

    def stop(self):
        """Stop the encoding thread"""
        self.running = False

        if self.encoder_thread and self.encoder_thread.is_alive():
            self.encoder_thread.join(timeout=1)

        print("Video publisher stopped")

# Initialize video publisher
publisher = VideoPublisher(HUB_URL)
publisher.start()

try:
    while True:
        # Get the latest Tello frame
        frame = tello.get_frame_read().frame

        # Resize and convert BGR → RGB
        frame = cv2.resize(frame, (WIDTH, HEIGHT))
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Flip vertically to match Unity / OpenGL coordinates
        frame_rgb = cv2.flip(frame_rgb, 0)

        # Convert to bytes
        frame_bytes = frame_rgb.tobytes()

        # Queue frame for publishing
        publisher.queue_frame(frame_bytes, WIDTH, HEIGHT)

        # Small sleep to prevent CPU overload (Tello runs ~30 FPS anyway)
        time.sleep(0.01)

except KeyboardInterrupt:
    print("Stopping Tello stream...")

finally:
    # Cleanup
    publisher.stop()
    tello.streamoff()
    tello.end()
    print("Tello disconnected and publisher stopped")
