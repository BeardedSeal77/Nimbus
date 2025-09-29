"""
Minimal ROS2 camera feed test - no processing, just display
Tests raw ROS2 websocket performance
"""
import websocket
import json
import base64
import numpy as np
import cv2
import time
import threading

class ROS2RawTest:
    def __init__(self):
        self.ws = None
        self.frame_count = 0
        self.start_time = None
        self.last_frame = None
        self.lock = threading.Lock()

    def on_open(self, ws):
        print("Connected to ROS2")
        subscribe_msg = {
            "op": "subscribe",
            "topic": "/camera/image_raw",
            "type": "sensor_msgs/Image"
        }
        ws.send(json.dumps(subscribe_msg))
        print("Subscribed to /camera/image_raw")
        self.start_time = time.time()

    def on_message(self, ws, message):
        try:
            msg = json.loads(message)
            topic = msg.get('topic', '')

            if topic == '/camera/image_raw':
                self.frame_count += 1

                # Decode frame
                msg_data = msg.get('msg', {})
                width = msg_data.get('width', 0)
                height = msg_data.get('height', 0)
                encoding = msg_data.get('encoding', '')
                data = msg_data.get('data', '')

                if data and width and height:
                    # Decode base64
                    image_data = base64.b64decode(data)

                    if encoding == 'bgra8':
                        nparr = np.frombuffer(image_data, dtype=np.uint8)
                        frame = nparr.reshape((height, width, 4))
                        frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
                    elif encoding == 'bgr8':
                        nparr = np.frombuffer(image_data, dtype=np.uint8)
                        frame = nparr.reshape((height, width, 3))
                    elif encoding == 'rgb8':
                        nparr = np.frombuffer(image_data, dtype=np.uint8)
                        frame = nparr.reshape((height, width, 3))
                        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                    else:
                        return

                    # Calculate FPS
                    elapsed = time.time() - self.start_time
                    fps = self.frame_count / elapsed if elapsed > 0 else 0

                    # Add FPS overlay
                    cv2.putText(frame, f"FPS: {fps:.1f} | Frames: {self.frame_count}",
                               (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                    with self.lock:
                        self.last_frame = frame

                    if self.frame_count % 30 == 0:
                        print(f"Received {self.frame_count} frames @ {fps:.1f} FPS")

        except Exception as e:
            print(f"Error: {e}")

    def on_error(self, ws, error):
        print(f"WebSocket error: {error}")

    def on_close(self, ws, close_status_code, close_msg):
        print("WebSocket closed")

    def run(self):
        ws_url = "ws://localhost:9090"
        print(f"Connecting to {ws_url}")

        self.ws = websocket.WebSocketApp(
            ws_url,
            on_open=self.on_open,
            on_message=self.on_message,
            on_error=self.on_error,
            on_close=self.on_close
        )

        # Run websocket in separate thread
        ws_thread = threading.Thread(target=self.ws.run_forever, daemon=True)
        ws_thread.start()

        # Display loop in main thread
        print("Press 'q' to quit")
        while True:
            with self.lock:
                if self.last_frame is not None:
                    cv2.imshow('ROS2 Raw Feed Test', self.last_frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cv2.destroyAllWindows()
        self.ws.close()

if __name__ == "__main__":
    test = ROS2RawTest()
    test.run()