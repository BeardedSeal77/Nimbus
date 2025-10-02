import socket
import cv2
import time

STREAM_URL = "http://127.0.0.1:5000/video_feed"  # Flask MJPEG stream
WIDTH = 640
HEIGHT = 480   

class HubVideoSubscriber:
    def __init__(self):
        self.conn = None
        self.running = False

    def start_tcp_server(self):
        # Create tcp server and wait for Unity connection
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.bind(('127.0.0.1', 9999))
        server.listen(1)
        print("Waiting for Unity to connect...")

        self.conn, addr = server.accept()
        print(f"Connected to Unity: {addr}")

    def start(self):
        # Connect to Unity first
        self.start_tcp_server()

        # Open MJPEG stream from Flask
        cap = cv2.VideoCapture(STREAM_URL)

        if not cap.isOpened():
            print("Could not open video stream from hub.py")
            return

        print("Receiving video from hub.py /video_feed...")

        while True:
            try:
                ret, frame = cap.read()
                if not ret or frame is None:
                    print("⚠️ Failed to grab frame from hub.py, retrying...")
                    time.sleep(0.05)  # small delay to avoid busy loop
                    continue  # continue instead of break

                # Resize and convert for Unity
                frame = cv2.resize(frame, (WIDTH, HEIGHT))
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                frame_rgb = cv2.flip(frame_rgb, 0)  # flip for Unity coords

                # Send to Unity
                data = frame_rgb.tobytes()
                length = len(data).to_bytes(4, byteorder='big')

                try:
                    self.conn.sendall(length + data)
                except:
                    print("Lost connection to Unity.")
                    break
            except Exception as e:
                print(f"Exception during frame capture: {e}")
                time.sleep(0.05)
                continue

        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    subscriber = HubVideoSubscriber()
    subscriber.start()
