import socket
import cv2
import time
import threading

STREAM_URL = "http://127.0.0.1:5000/video_feed"  # Flask MJPEG stream
WIDTH = 640
HEIGHT = 480   
TCP_PORT = 9999

class HubVideoSubscriber:
    def __init__(self):
        self.conn = None
        self.running = False
        self.server = None
        self.lock = threading.Lock()

    def start_tcp_server(self):
        # Create tcp server and wait for Unity connection
        if not self.server:
            self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server.bind(('127.0.0.1', TCP_PORT))
            self.server.listen(1)
            print(f"Waiting for Unity to connect on port {TCP_PORT}...")

        self.conn, addr = self.server.accept()
        print(f"Connected to Unity: {addr}")

    def connect_video_stream(self):
        while self.running:
            cap = cv2.VideoCapture(STREAM_URL)
            if cap.isOpened():
                print("Connected to hub video stream")
                return cap
            print("Failed to connect to hub video, retrying ...")
            time.sleep(1)
            
    def send_to_unity(self, frame):
        # Resize and convert for Unity
        frame = cv2.resize(frame, (WIDTH, HEIGHT))
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame_rgb = cv2.flip(frame_rgb, 0)  # flip for Unity coords

        # Send to Unity
        data = frame_rgb.tobytes()
        length = len(data).to_bytes(4, byteorder='big')

        try:
            self.conn.sendall(length + data)
        except (BrokenPipeError, ConnectionResetError, OSError):
            print("Lost connection to Unity.")
            try:
                self.conn.close()
            except:
                pass
            self.conn = None
            return False

    def get_frame(self, cap):
        with self.lock:
            while self.running:
                if not self.conn:
                    print("Waiting for Unity to reconnect...")
                    self.start_tcp_server()

                ret, frame = cap.read()
                if not ret or frame is None:
                    print("Failed to grab frame from hub.py, retrying...")

                self.send_to_unity(frame)

    def start(self):
        self.running = True
        # Connect to Unity first
        self.start_tcp_server()
        # Connect to MJPEG stream from hub
        cap = self.connect_video_stream()
        video_thread = threading.Thread(target=self.get_frame, args=(cap,), daemon=True)
        video_thread.start()

        try:
            while self.running:
                if not cap.isOpened():
                    print("Waiting for video stream to reconnect...")
                    cap = self.connect_video_stream()
                    video_thread = threading.Thread(target=self.get_frame, args=(cap,), daemon=True)
                    video_thread.start()
                time.sleep(1)
                # self.get_frame(cap)
                
        except KeyboardInterrupt:
            print("Shutting down server...")
            self.running = False
            if self.conn:
                self.conn.close()

        cap.release()
        cv2.destroyAllWindows()


def main():
    subscriber = HubVideoSubscriber()
    subscriber.start()

if __name__ == "__main__":
    main()