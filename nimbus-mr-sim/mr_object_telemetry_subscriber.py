import socket
import json
import time
import requests

DRONE_TELEMETRY_URL = "http://127.0.0.1:5000/api/target_status"
TCP_PORT = 8888

class TelemetryForwarder:
    def __init__(self):
        self.conn = None
        self.addr = None
        self.server = None
        self.sent_objects = set()  # already-sent objects (label, position)
        self.last_detection_count = None
        self.current_target = None
        self.waiting_first_detection = False  # True when target switched
        self.session = requests.Session()

    def start_tcp_server(self):
        self.sent_objects.clear()
        if not self.server:
            self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server.bind(('127.0.0.1', TCP_PORT))
            self.server.listen(1)
            print(f"Waiting for Unity to connect on port {TCP_PORT}...")

        self.conn, self.addr = self.server.accept()
        print(f"Connected to Unity: {self.addr}")

    def fetch_telemetry(self):
        try:
            response = self.session.get(DRONE_TELEMETRY_URL, timeout=2)
            if response.status_code == 200:
                return response.json()
            else:
                print(f"Non-200 status: {response.status_code}")
        except Exception as e:
            print("Failed to fetch telemetry:", e)
        return None

    def forward_new_objects(self):
        telemetry = self.fetch_telemetry()
        if not telemetry:
            print("No telemetry received")
            return

        if telemetry.get("status") != "ok":
            print(f"Telemetry error: {telemetry.get('message')}")
            return

        label = telemetry.get("target")
        detection_count = telemetry.get("detection_count")
        pos = telemetry.get("position")

        # If target switched, wait for first detection
        if label != self.current_target:
            print(f"Target switched: {self.current_target} -> {label}, waiting for first detection...")
            self.current_target = label
            self.last_detection_count = None
            self.sent_objects.clear()
            self.waiting_first_detection = True
            return

        # Initialize last_detection_count on first actual detection
        if self.waiting_first_detection:
            if detection_count > 0:
                print(f"First detection for target '{label}': {detection_count}")
                self.last_detection_count = detection_count
                self.waiting_first_detection = False
            else:
                # Still waiting for first detection
                print(f"Waiting for first detection of target '{label}'...")
                return

        # Only forward if detections increased by >= 15
        if detection_count - self.last_detection_count >= 100:
            x, y, z = pos.get("x", 0.0), pos.get("y", 0.0), pos.get("z", 0.0)
            unique_key = (label, (x, y, z))

            if unique_key not in self.sent_objects and self.conn:
                obj_data = {
                    "label": label,
                    "x": x,
                    "y": y,
                    "z": z
                }
                try:
                    msg_json = json.dumps(obj_data) + "\n"
                    self.conn.sendall(msg_json.encode("utf-8"))
                    print(f"[SENT] {msg_json.strip()}")
                    self.sent_objects.add(unique_key)
                    self.last_detection_count = detection_count
                except (BrokenPipeError, ConnectionResetError, OSError) as e:
                    print("Lost connection to Unity:", e)
                    try:
                        self.conn.close()
                    except:
                        pass
                    self.conn = None
        else:
            print(f"Target: {label}, Detections: {detection_count}, Pos: {pos}")

    def start(self):
        self.start_tcp_server()
        try:
            while True:
                if not self.conn:
                    print("Reconnecting to Unity...")
                    self.start_tcp_server()
                self.forward_new_objects()
                time.sleep(0.5)
        except KeyboardInterrupt:
            print("Shutting down server...")
            self.running = False
            if self.conn:
                self.conn.close()


def main():
    forwarder = TelemetryForwarder()
    forwarder.start()

if __name__ == "__main__":
    main()