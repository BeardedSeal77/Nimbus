import socket
import json
import time
import requests

DRONE_TELEMETRY_URL = "http://127.0.0.1:5000/api/debug/drone_telemetry"

class TelemetryForwarder:
    def __init__(self):
        self.conn = None
        self.addr = None
        self.server = None
        self.sent_objects = set()  # store already-sent objects by (label, position)

    def start_tcp_server(self):
        # Clear the objects set incase of reconnection
        self.sent_objects.clear()

        # Create TCP server and wait for Unity
        if not self.server:
            self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # allow quick restart
            self.server.bind(('127.0.0.1', 8888))
            self.server.listen(1)
            print(f"Waiting for Unity to connect on port {8888}...")

        self.conn, self.addr = self.server.accept()
        print(f"Connected to Unity: {self.addr}")

    def fetch_telemetry(self):  
        try:
            response = requests.get(DRONE_TELEMETRY_URL, timeout=2)
            if response.status_code == 200:
                return response.json()
        except Exception as e:
            print("Failed to fetch telemetry:", e)
        return None

    # Send object data to Unity via TCP
    def forward_new_objects(self):  
        telemetry = self.fetch_telemetry()
        if not telemetry or "world_objects" not in telemetry:
            return

        for label, obj in telemetry["world_objects"].items():
            position = obj.get("position", {})
            x, y, z = position.get("x", 0.0), position.get("y", 0.0), position.get("z", 0.0)

            # Use (label, position) as unique key
            unique_key = (label, (x, y, z))

            if unique_key not in self.sent_objects:
                # Flattened structure matches Unity’s DetectionData expectation
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
                except (BrokenPipeError, ConnectionResetError, OSError) as e:
                    print("Lost connection to Unity:", e)
                    try:
                        self.conn.close()
                    except:
                        pass
                    self.conn = None
                    break

    def start(self):
        self.start_tcp_server()

        while True:
            if not self.conn:
                print("Reconnecting to Unity...")
                self.start_tcp_server()
            
            self.forward_new_objects()
            time.sleep(0.2)


if __name__ == "__main__":
    forwarder = TelemetryForwarder()
    forwarder.start()
