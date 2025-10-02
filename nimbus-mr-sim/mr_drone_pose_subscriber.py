import socket
import json
import threading
import requests
import time
import math

DRONE_TELEMETRY_URL = "http://127.0.0.1:5000/api/debug/drone_telemetry"


class DroneTelemetryServer:
    def __init__(self):
        self.conn = None
        self.running = True

    def start_tcp_server(self):
        # Create TCP server and wait for Unity
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.bind(('127.0.0.1', 8765))  # Unity connects here
        server.listen(1)
        print("Waiting for Unity to connect...")

        self.conn, addr = server.accept()
        print(f"Connected to Unity: {addr}")

    def fetch_and_send_telemetry(self):
        while self.running:
            try:
                resp = requests.get(DRONE_TELEMETRY_URL)
                if resp.status_code == 200:
                    data = resp.json()
                    print("[DRONE TELEMETRY]", json.dumps(data, indent=2))

                    # Extract only the fields Unity expects
                    simplified = {
                        "x": data["position"]["x"],
                        "y": data["position"]["y"],
                        "z": data["position"]["z"],
                        "yaw": math.degrees(data["orientation"]["yaw"])
                    }

                    if self.conn:
                        msg_json = json.dumps(simplified) + "\n"
                        self.conn.sendall(msg_json.encode("utf-8"))
                        print("[SENT TO UNITY]", msg_json.strip())
                else:
                    print("[DRONE TELEMETRY] Failed with status:", resp.status_code)
            except Exception as e:
                print("[DRONE TELEMETRY] Error:", e)
            time.sleep(0.1)  # send every 100ms for smoother Unity updates

    def start(self):
        self.start_tcp_server()

        telemetry_thread = threading.Thread(target=self.fetch_and_send_telemetry, daemon=True)
        telemetry_thread.start()

        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("Shutting down server...")
            self.running = False
            if self.conn:
                self.conn.close()


if __name__ == "__main__":
    server = DroneTelemetryServer()
    server.start()
