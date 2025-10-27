import socket
import json
import threading
import requests
import time
import math

DRONE_TELEMETRY_URL = "http://127.0.0.1:5000/api/debug/drone_telemetry"
TCP_PORT = 8765  # same port Unity connects to

class DroneTelemetryServer:
    def __init__(self):
        self.conn = None
        self.running = True
        self.lock = threading.Lock()

    def start_tcp_server(self):
        # Create TCP server and wait for Unity
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.bind(('127.0.0.1', TCP_PORT))  # Unity connects here
        server.listen(1)
        print(f"Waiting for Unity to connect on port {TCP_PORT}...")

        self.conn, addr = server.accept()
        print(f"Connected to Unity: {addr}")

    def send_to_unity(self, msg):
        with self.lock:
            if not self.conn:
                return False

            try:
                self.conn.sendall(msg.encode("utf-8"))
                return True
            except (BrokenPipeError, ConnectionResetError, OSError):
                print("[WARNING] Lost connection to Unity.")
                if self.conn:
                    try:
                        self.conn.close()
                    except:
                        pass
                    self.conn = None
                    return False

    def fetch_and_send_telemetry(self):
        while self.running:
            try:
                if not self.conn:
                    self.start_tcp_server()

                resp = requests.get(DRONE_TELEMETRY_URL)
                if resp.status_code == 200:
                    data = resp.json()

                    # Extract only the fields Unity expects
                    simplified = {
                        "x": data["position"]["x"],
                        "y": data["position"]["y"],
                        "z": data["position"]["z"],
                        "yaw": math.degrees(data["orientation"]["yaw"])
                    }

                    
                    msg_json = json.dumps(simplified) + "\n"
                    success = self.send_to_unity(msg_json)
                    # self.conn.sendall(msg_json.encode("utf-8"))
                    if success:
                        print("[SENT TO UNITY]", msg_json.strip())
                
            except Exception as e:
                print("[DRONE TELEMETRY] Error:", e)
                if self.conn:
                    try:
                        self.conn.close()
                    except:
                        pass
                    self.conn = None
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

def main():
    server = DroneTelemetryServer()
    server.start()

if __name__ == "__main__":
    main()