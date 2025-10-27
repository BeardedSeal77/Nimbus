import socket
import json
import time
import random
import threading
import requests

# TCP server settings
TCP_PORT = 8889  # same port Unity connects to
HUD_ENDPOINT = "http://127.0.0.1:5000/hud/message"
MESSAGES = [
    "Drone taking off",
    "Drone landing",
    "Obstacle detected",
    "Battery low",
    "Returning home"
]

class DroneMessageServer:
    def __init__(self):
        self.conn = None
        self.running = True
        self.lock = threading.Lock()
        self.last_message = None

    def start_tcp_server(self):
        # Create TCP server and wait for Unity
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.bind(('127.0.0.1', TCP_PORT))
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

    def poll_hud_message(self):
        """Poll endpoint for new messages"""
        try:
            response = requests.get(HUD_ENDPOINT, timeout = 2)
            if response.status_code == 200:
                data = response.json()
                return data.get("message")
        except requests.RequestException as e:
            print("[HUD] Connection error:",e)
        return None

    def message_watcher(self):
        """Forever checks for new messages from flask"""
        while self.running:
            if not self.conn:
                self.start_tcp_server()

            message = self.poll_hud_message()
            if message and message != self.last_message:
                msg_json = json.dumps({"drone_message": message})+"\n"
                
                try:
                    success = self.send_to_unity(msg_json)
                    if success:
                        print("[SENT TO UNITY]", msg_json.strip())
                except Exception as e:
                    print("[MESSAGING] Error:", e)
                    if self.conn:
                        try:
                            self.conn.close()
                        except:
                            pass
                        self.conn = None
                time.sleep(2)  # send every x seconds

    def start(self):
        self.start_tcp_server()

        messaging_thread = threading.Thread(target=self.message_watcher, daemon=True)
        messaging_thread.start()

        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("Shutting down server...")
            self.running = False
            if self.conn:
                self.conn.close()


def main():
    server = DroneMessageServer()
    server.start()

if __name__ == "__main__":
    main()