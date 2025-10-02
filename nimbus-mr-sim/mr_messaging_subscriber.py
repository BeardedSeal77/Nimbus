import socket
import json
import time
import random

# TCP server settings
TCP_PORT = 8889  # same port Unity connects to
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

    def start_tcp_server(self):
        # Create TCP server and wait for Unity
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.bind(('127.0.0.1', TCP_PORT))
        server.listen(1)
        print(f"Waiting for Unity to connect on port {TCP_PORT}...")

        self.conn, addr = server.accept()
        print(f"Connected to Unity: {addr}")

    def send_random_messages(self):
        while self.running:
            if self.conn:
                message = random.choice(MESSAGES)
                msg_json = json.dumps({"drone_message": message}) + "\n"
                try:
                    self.conn.sendall(msg_json.encode("utf-8"))
                    print("[SENT]", msg_json.strip())
                except Exception as e:
                    print("[TCP] Lost connection to Unity:", e)
                    self.conn = None
            time.sleep(6)  # send every x seconds

    def start(self):
        self.start_tcp_server()
        try:
            self.send_random_messages()
        except KeyboardInterrupt:
            print("Shutting down server...")
            self.running = False
            if self.conn:
                self.conn.close()


if __name__ == "__main__":
    server = DroneMessageServer()
    server.start()
