import websocket
import socket
import json

ROS2_WS_URL = "ws://localhost:9090"
ROS_TOPIC = "/test/messages"


class ROS2DroneMessageSubscriber:
    def __init__(self):
        self.ws = None
        self.conn = None

    def start_tcp_server(self):
        # Create TCP server and wait for Unity
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.bind(('127.0.0.1', 8889))  # Unity connects here (different port than obj_pose)
        server.listen(1)
        print("Waiting for Unity to connect...")

        self.conn, addr = server.accept()
        print(f"Connected to Unity: {addr}")

    def on_open(self, ws):
        print("[WS] Connected to rosbridge")
        subscribe_msg = {
            "op": "subscribe",
            "topic": ROS_TOPIC
        }
        ws.send(json.dumps(subscribe_msg))
        print(f"[WS] Subscribed to {ROS_TOPIC}")

    def on_message(self, ws, message):
        msg = json.loads(message)

        if "msg" in msg and "data" in msg["msg"]:
            # Get the raw string message (drone message)
            drone_message = msg["msg"]["data"]

            # Forward it directly to Unity
            msg_json = json.dumps({"drone_message": drone_message}) + "\n"
            try:
                self.conn.sendall(msg_json.encode("utf-8"))
                print("[SENT]", msg_json.strip())
            except Exception as e:
                print("Lost connection to Unity:", e)
                ws.close()

    def on_error(self, ws, error):
        print("[WS] Error:", error)

    def on_close(self, ws, close_status_code, close_msg):
        print("[WS] Closed")

    def start(self):
        # First wait for Unity
        self.start_tcp_server()

        # Connect to rosbridge
        self.ws = websocket.WebSocketApp(
            ROS2_WS_URL,
            on_open=self.on_open,
            on_message=self.on_message,
            on_error=self.on_error,
            on_close=self.on_close,
        )
        self.ws.run_forever()


if __name__ == "__main__":
    subscriber = ROS2DroneMessageSubscriber()
    subscriber.start()
