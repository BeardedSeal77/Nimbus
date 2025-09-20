import websocket
import socket
import json
import math

ROS2_WS_URL = "ws://localhost:9090" 
ROS_TOPIC = "/test/drone_pose"  

class ROS2PoseSubscriber:
    def __init__(self):
        self.ws = None
        self.connected = False
        self.running = False
        self.conn = None

    def start_tcp_server(self):
        # Create TCP server and wait for Unity
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.bind(('127.0.0.1', 8765))  # Unity connects here
        server.listen(1)
        print("Waiting for Unity to connect...")

        self.conn, addr = server.accept()
        print(f"Connected to Unity: {addr}")

    def on_open(self, ws):
        print("WebSocket opened")
        self.connected = True

        # Subscribe to ROS topic
        subscribe_msg = {
            "op": "subscribe",
            "topic": ROS_TOPIC
        }
        ws.send(json.dumps(subscribe_msg))
        print(f"Subscribed to {ROS_TOPIC}")

    def on_message(self, ws, message):
        msg = json.loads(message)

        if "msg" in msg and "position" in msg["msg"] and "orientation" in msg["msg"]:
            pos = msg["msg"]["position"]
            ori = msg["msg"]["orientation"]

            # Position
            x = pos["x"]
            y = pos["y"]
            z = pos["z"]

            # Orientation (quaternion -> yaw)
            yaw = self.quaternion_to_yaw(ori["x"], ori["y"], ori["z"], ori["w"])

            data = {
                "x": x,
                "y": y,
                "z": z,
                "yaw": yaw
            }

            msg_json = json.dumps(data) + "\n"

            try:
                self.conn.sendall(msg_json.encode("utf-8"))
            except Exception as e:
                print("Lost connection to Unity:", e)
                ws.close()

    def quaternion_to_yaw(self, x, y, z, w):
        # Convert quaternion to yaw angle in degrees
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return math.degrees(yaw) % 360

    def on_error(self, ws, error):
        print(f"WebSocket error: {error}")

    def on_close(self, ws, close_status_code, close_msg):
        print("WebSocket closed")
        self.connected = False

    def start(self):
        # Connect to Unity first
        self.start_tcp_server()

        # Start ROS bridge WebSocket
        self.ws = websocket.WebSocketApp(
            ROS2_WS_URL,
            on_open=self.on_open,
            on_message=self.on_message,
            on_error=self.on_error,
            on_close=self.on_close
        )

        self.running = True
        self.ws.run_forever()


if __name__ == "__main__":
    subscriber = ROS2PoseSubscriber()
    subscriber.start()
