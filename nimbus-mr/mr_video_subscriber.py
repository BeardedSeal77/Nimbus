import websocket
import socket
import json
import base64
import cv2
import numpy as np

ROS2_WS_URL = "ws://localhost:9090" 
WIDTH = 640
HEIGHT = 480   

class ROS2VideoSubscriber:
    def __init__(self):
        self.ws = None
        self.connected = False
        self.running = False
        self.conn = None

    def start_tcp_server(self):
        # Creaate tcp server and wait for connection
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.bind(('127.0.0.1', 9999))
        server.listen(1)
        print("Waiting for Unity to connect...")

        # Connection establishing, server.accept waits until connection is received
        self.conn, addr = server.accept()
        print(f"Connected to Unity: {addr}")

    def on_open(self, ws):
        print("WebSocket opened")
        self.connected = True

        # Subscribe to /camera/image_raw topic
        subscribe_msg = {
            "op": "subscribe",
            "topic": "/camera/image_raw"
        }
        ws.send(json.dumps(subscribe_msg))


#           JSON message example
# {
#   "op": "publish",
#   "topic": "/camera/image_raw",
#   "msg": {
#     "header": { ... },
#     "height": 480,
#     "width": 640,
#     "encoding": "jpeg",
#     "is_bigendian": 0,
#     "step": 0,
#     "data": "/9j/4AAQSkZJRgABAQAAAQABAAD..."  <-- base64-encoded JPEG image
#   }
# }

    def on_message(self, ws, message):
        msg = json.loads(message)

        if 'msg' in msg and 'data' in msg['msg']:
            # Extract base64 JPEG data from the message
            b64data = msg['msg']['data']
            jpeg_bytes = base64.b64decode(b64data)
            np_arr = np.frombuffer(jpeg_bytes, np.uint8)
            # Decode JPEG to OpenCV image
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if frame is not None:
                # Resize to Unity size
                frame = cv2.resize(frame, (WIDTH, HEIGHT))
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                frame_rgb = cv2.flip(frame_rgb, 0) # Unity coords are flipped

                # Sending to Unity
                data = frame_rgb.tobytes()
                # Converts length of frame bytes to 4 bytes big-endian order
                length = len(data).to_bytes(4, byteorder='big')

                try:
                    self.conn.sendall(length+data)
                except:
                    print("Lost connection to Unity.")
                    ws.close()

    def on_error(self, ws, error):
        print(f"WebSocket error: {error}")

    def on_close(self, ws, close_status_code, close_msg):
        print("WebSocket closed")
        self.connected = False

    def start(self):
        # Connect to Unity first
        self.start_tcp_server()

        # Creating websocket
        self.ws = websocket.WebSocketApp(
            ROS2_WS_URL,
            on_open=self.on_open,
            on_message=self.on_message,
            on_error=self.on_error,
            on_close=self.on_close
        )

        self.running = True
        self.ws.run_forever()

        cv2.destroyAllWindows()


if __name__ == "__main__":
    subscriber = ROS2VideoSubscriber()
    subscriber.start()
