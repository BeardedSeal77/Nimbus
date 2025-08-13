import socket
import json
import websocket

# Unity TCP settings
HOST = '127.0.0.1'
PORT = 9998

# ROS 2 WebSocket bridge
ROS2_WS_URL = "ws://localhost:9090"  # Ros port

TOPIC = "/unity/yaw"
MSG_TYPE = "std_msgs/Float32"

def main():
    # Connect to rosbridge websocket
    ws = websocket.WebSocket()
    ws.connect(ROS2_WS_URL)
    print(f"Connected to rosbridge at {ROS2_WS_URL}")

    # Advertise the topic for ROS
    advertise_msg = {
        "op": "advertise",
        "topic": TOPIC,
        "type": MSG_TYPE
    }
    ws.send(json.dumps(advertise_msg))
    print(f"Advertised topic {TOPIC} as {MSG_TYPE}")

    # Start TCP server for Unity
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind((HOST, PORT))
    server.listen(1)
    print(f"Waiting for Unity on port {PORT}...")

    # Waits for connection before proceeding
    conn, addr = server.accept()
    print(f"Connected by {addr}")

    buffer = ""
    while True:
        data = conn.recv(1024).decode('utf-8')
        if not data:
            print("Lost connection to Unity")
            break

        # Each yaw value contained before line break character
        buffer += data
        while '\n' in buffer:
            line, buffer = buffer.split('\n', 1)
            try:
                yaw = float(line)
                print(f"Received yaw: {yaw}")

                # Publish yaw via rosbridge
                publish_msg = {
                    "op": "publish",
                    "topic": TOPIC,
                    "msg": {"data": yaw}
                }
                ws.send(json.dumps(publish_msg))

            except ValueError:
                print("Invalid yaw data:", line)

if __name__ == "__main__":
    main()
