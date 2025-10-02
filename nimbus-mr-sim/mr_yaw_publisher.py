import socket
import json
import websocket

# Unity TCP settings
HOST = '127.0.0.1'
PORT = 9998

def main():
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
                # Temporarily output yaw in terminal
                yaw = float(line)
                print(f"Received yaw: {yaw}")

            except ValueError:
                print("Invalid yaw data:", line)

if __name__ == "__main__":
    main()
