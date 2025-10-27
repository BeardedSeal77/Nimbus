import socket
import json
import websocket
import time
import requests

# Unity TCP settings
HOST = '127.0.0.1'
PORT = 9998

FLASK_URL = "http://127.0.0.1:5000/api/headset/yaw"

def start_tcp_server():
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind((HOST, PORT))
    server.listen(1)
    server.settimeout(1.0)  # allows checking for shutdown / Ctrl+C
    print(f"Waiting for Unity on port {PORT}...")
    return server

def wait_for_unity(server, running):
    conn = None
    addr = None
    while running and conn is None:
        try:
            conn, addr = server.accept()
            print(f"Connected by {addr}")
        except socket.timeout:
            continue  # retry until Unity connects
    return conn, addr

def send_yaw_to_flask(yaw):
    try:
        response = requests.post(FLASK_URL, json={'yaw': yaw}, timeout=0.5)
        if response.status_code != 200:
            print(f"[Flask Error] {response.text}")
    except requests.exceptions.RequestException:
        print("[Warning] Failed to send yaw to Flask")

def main():
    # Start TCP server for Unity
    running = True
    server = start_tcp_server()

    # Waits for connection before proceeding
    conn, addr = None, None
    
    buffer = ""
    try:
        while running:
            if conn is None:
                conn, addr = wait_for_unity(server, running)
                buffer = ""  # clear buffer on new connection
            
            try:
                data = conn.recv(1024).decode('utf-8')
                if not data:
                    print("[WARNING] Lost connection to Unity.")
                    conn.close()
                    conn = None
                    continue

                # Each yaw value contained before line break character
                buffer += data
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    try:
                        # Output yaw in terminal and send to flask endpoint
                        yaw = float(line)
                        print(f"Received yaw: {yaw}")

                        send_yaw_to_flask(yaw)
                    except ValueError:
                        print("Invalid yaw data:", line)

            except (ConnectionResetError, OSError, BrokenPipeError):
                print("[WARNING] Lost connection to Unity.")
                try:
                    conn.close()
                except:
                    pass
                conn = None
                return False
    except KeyboardInterrupt:
        print("\nShutting down server...")
        running = False
        if conn:
            conn.close()
        server.close()

if __name__ == "__main__":
    main()
