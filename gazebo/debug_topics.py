#!/usr/bin/env python3
"""
Debug Topics Script
Tests communication between Gazebo (WSL2) and Docker ROS2
"""

import websocket
import json
import time
import threading

class RosbridgeDebugger:
    def __init__(self):
        self.ws = None
        self.connected = False

    def connect_to_docker(self):
        """Connect to Docker rosbridge"""
        try:
            print("🔗 Connecting to Docker rosbridge at ws://192.168.8.102:9090...")
            self.ws = websocket.WebSocket()
            self.ws.connect("ws://192.168.8.102:9090")
            self.connected = True
            print("✅ Connected to Docker rosbridge!")

            # Start listening thread
            listener_thread = threading.Thread(target=self.listen_for_messages, daemon=True)
            listener_thread.start()

        except Exception as e:
            print(f"❌ Failed to connect to Docker: {e}")
            return False
        return True

    def listen_for_messages(self):
        """Listen for incoming messages from rosbridge"""
        while self.connected:
            try:
                message = self.ws.recv()
                data = json.loads(message)
                print(f"📨 Received from Docker: {data}")
            except Exception as e:
                print(f"❌ Error receiving message: {e}")
                break

    def send_test_message(self):
        """Send a test message to Docker"""
        if not self.connected:
            print("❌ Not connected to Docker!")
            return

        test_msg = {
            "op": "publish",
            "topic": "/test/hello",
            "type": "std_msgs/String",
            "msg": {
                "data": "Hello from Windows debug script!"
            }
        }

        try:
            self.ws.send(json.dumps(test_msg))
            print("📤 Sent test message to Docker!")
        except Exception as e:
            print(f"❌ Failed to send test message: {e}")

    def subscribe_to_topic(self, topic_name):
        """Subscribe to a topic from Docker"""
        if not self.connected:
            print("❌ Not connected to Docker!")
            return

        subscribe_msg = {
            "op": "subscribe",
            "topic": topic_name,
            "type": "std_msgs/String"  # Adjust type as needed
        }

        try:
            self.ws.send(json.dumps(subscribe_msg))
            print(f"📥 Subscribed to {topic_name}")
        except Exception as e:
            print(f"❌ Failed to subscribe to {topic_name}: {e}")

    def list_topics(self):
        """List all available topics"""
        if not self.connected:
            print("❌ Not connected to Docker!")
            return

        list_msg = {
            "op": "call_service",
            "service": "/rosapi/topics",
            "args": {}
        }

        try:
            self.ws.send(json.dumps(list_msg))
            print("📋 Requested topic list from Docker...")
        except Exception as e:
            print(f"❌ Failed to get topic list: {e}")

def main():
    print("=" * 50)
    print("🐛 ROS2 TOPIC DEBUGGER")
    print("Testing Gazebo → Docker communication")
    print("=" * 50)

    debugger = RosbridgeDebugger()

    # Connect to Docker
    if not debugger.connect_to_docker():
        print("❌ Cannot proceed without Docker connection")
        return

    # Wait a moment
    time.sleep(1)

    print("\n🧪 Running tests...")

    # Test 1: List topics
    print("\n1️⃣ Listing topics in Docker:")
    debugger.list_topics()
    time.sleep(2)

    # Test 2: Send test message
    print("\n2️⃣ Sending test message:")
    debugger.send_test_message()
    time.sleep(1)

    # Test 3: Subscribe to Gazebo topics
    print("\n3️⃣ Subscribing to potential Gazebo topics:")
    topics_to_test = [
        "/tello/status",
        "/tello/cmd_vel",
        "/tello/odom",
        "/tello/camera/image_raw"
    ]

    for topic in topics_to_test:
        debugger.subscribe_to_topic(topic)
        time.sleep(0.5)

    print("\n⏳ Listening for messages for 10 seconds...")
    print("(If Gazebo topics appear, the bridge is working!)")
    time.sleep(10)

    print("\n🏁 Debug complete!")
    print("\nIf you see Gazebo topics above, the issue is elsewhere.")
    print("If you don't see them, Gazebo isn't reaching Docker.")

if __name__ == "__main__":
    main()