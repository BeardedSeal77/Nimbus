# TESTING PUBLISHER - SENDS RANDOM MESSAGES EVERY 5 SECONDS TO ROS TOPIC
# Commands to check the topic list and to see the messages sent to the topic:
# 
# docker exec ros2-central bash -c "source /opt/ros/humble/setup.bash && ros2 topic list"
# 
# docker exec ros2-central bash -c "source /opt/ros/humble/setup.bash && ros2 topic echo /test/messages"

import random
import socket
import json
import time
import websocket

# ROS 2 WebSocket bridge
ROS2_WS_URL = "ws://localhost:9090"  # Ros port
TOPIC = "/test/messages"
MSG_TYPE = "std_msgs/String"

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

    drone_messages = [
    "Drone altitude stable at 35.2 meters",
    "Battery level at 76%, proceeding to waypoint 3",
    "Warning: High wind detected, adjusting heading",
    "GPS lock acquired, position fix successful",
    "Landing sequence initiated, ETA 12 seconds"
    ]

    try:
        while True:
            data_message = drone_messages[random.randint(0,4)]
            print("Publishing message: "+ data_message)

            # message object
            publish_msg = {
                "op": "publish",
                "topic": TOPIC,
                "msg": {"data": data_message}
            }
            # sending message over websocket
            ws.send(json.dumps(publish_msg))

            # 5 second delay between messages
            time.sleep(5)
    except KeyboardInterrupt:
        print("\nStopping publisher...")
    finally:
            ws.close()
            print("WebSocket closed")

if __name__ == "__main__":
    main()
