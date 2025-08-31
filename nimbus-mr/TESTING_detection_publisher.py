# publish dummy object data (label, xyz)

# TESTING PUBLISHER - SENDS OBJECT COORDINATES
# Commands to check the topic list and to see the messages sent to the topic:
# 
# docker exec ros2-central bash -c "source /opt/ros/humble/setup.bash && ros2 topic list"
# 
# docker exec ros2-central bash -c "source /opt/ros/humble/setup.bash && ros2 topic echo /test/obj_pose"

import math
import random
import socket
import json
import time
import websocket

# ROS 2 WebSocket bridge
ROS2_WS_URL = "ws://localhost:9090"  # Ros port
TOPIC = "/test/obj_pose"
MSG_TYPE = "std_msgs/String"    #stringifdied JSON reconvert to JSON in subscriber


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


    object_detections = [
        {"label": "Tree", "x": 3.2, "y": -1.5, "z": 0.0},
        {"label": "Car", "x": -2.0, "y": 4.1, "z": 0.0},
        {"label": "Person", "x": 1.0, "y": 0.5, "z": 0.0},
        {"label": "Dog", "x": -3.4, "y": -2.2, "z": 0.0},
        {"label": "Bench", "x": 0.0, "y": 6.0, "z": 0.0}
    ]

    try:
        while True:
            for obj in object_detections:  
                print("Publishing Drone Pose: ", obj)

                publish_msg = {
                    "op": "publish",
                    "topic": TOPIC,
                    "msg": {"data": json.dumps(obj)}    # stringify the obj
                }

                ws.send(json.dumps(publish_msg))

                time.sleep(5)
    except KeyboardInterrupt:
        print("\nStopping publisher...")
    finally:
            ws.close()
            print("WebSocket closed")

if __name__ == "__main__":
    main()
