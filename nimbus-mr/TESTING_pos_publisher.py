# publish dummy xyz and yaw data of the drone

# TESTING PUBLISHER - SENDS DRONE COORDINATES OF A DRONE FLYING IN A CIRCLE
# Commands to check the topic list and to see the messages sent to the topic:
# 
# docker exec ros2-central bash -c "source /opt/ros/humble/setup.bash && ros2 topic list"
# 
# docker exec ros2-central bash -c "source /opt/ros/humble/setup.bash && ros2 topic echo /test/drone_pose"

import math
import random
import socket
import json
import time
import websocket

# ROS 2 WebSocket bridge
ROS2_WS_URL = "ws://localhost:9090"  # Ros port
TOPIC = "/test/drone_pose"
MSG_TYPE = "geometry_msgs/Pose"


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


    t = 0
    r = 5.0   # radius of the circle
    z = 2.0   # fixed altitude

    try:
        while True:
            # Generate dummy SLAM data (circle path) this is parametric equations and angle = t* 0.1 to increase the angle slowly
            angle = t * 0.1
            x = r * math.cos(angle)
            y = r * math.sin(angle)
            yaw = angle # in radians

            # Convert yaw to quaternion (this is what RTAB-map uses) CONVERT BACK TO YAW IN SUBSCRIBER
            qx = 0.0
            qy = 0.0
            qz = math.sin(yaw / 2.0)
            qw = math.cos(yaw / 2.0)

            data = {
                "position": {"x": x, "y": y, "z": z},
                "orientation": {"x": qx, "y": qy, "z": qz, "w": qw}
            }

            print("Publishing Drone Pose: ", data)

            publish_msg = {
                 "op": "publish",
                 "topic": TOPIC,
                 "msg": data
            }

            ws.send(json.dumps(publish_msg))

            time.sleep(0.2)
            t += 1
    except KeyboardInterrupt:
        print("\nStopping publisher...")
    finally:
            ws.close()
            print("WebSocket closed")

if __name__ == "__main__":
    main()
