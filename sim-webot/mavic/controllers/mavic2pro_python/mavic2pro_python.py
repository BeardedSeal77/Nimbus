# Copyright 1996-2024 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Python controller for Mavic 2 Pro with ROS2 integration via rosbridge.
   Connects to ROS2 container running at localhost:9090
   Also provides MJPEG video stream on http://localhost:8080/video"""

from controller import Robot, Keyboard
import json
import time
import threading
import base64
import io
from http.server import BaseHTTPRequestHandler, HTTPServer
try:
    import websocket
except ImportError:
    print("Warning: 'websocket-client' module not found. Install with: pip install websocket-client")
    websocket = None
try:
    from PIL import Image
except ImportError:
    print("Warning: 'Pillow' module not found. Install with: pip install Pillow")
    Image = None


def clamp(value, value_min, value_max):
    return min(max(value, value_min), value_max)


class MJPEGStreamHandler(BaseHTTPRequestHandler):
    """HTTP handler for MJPEG video streaming"""

    def do_GET(self):
        if self.path == '/video':
            self.send_response(200)
            self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=--jpgboundary')
            self.end_headers()

            print("Client connected to video stream")

            while True:
                try:
                    # Get latest frame from global
                    if hasattr(self.server, 'latest_frame') and self.server.latest_frame:
                        jpg = self.server.latest_frame

                        # Write MJPEG frame with headers as raw bytes
                        self.wfile.write(b"--jpgboundary\r\n")
                        self.wfile.write(b"Content-type: image/jpeg\r\n")
                        self.wfile.write(f"Content-length: {len(jpg)}\r\n\r\n".encode())
                        self.wfile.write(jpg)
                        self.wfile.write(b"\r\n")

                    time.sleep(0.033)  # 30 FPS
                except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                    print("Client disconnected from video stream")
                    break
                except Exception as e:
                    print(f"Error streaming frame: {e}")
                    break
        else:
            self.send_response(404)
            self.end_headers()

    def log_message(self, format, *args):
        pass  # Suppress logs


class VideoStreamServer:
    """MJPEG video streaming server"""

    def __init__(self, port=8080):
        self.port = port
        self.server = None
        self.thread = None
        self.running = False

    def start(self):
        """Start the HTTP server in a background thread"""
        self.server = HTTPServer(('0.0.0.0', self.port), MJPEGStreamHandler)
        self.server.latest_frame = None
        self.running = True
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        print(f"MJPEG video stream started on http://localhost:{self.port}/video")

    def update_frame(self, image_data, width, height):
        """Update the current frame (expects BGRA raw bytes from Webots)"""
        if not self.running or Image is None:
            return

        try:
            # Convert BGRA to RGB
            img = Image.frombytes('RGBA', (width, height), image_data)
            img = img.convert('RGB')

            # Encode as JPEG
            buffer = io.BytesIO()
            img.save(buffer, format='JPEG', quality=85)
            self.server.latest_frame = buffer.getvalue()
        except Exception as e:
            print(f"Error encoding frame: {e}")

    def stop(self):
        """Stop the HTTP server"""
        if self.server:
            self.server.shutdown()
            self.running = False
            print("Video stream stopped")


class ROS2Bridge:
    """Handle WebSocket connection to ROS2 rosbridge"""

    def __init__(self, host='localhost', port=9090):
        self.host = host
        self.port = port
        self.ws = None
        self.connected = False
        self.receive_thread = None
        self.running = False

    def connect(self):
        """Connect to rosbridge WebSocket server"""
        if websocket is None:
            print("Cannot connect to ROS2: websocket-client not installed")
            return False

        try:
            url = f"ws://{self.host}:{self.port}"
            print(f"Connecting to ROS2 rosbridge at {url}...")
            self.ws = websocket.WebSocketApp(
                url,
                on_open=self._on_open,
                on_message=self._on_message,
                on_error=self._on_error,
                on_close=self._on_close
            )

            self.running = True
            self.receive_thread = threading.Thread(target=self.ws.run_forever, daemon=True)
            self.receive_thread.start()

            # Wait for connection
            timeout = 5
            start_time = time.time()
            while not self.connected and (time.time() - start_time) < timeout:
                time.sleep(0.1)

            return self.connected

        except Exception as e:
            print(f"Failed to connect to ROS2: {e}")
            return False

    def _on_open(self, ws):
        """Callback when WebSocket connection opens"""
        self.connected = True
        print("Connected to ROS2 rosbridge")

    def _on_message(self, ws, message):
        """Callback when message received from ROS2"""
        try:
            data = json.loads(message)
            # Handle incoming ROS2 messages here
            if data.get('op') == 'publish':
                topic = data.get('topic')
                msg = data.get('msg')
                print(f"Received from {topic}: {msg}")
        except Exception as e:
            print(f"Error processing message: {e}")

    def _on_error(self, ws, error):
        """Callback when WebSocket error occurs"""
        print(f"ROS2 connection error: {error}")

    def _on_close(self, ws, close_status_code, close_msg):
        """Callback when WebSocket connection closes"""
        self.connected = False
        print("Disconnected from ROS2 rosbridge")

    def advertise(self, topic, msg_type):
        """Advertise a ROS2 topic for publishing"""
        if not self.connected or not self.ws:
            return False

        msg = {
            "op": "advertise",
            "topic": topic,
            "type": msg_type
        }
        try:
            self.ws.send(json.dumps(msg))
            return True
        except Exception as e:
            print(f"Failed to advertise topic: {e}")
            return False

    def publish(self, topic, msg):
        """Publish a message to a ROS2 topic"""
        if not self.connected or not self.ws:
            return False

        message = {
            "op": "publish",
            "topic": topic,
            "msg": msg
        }
        try:
            self.ws.send(json.dumps(message))
            return True
        except Exception as e:
            print(f"Failed to publish: {e}")
            return False

    def subscribe(self, topic, msg_type):
        """Subscribe to a ROS2 topic"""
        if not self.connected or not self.ws:
            return False

        msg = {
            "op": "subscribe",
            "topic": topic,
            "type": msg_type
        }
        try:
            self.ws.send(json.dumps(msg))
            return True
        except Exception as e:
            print(f"Failed to subscribe: {e}")
            return False

    def disconnect(self):
        """Disconnect from rosbridge"""
        self.running = False
        if self.ws:
            self.ws.close()


class Mavic2ProROS2Controller(Robot):
    K_VERTICAL_THRUST = 68.5
    K_VERTICAL_OFFSET = 0.6
    K_VERTICAL_P = 3.0
    K_ROLL_P = 50.0
    K_PITCH_P = 30.0

    def __init__(self):
        Robot.__init__(self)

        self.time_step = int(self.getBasicTimeStep())

        self.camera = self.getDevice("camera")
        self.camera.enable(self.time_step)

        self.front_left_led = self.getDevice("front left led")
        self.front_right_led = self.getDevice("front right led")

        self.imu = self.getDevice("inertial unit")
        self.imu.enable(self.time_step)

        self.gps = self.getDevice("gps")
        self.gps.enable(self.time_step)

        self.compass = self.getDevice("compass")
        self.compass.enable(self.time_step)

        self.gyro = self.getDevice("gyro")
        self.gyro.enable(self.time_step)

        self.keyboard = self.getKeyboard()
        self.keyboard.enable(self.time_step)

        self.camera_roll_motor = self.getDevice("camera roll")
        self.camera_pitch_motor = self.getDevice("camera pitch")

        self.front_left_motor = self.getDevice("front left propeller")
        self.front_right_motor = self.getDevice("front right propeller")
        self.rear_left_motor = self.getDevice("rear left propeller")
        self.rear_right_motor = self.getDevice("rear right propeller")

        motors = [self.front_left_motor, self.front_right_motor,
                  self.rear_left_motor, self.rear_right_motor]
        for motor in motors:
            motor.setPosition(float('inf'))
            motor.setVelocity(1.0)

        self.target_altitude = 1.0

        # Initialize ROS2 bridge
        self.ros2 = ROS2Bridge(host='localhost', port=9090)
        self.ros2_connected = False

        # Initialize video stream server
        self.video_server = VideoStreamServer(port=8080)

    def setup_ros2_topics(self):
        """Setup ROS2 topics for publishing drone state"""
        if not self.ros2_connected:
            return

        # Advertise topics
        self.ros2.advertise("/drone/pose", "geometry_msgs/PoseStamped")
        self.ros2.advertise("/drone/velocity", "geometry_msgs/TwistStamped")
        self.ros2.advertise("/drone/altitude", "std_msgs/Float64")
        self.ros2.advertise("/drone/status", "std_msgs/String")
        self.ros2.advertise("/camera/image_raw", "sensor_msgs/Image")

        print("ROS2 topics advertised")

    def publish_camera_image(self):
        """Publish camera image to ROS2"""
        if not self.ros2_connected:
            return

        # Get camera image
        image = self.camera.getImage()
        if image is None:
            return

        width = self.camera.getWidth()
        height = self.camera.getHeight()

        # Convert BGRA to RGB and encode as base64
        image_bytes = bytes(image)
        image_data = base64.b64encode(image_bytes).decode('utf-8')

        # Create ROS2 Image message
        image_msg = {
            "header": {
                "stamp": {"sec": int(self.getTime()), "nanosec": 0},
                "frame_id": "camera"
            },
            "height": height,
            "width": width,
            "encoding": "bgra8",
            "is_bigendian": 0,
            "step": width * 4,
            "data": image_data
        }
        self.ros2.publish("/camera/image_raw", image_msg)

    def publish_drone_state(self, x_pos, y_pos, altitude, roll, pitch, yaw,
                           roll_velocity, pitch_velocity, yaw_velocity):
        """Publish current drone state to ROS2"""
        if not self.ros2_connected:
            return

        # Publish pose
        pose_msg = {
            "header": {
                "stamp": {"sec": int(self.getTime()), "nanosec": 0},
                "frame_id": "world"
            },
            "pose": {
                "position": {"x": x_pos, "y": y_pos, "z": altitude},
                "orientation": {"x": roll, "y": pitch, "z": yaw, "w": 1.0}
            }
        }
        self.ros2.publish("/drone/pose", pose_msg)

        # Publish velocity
        velocity_msg = {
            "header": {
                "stamp": {"sec": int(self.getTime()), "nanosec": 0},
                "frame_id": "world"
            },
            "twist": {
                "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
                "angular": {"x": roll_velocity, "y": pitch_velocity, "z": yaw_velocity}
            }
        }
        self.ros2.publish("/drone/velocity", velocity_msg)

        # Publish altitude
        altitude_msg = {"data": altitude}
        self.ros2.publish("/drone/altitude", altitude_msg)

    def run(self):
        print("Start the drone...")

        # Start video stream server
        self.video_server.start()

        # Try to connect to ROS2
        print("Attempting to connect to ROS2 container...")
        self.ros2_connected = self.ros2.connect()

        if self.ros2_connected:
            self.setup_ros2_topics()
        else:
            print("Warning: Running without ROS2 connection")

        while self.step(self.time_step) != -1:
            if self.getTime() > 1.0:
                break

        print("You can control the drone with your computer keyboard:")
        print("- 'up': move forward.")
        print("- 'down': move backward.")
        print("- 'right': turn right.")
        print("- 'left': turn left.")
        print("- 'shift + up': increase the target altitude.")
        print("- 'shift + down': decrease the target altitude.")
        print("- 'shift + right': strafe right.")
        print("- 'shift + left': strafe left.")

        last_publish_time = 0
        publish_interval = 0.033  # Publish at 30Hz

        while self.step(self.time_step) != -1:
            time_now = self.getTime()

            roll, pitch, yaw = self.imu.getRollPitchYaw()
            x_pos, y_pos, altitude = self.gps.getValues()
            roll_velocity, pitch_velocity, yaw_velocity = self.gyro.getValues()

            led_state = int(time_now) % 2
            self.front_left_led.set(1 if led_state else 0)
            self.front_right_led.set(0 if led_state else 1)

            self.camera_roll_motor.setPosition(-0.115 * roll_velocity)
            self.camera_pitch_motor.setPosition(-0.1 * pitch_velocity)

            roll_disturbance = 0.0
            pitch_disturbance = 0.0
            yaw_disturbance = 0.0

            key = self.keyboard.getKey()
            while key >= 0:
                if key == Keyboard.UP:
                    pitch_disturbance = -2.0
                elif key == Keyboard.DOWN:
                    pitch_disturbance = 2.0
                elif key == Keyboard.RIGHT:
                    yaw_disturbance = -1.3
                elif key == Keyboard.LEFT:
                    yaw_disturbance = 1.3
                elif key == Keyboard.SHIFT + Keyboard.RIGHT:
                    roll_disturbance = -1.0
                elif key == Keyboard.SHIFT + Keyboard.LEFT:
                    roll_disturbance = 1.0
                elif key == Keyboard.SHIFT + Keyboard.UP:
                    self.target_altitude += 0.05
                    print(f"target altitude: {self.target_altitude:.2f} [m]")
                elif key == Keyboard.SHIFT + Keyboard.DOWN:
                    self.target_altitude -= 0.05
                    print(f"target altitude: {self.target_altitude:.2f} [m]")

                key = self.keyboard.getKey()

            roll_input = self.K_ROLL_P * clamp(roll, -1.0, 1.0) + roll_velocity + roll_disturbance
            pitch_input = self.K_PITCH_P * clamp(pitch, -1.0, 1.0) + pitch_velocity + pitch_disturbance
            yaw_input = yaw_disturbance
            clamped_difference_altitude = clamp(self.target_altitude - altitude + self.K_VERTICAL_OFFSET, -1.0, 1.0)
            vertical_input = self.K_VERTICAL_P * pow(clamped_difference_altitude, 3.0)

            front_left_motor_input = self.K_VERTICAL_THRUST + vertical_input - roll_input + pitch_input - yaw_input
            front_right_motor_input = self.K_VERTICAL_THRUST + vertical_input + roll_input + pitch_input + yaw_input
            rear_left_motor_input = self.K_VERTICAL_THRUST + vertical_input - roll_input - pitch_input + yaw_input
            rear_right_motor_input = self.K_VERTICAL_THRUST + vertical_input + roll_input - pitch_input - yaw_input

            self.front_left_motor.setVelocity(front_left_motor_input)
            self.front_right_motor.setVelocity(-front_right_motor_input)
            self.rear_left_motor.setVelocity(-rear_left_motor_input)
            self.rear_right_motor.setVelocity(rear_right_motor_input)

            # Update video stream with current camera frame
            image = self.camera.getImage()
            if image is not None:
                self.video_server.update_frame(
                    bytes(image),
                    self.camera.getWidth(),
                    self.camera.getHeight()
                )

            # Publish to ROS2 at specified interval
            if self.ros2_connected and (time_now - last_publish_time) >= publish_interval:
                self.publish_drone_state(x_pos, y_pos, altitude, roll, pitch, yaw,
                                        roll_velocity, pitch_velocity, yaw_velocity)
                self.publish_camera_image()
                last_publish_time = time_now

        # Cleanup
        self.video_server.stop()
        if self.ros2_connected:
            self.ros2.disconnect()


controller = Mavic2ProROS2Controller()
controller.run()