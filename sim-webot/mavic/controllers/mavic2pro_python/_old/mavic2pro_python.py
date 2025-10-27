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
import queue
import requests
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


# ============================================================================
# GLOBAL CONFIGURATION
# ============================================================================

CONFIG = {
    # Flight Control Parameters (balanced for Webots simulation)
    'K_VERTICAL_THRUST': 68.5,  # Base thrust (back to stable value)
    'K_VERTICAL_OFFSET': 0.6,
    'K_VERTICAL_P': 3.5,
    'K_VERTICAL_D': 0.3,
    'K_ROLL_P': 55.0,
    'K_PITCH_P': 35.0,

    # Motor limiting (prevent overcorrection oscillations)
    'MAX_MOTOR_DIFFERENTIAL': 15.0,  # Max difference from base thrust per motor

    # Control Inputs (keyboard disturbances) - increased for higher speed
    'PITCH_DISTURBANCE_FORWARD': -6.0,
    'PITCH_DISTURBANCE_BACKWARD': 6.0,
    'YAW_DISTURBANCE_RIGHT': -1.3,
    'YAW_DISTURBANCE_LEFT': 1.3,
    'ROLL_DISTURBANCE_RIGHT': -6.0,
    'ROLL_DISTURBANCE_LEFT': 6.0,
    'ALTITUDE_INCREMENT': 0.05,

    # Smooth control ramping (prevents snapping)
    'DISTURBANCE_RAMP_TIME': 0.3,  # Seconds to reach max disturbance

    # Home Position
    'HOME_POSITION': {'x': 0.0, 'y': 0.0, 'z': 0.0},
    'TAKEOFF_ALTITUDE': 2.0,  # Home + 2m on z-axis

    # Video Stream
    'VIDEO_PORT': 8080,
    'VIDEO_JPEG_QUALITY': 85,

    # ROS2 Connection
    'ROS2_HOST': 'localhost',
    'ROS2_PORT': 9090,
    'ROS2_PUBLISH_INTERVAL': 0.033,  # 30Hz

    # Nimbus Hub (Central Communication)
    'HUB_URL': 'http://localhost:5000',
    'HUB_PUBLISH_INTERVAL': 0.01,  # 100Hz (10ms)

    # Camera
    'CAMERA_ROLL_FACTOR': -0.115,
    'CAMERA_PITCH_FACTOR': -0.1,
    'CAMERA_MANUAL_INCREMENT': 0.05,  # Radians per keypress
    'CAMERA_PITCH_STABILIZATION_GAIN': -0.8,  # Inverted pitch compensation (negative = counter drone pitch)
    'CAMERA_STABILIZATION_SMOOTHING': 0.3,  # Seconds to smooth camera movements
}

# World Objects (from mavic_2_pro.wbt)
WORLD_OBJECTS = {
    'car': {
        'type': 'TeslaModel3Simple',
        'position': {'x': -41.5139, 'y': 4.34169, 'z': 0.31},
        'rotation': {'x': 0, 'y': 0, 'z': 1, 'angle': -0.2618053071795865}
    },
    'bench': {
        'type': 'Bench',
        'position': {'x': -23.255, 'y': -2.62401, 'z': 0},
        'rotation': {'x': 0, 'y': 0, 'z': 1, 'angle': 1.0472}
    },
    'slide': {
        'type': 'Slide',
        'position': {'x': -11.29, 'y': 5.63, 'z': 0},
        'rotation': {'x': 0, 'y': 0, 'z': 1, 'angle': 0}
    },
    'human': {
        'type': 'Pedestrian',
        'position': {'x': -8.89, 'y': -6.67, 'z': 1.27},
        'rotation': {'x': 0, 'y': 0, 'z': 1, 'angle': 1.5708}
    },
    'cabinet': {
        'type': 'Cabinet',
        'position': {'x': -31.795, 'y': 13.8306, 'z': 0},
        'rotation': {'x': 0, 'y': 0, 'z': 1, 'angle': -2.094395307179586}
    },
    'cardboard_box': {
        'type': 'CardboardBox',
        'position': {'x': -0.730157, 'y': -1.22891, 'z': 0.3},
        'rotation': {'x': 0, 'y': 0, 'z': 1, 'angle': -1.8325953071795862}
    },
    'manhole': {
        'type': 'SquareManhole',
        'position': {'x': 0, 'y': 0, 'z': -0.03},
        'rotation': {'x': 0, 'y': 0, 'z': 1, 'angle': 0}
    }
}


class DroneState:
    """Tracks comprehensive drone state"""

    def __init__(self):
        # Home position (starting position)
        self.home_position = CONFIG['HOME_POSITION'].copy()

        # Current position (x, y, z in meters)
        self.position = {'x': 0.0, 'y': 0.0, 'z': 0.0}

        # Orientation (roll, pitch, yaw in radians)
        self.orientation = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}

        # Angular velocity (roll, pitch, yaw rates in rad/s)
        self.angular_velocity = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}

        # Linear velocity (x, y, z in m/s) - computed from position changes
        self.velocity = {'x': 0.0, 'y': 0.0, 'z': 0.0}

        # Linear acceleration (x, y, z in m/s²) - computed from velocity changes
        self.acceleration = {'x': 0.0, 'y': 0.0, 'z': 0.0}

        # Previous values for derivative calculations
        self._prev_position = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self._prev_velocity = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self._prev_time = 0.0

        # Flight mode
        self.mode = 'MANUAL'  # MANUAL, AUTO, RETURNING_HOME, LANDING, TAKEOFF
        self.target_altitude = CONFIG['TAKEOFF_ALTITUDE']

    def update(self, x, y, z, roll, pitch, yaw, roll_vel, pitch_vel, yaw_vel, current_time):
        """Update drone state with current sensor readings"""
        # Update position
        self.position['x'] = x
        self.position['y'] = y
        self.position['z'] = z

        # Update orientation
        self.orientation['roll'] = roll
        self.orientation['pitch'] = pitch
        self.orientation['yaw'] = yaw

        # Update angular velocity
        self.angular_velocity['roll'] = roll_vel
        self.angular_velocity['pitch'] = pitch_vel
        self.angular_velocity['yaw'] = yaw_vel

        # Calculate linear velocity (if enough time has passed)
        dt = current_time - self._prev_time
        if dt > 0.001:  # Avoid division by zero
            self.velocity['x'] = (x - self._prev_position['x']) / dt
            self.velocity['y'] = (y - self._prev_position['y']) / dt
            self.velocity['z'] = (z - self._prev_position['z']) / dt

            # Calculate acceleration
            self.acceleration['x'] = (self.velocity['x'] - self._prev_velocity['x']) / dt
            self.acceleration['y'] = (self.velocity['y'] - self._prev_velocity['y']) / dt
            self.acceleration['z'] = (self.velocity['z'] - self._prev_velocity['z']) / dt

            # Update previous values
            self._prev_position = self.position.copy()
            self._prev_velocity = self.velocity.copy()
            self._prev_time = current_time

    def distance_to_home(self):
        """Calculate distance from current position to home"""
        dx = self.position['x'] - self.home_position['x']
        dy = self.position['y'] - self.home_position['y']
        dz = self.position['z'] - self.home_position['z']
        return (dx**2 + dy**2 + dz**2)**0.5

    def get_state_dict(self):
        """Get complete state as dictionary for logging/publishing"""
        return {
            'position': self.position.copy(),
            'orientation': self.orientation.copy(),
            'angular_velocity': self.angular_velocity.copy(),
            'velocity': self.velocity.copy(),
            'acceleration': self.acceleration.copy(),
            'mode': self.mode,
            'target_altitude': self.target_altitude,
            'distance_to_home': self.distance_to_home()
        }


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
    """MJPEG video streaming server with async encoding"""

    def __init__(self, port=8080):
        self.port = port
        self.server = None
        self.server_thread = None
        self.encoder_thread = None
        self.running = False
        self.frame_queue = queue.Queue(maxsize=2)  # Limit queue size to avoid lag

    def start(self):
        """Start the HTTP server and encoding thread"""
        self.server = HTTPServer(('0.0.0.0', self.port), MJPEGStreamHandler)
        self.server.latest_frame = None
        self.running = True

        # Start encoding thread
        self.encoder_thread = threading.Thread(target=self._encoding_loop, daemon=True)
        self.encoder_thread.start()

        # Start HTTP server thread
        self.server_thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.server_thread.start()

        print(f"MJPEG video stream started on http://localhost:{self.port}/video")

    def queue_frame(self, image_data, width, height):
        """Queue a frame for encoding (non-blocking, called from main loop)"""
        if not self.running:
            return

        try:
            # Non-blocking put - drops frame if queue full
            self.frame_queue.put_nowait((image_data, width, height))
        except queue.Full:
            pass  # Drop frame if encoder is behind

    def _encoding_loop(self):
        """Background thread that encodes frames"""
        while self.running:
            try:
                # Get frame from queue with timeout
                image_data, width, height = self.frame_queue.get(timeout=0.1)

                if Image is None:
                    continue

                # Convert BGRA to RGB
                img = Image.frombytes('RGBA', (width, height), image_data)
                img = img.convert('RGB')

                # Encode as JPEG
                buffer = io.BytesIO()
                img.save(buffer, format='JPEG', quality=85)
                self.server.latest_frame = buffer.getvalue()

            except queue.Empty:
                continue
            except Exception as e:
                print(f"Error encoding frame: {e}")

    def stop(self):
        """Stop the HTTP server and encoding thread"""
        self.running = False

        if self.encoder_thread and self.encoder_thread.is_alive():
            self.encoder_thread.join(timeout=1)

        if self.server:
            self.server.shutdown()

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
    def __init__(self):
        Robot.__init__(self)

        self.time_step = int(self.getBasicTimeStep())

        # Initialize drone state tracker
        self.state = DroneState()

        # Initialize devices
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

        # Camera manual control state
        self.camera_manual_roll = 0.0
        self.camera_manual_pitch = 0.0
        self.camera_stabilization_enabled = True

        # Camera smoothed pitch stabilization
        self.camera_target_pitch = 0.0
        self.camera_current_pitch = 0.0

        # Smooth disturbance control (current and target values)
        self.current_roll_disturbance = 0.0
        self.current_pitch_disturbance = 0.0
        self.current_yaw_disturbance = 0.0
        self.target_roll_disturbance = 0.0
        self.target_pitch_disturbance = 0.0
        self.target_yaw_disturbance = 0.0

        # PID derivative tracking
        self.prev_altitude = 0.0

        self.front_left_motor = self.getDevice("front left propeller")
        self.front_right_motor = self.getDevice("front right propeller")
        self.rear_left_motor = self.getDevice("rear left propeller")
        self.rear_right_motor = self.getDevice("rear right propeller")

        motors = [self.front_left_motor, self.front_right_motor,
                  self.rear_left_motor, self.rear_right_motor]
        for motor in motors:
            motor.setPosition(float('inf'))
            motor.setVelocity(1.0)

        # Initialize ROS2 bridge
        self.ros2 = ROS2Bridge(host=CONFIG['ROS2_HOST'], port=CONFIG['ROS2_PORT'])
        self.ros2_connected = False

        # Initialize video stream server
        self.video_server = VideoStreamServer(port=CONFIG['VIDEO_PORT'])

    def go_home(self):
        """Command drone to return to home position and land"""
        print("Returning home...")
        self.state.mode = 'RETURNING_HOME'
        # Target position will be home
        # Will transition to LANDING when close to home

    def land(self):
        """Command drone to land at current position"""
        print("Landing...")
        self.state.mode = 'LANDING'
        self.state.target_altitude = 0.0

    def takeoff(self, altitude=None):
        """Command drone to take off to specified altitude"""
        if altitude is None:
            altitude = CONFIG['TAKEOFF_ALTITUDE']
        print(f"Taking off to {altitude}m...")
        self.state.mode = 'TAKEOFF'
        self.state.target_altitude = altitude

    def setup_ros2_topics(self):
        """Setup ROS2 topics for publishing drone state"""
        if not self.ros2_connected:
            return

        # Advertise topics
        self.ros2.advertise("/drone/pose", "geometry_msgs/PoseStamped")
        self.ros2.advertise("/drone/velocity", "geometry_msgs/TwistStamped")
        self.ros2.advertise("/drone/acceleration", "geometry_msgs/AccelStamped")
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

        # Publish velocity (with computed linear velocity from state)
        velocity_msg = {
            "header": {
                "stamp": {"sec": int(self.getTime()), "nanosec": 0},
                "frame_id": "world"
            },
            "twist": {
                "linear": {
                    "x": self.state.velocity['x'],
                    "y": self.state.velocity['y'],
                    "z": self.state.velocity['z']
                },
                "angular": {"x": roll_velocity, "y": pitch_velocity, "z": yaw_velocity}
            }
        }
        self.ros2.publish("/drone/velocity", velocity_msg)

        # Publish acceleration (computed from velocity changes)
        acceleration_msg = {
            "header": {
                "stamp": {"sec": int(self.getTime()), "nanosec": 0},
                "frame_id": "world"
            },
            "accel": {
                "linear": {
                    "x": self.state.acceleration['x'],
                    "y": self.state.acceleration['y'],
                    "z": self.state.acceleration['z']
                },
                "angular": {"x": 0.0, "y": 0.0, "z": 0.0}
            }
        }
        self.ros2.publish("/drone/acceleration", acceleration_msg)

        # Publish altitude
        altitude_msg = {"data": altitude}
        self.ros2.publish("/drone/altitude", altitude_msg)

        # Compute facing vector from yaw (forward direction in XY plane)
        import math
        facing_x = math.cos(yaw)
        facing_y = math.sin(yaw)
        facing_z = 0.0  # Assuming level flight for forward direction

        # Publish status (flight mode, distance to home, facing vector, etc.)
        status_msg = {
            "data": f"Mode: {self.state.mode} | "
                   f"Alt: {altitude:.2f}m | "
                   f"Home Dist: {self.state.distance_to_home():.2f}m | "
                   f"Target Alt: {self.state.target_altitude:.2f}m | "
                   f"Facing: ({facing_x:.2f}, {facing_y:.2f}, {facing_z:.2f})"
        }
        self.ros2.publish("/drone/status", status_msg)

    def publish_to_hub(self):
        """Publish complete drone state to Nimbus Hub (fast HTTP)"""
        try:
            # Compute facing vector from yaw
            import math
            yaw = self.state.orientation['yaw']
            facing_x = math.cos(yaw)
            facing_y = math.sin(yaw)
            facing_z = 0.0

            # Build complete state dictionary
            state_dict = {
                'position': self.state.position.copy(),
                'orientation': self.state.orientation.copy(),
                'velocity': self.state.velocity.copy(),
                'acceleration': self.state.acceleration.copy(),
                'angular_velocity': self.state.angular_velocity.copy(),
                'facing_vector': {'x': facing_x, 'y': facing_y, 'z': facing_z},
                'mode': self.state.mode,
                'target_altitude': self.state.target_altitude,
                'distance_to_home': self.state.distance_to_home(),
                'connected': True,
                'timestamp': self.getTime(),
                'world_objects': WORLD_OBJECTS  # Ground truth object positions for TRIG depth method
            }

            # Non-blocking POST to hub
            requests.post(
                f"{CONFIG['HUB_URL']}/drone/state",
                json=state_dict,
                timeout=0.005  # 5ms timeout
            )

        except Exception as e:
            # Silently fail - don't block control loop
            pass

    def run(self):
        print("=" * 60)
        print("MAVIC 2 PRO - ROS2 CONTROLLER")
        print("=" * 60)
        print(f"Home Position: ({CONFIG['HOME_POSITION']['x']}, "
              f"{CONFIG['HOME_POSITION']['y']}, {CONFIG['HOME_POSITION']['z']})")
        print(f"Takeoff Altitude: {CONFIG['TAKEOFF_ALTITUDE']}m")
        print(f"World Objects Loaded: {len(WORLD_OBJECTS)}")
        for obj_name, obj_data in WORLD_OBJECTS.items():
            pos = obj_data['position']
            print(f"  - {obj_name}: ({pos['x']:.2f}, {pos['y']:.2f}, {pos['z']:.2f})")
        print("=" * 60)

        target_pos = None
        flying_to_target = False

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
        print("- Arrow Up: move forward")
        print("- Arrow Down: move backward")
        print("- Arrow Right: strafe right")
        print("- Arrow Left: strafe left")
        print("- Numpad 1: decrease altitude")
        print("- Numpad 3: increase altitude")
        print("- Numpad 7: yaw left")
        print("- Numpad 9: yaw right")
        print("- Numpad 2: camera tilt down")
        print("- Numpad 8: camera tilt up")
        print("- Numpad 4: camera pan left")
        print("- Numpad 6: camera pan right")
        print("- H: return home and land")
        print("- L: land at current position")

        last_publish_time = 0
        last_hub_publish_time = 0
        publish_interval = CONFIG['ROS2_PUBLISH_INTERVAL']
        hub_publish_interval = CONFIG['HUB_PUBLISH_INTERVAL']

        while self.step(self.time_step) != -1:
            time_now = self.getTime()

            # Read sensor data
            roll, pitch, yaw = self.imu.getRollPitchYaw()
            x_pos, y_pos, altitude = self.gps.getValues()
            roll_velocity, pitch_velocity, yaw_velocity = self.gyro.getValues()

            # Update drone state
            self.state.update(x_pos, y_pos, altitude, roll, pitch, yaw,
                            roll_velocity, pitch_velocity, yaw_velocity, time_now)

            # LED blinking
            led_state = int(time_now) % 2
            self.front_left_led.set(1 if led_state else 0)
            self.front_right_led.set(0 if led_state else 1)

            # Camera control with inverted pitch stabilization
            dt = self.time_step / 1000.0

            if self.camera_stabilization_enabled:
                # Calculate target pitch: invert drone pitch + velocity damping + manual offset
                self.camera_target_pitch = (
                    CONFIG['CAMERA_PITCH_STABILIZATION_GAIN'] * pitch +  # Invert drone pitch angle
                    CONFIG['CAMERA_PITCH_FACTOR'] * pitch_velocity +      # Velocity damping
                    self.camera_manual_pitch                              # Manual adjustment
                )

                # Smooth interpolation toward target pitch
                smoothing_rate = 1.0 / CONFIG['CAMERA_STABILIZATION_SMOOTHING']
                max_pitch_change = smoothing_rate * dt

                pitch_diff = self.camera_target_pitch - self.camera_current_pitch
                if abs(pitch_diff) <= max_pitch_change:
                    self.camera_current_pitch = self.camera_target_pitch
                else:
                    self.camera_current_pitch += max_pitch_change if pitch_diff > 0 else -max_pitch_change

                # Apply smoothed pitch and roll stabilization
                self.camera_pitch_motor.setPosition(self.camera_current_pitch)
                self.camera_roll_motor.setPosition(
                    CONFIG['CAMERA_ROLL_FACTOR'] * roll_velocity + self.camera_manual_roll
                )
            else:
                # Pure manual control
                self.camera_roll_motor.setPosition(self.camera_manual_roll)
                self.camera_pitch_motor.setPosition(self.camera_manual_pitch)

            # Handle keyboard input - set target disturbances
            self.target_roll_disturbance = 0.0
            self.target_pitch_disturbance = 0.0
            self.target_yaw_disturbance = 0.0

            key = self.keyboard.getKey()
            while key >= 0:
                # Arrow keys - Strafe controls
                if key == Keyboard.UP:
                    self.target_pitch_disturbance = CONFIG['PITCH_DISTURBANCE_FORWARD']
                elif key == Keyboard.DOWN:
                    self.target_pitch_disturbance = CONFIG['PITCH_DISTURBANCE_BACKWARD']
                elif key == Keyboard.RIGHT:
                    self.target_roll_disturbance = CONFIG['ROLL_DISTURBANCE_RIGHT']
                elif key == Keyboard.LEFT:
                    self.target_roll_disturbance = CONFIG['ROLL_DISTURBANCE_LEFT']

                # Numpad altitude control (multiple key codes for NumLock on/off)
                elif key == 321 or key == ord('1'):  # Numpad 1
                    self.state.target_altitude -= CONFIG['ALTITUDE_INCREMENT']
                    print(f"Target altitude: {self.state.target_altitude:.2f}m")
                elif key == 323 or key == ord('3'):  # Numpad 3
                    self.state.target_altitude += CONFIG['ALTITUDE_INCREMENT']
                    print(f"Target altitude: {self.state.target_altitude:.2f}m")

                # Numpad yaw control
                elif key == 327 or key == ord('7'):  # Numpad 7
                    self.target_yaw_disturbance = CONFIG['YAW_DISTURBANCE_LEFT']
                elif key == 329 or key == ord('9'):  # Numpad 9
                    self.target_yaw_disturbance = CONFIG['YAW_DISTURBANCE_RIGHT']

                # Numpad camera pitch control
                elif key == 322 or key == ord('2'):  # Numpad 2
                    self.camera_manual_pitch -= CONFIG['CAMERA_MANUAL_INCREMENT']
                    print(f"Camera pitch: {self.camera_manual_pitch:.3f} rad")
                elif key == 328 or key == ord('8'):  # Numpad 8
                    self.camera_manual_pitch += CONFIG['CAMERA_MANUAL_INCREMENT']
                    print(f"Camera pitch: {self.camera_manual_pitch:.3f} rad")

                # Numpad camera roll control
                elif key == 324 or key == ord('4'):  # Numpad 4
                    self.camera_manual_roll += CONFIG['CAMERA_MANUAL_INCREMENT']
                    print(f"Camera roll: {self.camera_manual_roll:.3f} rad")
                elif key == 326 or key == ord('6'):  # Numpad 6
                    self.camera_manual_roll -= CONFIG['CAMERA_MANUAL_INCREMENT']
                    print(f"Camera roll: {self.camera_manual_roll:.3f} rad")

                # Debug: print unknown keys
                elif key not in [Keyboard.UP, Keyboard.DOWN, Keyboard.LEFT, Keyboard.RIGHT]:
                    if key > 0 and key < 500:  # Reasonable key range
                        print(f"DEBUG: Unknown key pressed: {key}")

                # Other commands
                if key == ord('H'):
                    self.go_home()
                elif key == ord('L'):
                    self.land()

                key = self.keyboard.getKey()
            
            # Get AI state from hub
            ai_state = self.get_ai_state()

            if ai_state is not None:
                # The AI result data is inside ai_state["ai_results"]
                results = ai_state.get("ai_results", {})

                # Check if depth info (object + drone positions) is available
                if results.get("processing_status") == "object_detected_with_depth":
                    if results.get("intent") == "go":
                        flying_to_target = True
                        depth_result = results.get("depth_result", {})
                        target_pos = depth_result.get("object_position")
                    else:
                        flying_to_target = False
            
            if flying_to_target:
                if flying_to_target and target_pos is not None:
                    print("in core logic")
                    # --- PID PARAMETERS ---
                    Kp = 0.4
                    Ki = 0.0
                    Kd = 0.15
                    SCALE = 1.0        # scale PID output to disturbance range
                    MAX_DISTURB = 4.0  # maximum allowed pitch/roll disturbance

                    # Initialize PID memory if missing
                    if not hasattr(self, 'pid_prev_error'):
                        self.pid_prev_error = {'x': 0.0, 'y': 0.0}
                        self.pid_integral = {'x': 0.0, 'y': 0.0}

                    dt = self.time_step / 1000.0  # convert ms to seconds

                    # --- POSITION ERROR (desired - current) ---
                    error_x = target_pos['x'] - self.state.position['x']   # forward/back
                    error_y = target_pos['y'] - self.state.position['y']   # left/right

                    # YAW CORRECTION
                    import math

                    print("yaw: "+str(yaw))
                    cos_yaw = math.cos(-yaw)
                    sin_yaw = math.sin(-yaw)

                    local_error_x = cos_yaw*error_x-sin_yaw*error_y
                    local_error_y = sin_yaw * error_x + cos_yaw * error_y

                    error_x = local_error_x
                    error_y = local_error_y

                    # --- INTEGRAL TERM ---
                    self.pid_integral['x'] += error_x * dt
                    self.pid_integral['y'] += error_y * dt

                    # --- DERIVATIVE TERM ---
                    deriv_x = (error_x - self.pid_prev_error['x']) / dt
                    deriv_y = (error_y - self.pid_prev_error['y']) / dt

                    # --- PID OUTPUT ---
                    control_x = Kp * error_x + Ki * self.pid_integral['x'] + Kd * deriv_x
                    control_y = Kp * error_y + Ki * self.pid_integral['y'] + Kd * deriv_y

                    # --- CLAMP OUTPUTS (to avoid flipping) ---
                    control_x = max(-MAX_DISTURB, min(MAX_DISTURB, control_x))
                    control_y = max(-MAX_DISTURB, min(MAX_DISTURB, control_y))

                    # --- SAVE FOR NEXT LOOP ---
                    self.pid_prev_error['x'] = error_x
                    self.pid_prev_error['y'] = error_y

                    # --- APPLY TO DISTURBANCES ---
                    # Note: x → pitch (forward/back), y → roll (left/right)
                    self.target_pitch_disturbance = -control_x * SCALE   # Negative: forward in world space usually = negative pitch
                    self.target_roll_disturbance = control_y * SCALE

                    print(f"[PID] target_pitch={self.target_pitch_disturbance:.2f}, target_roll={self.target_roll_disturbance:.2f}")

            # Smooth ramping of disturbances (interpolate current toward target)
            dt = self.time_step / 1000.0  # Convert ms to seconds
            ramp_rate = 1.0 / CONFIG['DISTURBANCE_RAMP_TIME']  # Units per second

            # Calculate maximum change allowed this frame
            max_change = ramp_rate * dt

            # Ramp roll disturbance
            roll_diff = self.target_roll_disturbance - self.current_roll_disturbance
            if abs(roll_diff) <= max_change:
                self.current_roll_disturbance = self.target_roll_disturbance
            else:
                self.current_roll_disturbance += max_change if roll_diff > 0 else -max_change

            # Ramp pitch disturbance
            pitch_diff = self.target_pitch_disturbance - self.current_pitch_disturbance
            if abs(pitch_diff) <= max_change:
                self.current_pitch_disturbance = self.target_pitch_disturbance
            else:
                self.current_pitch_disturbance += max_change if pitch_diff > 0 else -max_change

            # Ramp yaw disturbance
            yaw_diff = self.target_yaw_disturbance - self.current_yaw_disturbance
            if abs(yaw_diff) <= max_change:
                self.current_yaw_disturbance = self.target_yaw_disturbance
            else:
                self.current_yaw_disturbance += max_change if yaw_diff > 0 else -max_change

            # Use smoothed values for control
            roll_disturbance = self.current_roll_disturbance
            pitch_disturbance = self.current_pitch_disturbance
            yaw_disturbance = self.current_yaw_disturbance

            # PID control with damping
            # Roll/Pitch: stronger P gain + velocity damping + disturbance input
            print("roll_disturbance: "+str(roll_disturbance))
            print("pitch_disturbance: "+str(pitch_disturbance))
            print("target object coordinates: "+str(target_pos))
            roll_input = CONFIG['K_ROLL_P'] * clamp(roll, -1.0, 1.0) + roll_velocity + roll_disturbance
            pitch_input = CONFIG['K_PITCH_P'] * clamp(pitch, -1.0, 1.0) + pitch_velocity + pitch_disturbance
            yaw_input = yaw_disturbance

            # Vertical: PD controller (P + D term for damping)
            altitude_error = self.state.target_altitude - altitude + CONFIG['K_VERTICAL_OFFSET']
            altitude_rate = (altitude - self.prev_altitude) / (self.time_step / 1000.0)  # Derivative
            self.prev_altitude = altitude

            clamped_altitude_error = clamp(altitude_error, -1.0, 1.0)
            vertical_input = (CONFIG['K_VERTICAL_P'] * pow(clamped_altitude_error, 3.0) -
                            CONFIG['K_VERTICAL_D'] * altitude_rate)

            # Motor control with differential limiting
            base_thrust = CONFIG['K_VERTICAL_THRUST'] + vertical_input

            # Calculate raw motor commands
            front_left_motor_input = base_thrust - roll_input + pitch_input - yaw_input
            front_right_motor_input = base_thrust + roll_input + pitch_input + yaw_input
            rear_left_motor_input = base_thrust - roll_input - pitch_input + yaw_input
            rear_right_motor_input = base_thrust + roll_input - pitch_input - yaw_input

            # Limit differential from base thrust to prevent overcorrection
            max_diff = CONFIG['MAX_MOTOR_DIFFERENTIAL']
            front_left_motor_input = clamp(front_left_motor_input, base_thrust - max_diff, base_thrust + max_diff)
            front_right_motor_input = clamp(front_right_motor_input, base_thrust - max_diff, base_thrust + max_diff)
            rear_left_motor_input = clamp(rear_left_motor_input, base_thrust - max_diff, base_thrust + max_diff)
            rear_right_motor_input = clamp(rear_right_motor_input, base_thrust - max_diff, base_thrust + max_diff)

            # Ensure motors stay within physical limits (positive values)
            front_left_motor_input = max(0.1, front_left_motor_input)
            front_right_motor_input = max(0.1, front_right_motor_input)
            rear_left_motor_input = max(0.1, rear_left_motor_input)
            rear_right_motor_input = max(0.1, rear_right_motor_input)

            self.front_left_motor.setVelocity(front_left_motor_input)
            self.front_right_motor.setVelocity(-front_right_motor_input)
            self.rear_left_motor.setVelocity(-rear_left_motor_input)
            self.rear_right_motor.setVelocity(rear_right_motor_input)

            # Queue camera frame for async encoding (non-blocking)
            image = self.camera.getImage()
            if image is not None:
                self.video_server.queue_frame(
                    bytes(image),
                    self.camera.getWidth(),
                    self.camera.getHeight()
                )

            # Publish to Nimbus Hub at high frequency (100Hz)
            if (time_now - last_hub_publish_time) >= hub_publish_interval:
                self.publish_to_hub()
                last_hub_publish_time = time_now

            # Publish to ROS2 at lower frequency (30Hz) - for legacy systems
            if self.ros2_connected and (time_now - last_publish_time) >= publish_interval:
                self.publish_drone_state(x_pos, y_pos, altitude, roll, pitch, yaw,
                                        roll_velocity, pitch_velocity, yaw_velocity)
                self.publish_camera_image()
                last_publish_time = time_now

        # Cleanup
        self.video_server.stop()
        if self.ros2_connected:
            self.ros2.disconnect()

    def get_ai_state(self):
        """Fetch AI results from hub"""
        try:
            response = requests.get(f"{CONFIG['HUB_URL']}/api/debug/ai_state", timeout=0.01)
            if response.status_code == 200:
                return response.json()
        except Exception:
            pass
        return None


controller = Mavic2ProROS2Controller()
controller.run()