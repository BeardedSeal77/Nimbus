#!/usr/bin/env python3
"""
Drone Main Application
Handles drone control, flight operations, video capture, and ROS2 publishing for the Nimbus system.
"""


import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import CommandLong
from mavros_msgs.srv import CommandBool
import math

class DroneNode(Node):
    def __init__(self):
        super().__init__('drone_node')
        self.command_sub = self.create_subscription(
            String,
            '/drone_command',
            self.command_callback,
            10
        )
        self.status_pub = self.create_publisher(
            String,
            '/drone_status',
            10
        )
        self.position_pub = self.create_publisher(
            PoseStamped,
            '/mavros/setpoint_position/local',
            10
        )
        self.command_pub = self.create_publisher(
            CommandLong,
            '/mavros/cmd/command',
            10
        )
        self.arming_client = self.create_client(
            CommandBool,
            '/mavros/cmd/arming'
        )
        self.current_position = [0.0, 0.0, 0.0]  # Origin (x, y, z)
        self.current_yaw = 0.0  # degrees
        self.get_logger().info('Drone node started, waiting for commands...')
        self.arm_drone()

    def arm_drone(self):
        """Arm the drone to enable movement."""
        if not self.arming_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Arming service not available')
            return
        request = CommandBool.Request()
        request.value = True
        future = self.arming_client.call_async(request)
        future.add_done_callback(self.arming_callback)

    def arming_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Drone armed successfully')
            else:
                self.get_logger().error('Failed to arm drone')
        except Exception as e:
            self.get_logger().error(f'Arming error: {str(e)}')

    def command_callback(self, msg):
        try:
            if msg.data.startswith('goto'):
                _, x_str, y_str = msg.data.split()
                target_x, target_y = float(x_str), float(y_str)
                self.get_logger().info(f'Received command to move to ({target_x}, {target_y})')
                
                # Calculate polar coordinates angle and distance
                delta_x = target_x - self.current_position[0]
                delta_y = target_y - self.current_position[1]
                distance = math.sqrt(delta_x**2 + delta_y**2)
                target_angle = math.degrees(math.atan2(delta_y, delta_x))
                
                # Calculate rotation
                angle_to_rotate = target_angle - self.current_yaw
                # Normalize angle to [-180, 180] to ensure the shortest rotation path, e.g. rotating 270° becomes -90°
                angle_to_rotate = ((angle_to_rotate + 180) % 360) - 180
                
                # Execute movement: rotate then move forward
                if abs(angle_to_rotate) > 0.01:  # Small threshold to avoid jitter
                    self.rotate(angle_to_rotate)
                    status_msg = String()
                    status_msg.data = f'Rotated {angle_to_rotate:.2f} degrees'
                    self.status_pub.publish(status_msg)
                    self.get_logger().info(f'Published status: {status_msg.data}')
                
                if distance > 0.01:  # Small threshold to avoid jitter
                    self.move_forward(target_x, target_y)
                    status_msg = String()
                    status_msg.data = f'Moved to ({target_x:.2f}, {target_y:.2f})'
                    self.status_pub.publish(status_msg)
                    self.get_logger().info(f'Published status: {status_msg.data}')
                
                # Update current position and yaw
                self.current_position = [target_x, target_y, self.current_position[2]]
                self.current_yaw = target_angle
                self.get_logger().info(f'Updated position: {self.current_position}, yaw: {self.current_yaw:.2f} degrees')
            
            elif msg.data == 'green_light':
                self.get_logger().info('Received green light from web UI, executing default sequence')
                moves = [
                    ('left', 0.5),
                    ('up', 0.5),
                    ('right', 0.5),
                    ('down', 0.5)
                ]
                for direction, distance in moves:
                    self.move(direction, distance)
                    status_msg = String()
                    status_msg.data = f'Moved {direction} {distance} meters'
                    self.status_pub.publish(status_msg)
                    self.get_logger().info(f'Published status: {status_msg.data}')
                    
        except ValueError as e:
            self.get_logger().error(f'Invalid command format: {msg.data}. Expected "goto x y" or "green_light"')

    def rotate(self, angle):
        """Send rotation command to the drone."""
        self.get_logger().info(f'Rotating {angle:.2f} degrees...')
        cmd = CommandLong()
        cmd.command = 115  # MAV_CMD_CONDITION_YAW
        cmd.param1 = abs(angle)  # Target angle
        cmd.param4 = -1 if angle < 0 else 1  # Direction: -1 for counterclockwise, 1 for clockwise
        self.command_pub.publish(cmd)

    def move_forward(self, target_x, target_y):
        """Move drone to the target position."""
        self.get_logger().info(f'Moving to ({target_x:.2f}, {target_y:.2f})...')
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"
        pose.pose.position.x = target_x
        pose.pose.position.y = target_y
        pose.pose.position.z = self.current_position[2]  # Maintain current altitude
        # Set orientation based on current yaw
        yaw_rad = math.radians(self.current_yaw)
        pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
        pose.pose.orientation.w = math.cos(yaw_rad / 2.0)
        self.position_pub.publish(pose)

    def move(self, direction, distance):
        """Move drone in specified direction for default sequence."""
        self.get_logger().info(f'Moving {direction} {distance} meters...')
        target_x, target_y = self.current_position[0], self.current_position[1]
        if direction == 'left':
            target_x -= distance
        elif direction == 'right':
            target_x += distance
        elif direction == 'up':
            target_y += distance
        elif direction == 'down':
            target_y -= distance
        
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"
        pose.pose.position.x = target_x
        pose.pose.position.y = target_y
        pose.pose.position.z = self.current_position[2]  # Maintain current altitude
        # Maintain current orientation
        yaw_rad = math.radians(self.current_yaw)
        pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
        pose.pose.orientation.w = math.cos(yaw_rad / 2.0)
        self.position_pub.publish(pose)
        
        # Update position
        self.current_position[0], self.current_position[1] = target_x, target_y

def main(args=None):
    rclpy.init(args=args)
    drone_node = DroneNode()
    rclpy.spin(drone_node)
    drone_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()