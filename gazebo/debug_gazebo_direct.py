#!/usr/bin/env python3
"""
Debug Gazebo Direct Connection
Connects directly to WSL2 Gazebo ROS2 system to see what topics exist
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import time
import os

class GazeboTopicDebugger(Node):
    def __init__(self):
        super().__init__('gazebo_topic_debugger')
        self.get_logger().info('🔍 Starting Gazebo Topic Debugger...')

        # Try to subscribe to potential Gazebo topics
        self.subscriptions = []

        # Test topics
        test_topics = [
            ('/tello/status', String),
            ('/tello/cmd_vel', Twist),
            # Add more as needed
        ]

        for topic_name, msg_type in test_topics:
            try:
                sub = self.create_subscription(
                    msg_type,
                    topic_name,
                    lambda msg, topic=topic_name: self.topic_callback(msg, topic),
                    10
                )
                self.subscriptions.append(sub)
                self.get_logger().info(f'📡 Subscribed to {topic_name}')
            except Exception as e:
                self.get_logger().error(f'❌ Failed to subscribe to {topic_name}: {e}')

    def topic_callback(self, msg, topic_name):
        self.get_logger().info(f'📨 Received on {topic_name}: {msg}')

def main():
    print("=" * 60)
    print("🐛 GAZEBO DIRECT ROS2 TOPIC DEBUGGER")
    print("Connecting directly to WSL2 Gazebo ROS2 system")
    print("=" * 60)

    # Set environment to connect to WSL2 Gazebo
    print("🔧 Setting ROS2 environment...")

    # Set ROS2 environment to connect to WSL2 (dynamic IP)
    # You might need to adjust this IP based on your WSL2 setup
    wsl2_ip = "172.29.75.106"  # Your WSL2 IP from earlier

    os.environ['ROS_DOMAIN_ID'] = '0'
    os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'

    # Create FastRTPS profile to connect to WSL2
    fastrtps_config = f'''<?xml version="1.0" encoding="UTF-8"?>
<profiles>
  <participant profile_name="default_participant" is_default_profile="true">
    <rtps>
      <builtin>
        <discovery_config>
          <static_edp_xml_config>
            <participant>
              <name>WSL2Gazebo</name>
              <unicast_locator address="{wsl2_ip}" port="7400"/>
            </participant>
          </static_edp_xml_config>
        </discovery_config>
      </builtin>
    </rtps>
  </participant>
</profiles>'''

    # Write config to temp file
    config_path = os.path.join(os.environ['TEMP'], 'fastrtps_wsl2.xml')
    with open(config_path, 'w') as f:
        f.write(fastrtps_config)

    os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = config_path

    print(f"📍 Targeting WSL2 at: {wsl2_ip}")
    print(f"🔧 FastRTPS config: {config_path}")

    try:
        # Initialize ROS2
        rclpy.init()

        # Create debugger node
        debugger = GazeboTopicDebugger()

        print("\n🎯 Attempting to connect to WSL2 Gazebo...")
        print("⏳ Listening for topics for 15 seconds...")
        print("(Run this while Gazebo is active in WSL2)")

        # Spin for a while to see if we get any messages
        start_time = time.time()
        while time.time() - start_time < 15.0:
            rclpy.spin_once(debugger, timeout_sec=1.0)

            # List available topics every 5 seconds
            if int(time.time() - start_time) % 5 == 0:
                print(f"\n📋 Available topics at {int(time.time() - start_time)}s:")
                # This requires ROS2 CLI tools, might not work from Windows
                # topics = debugger.get_topic_names_and_types()
                # for topic, types in topics:
                #     print(f"  - {topic}: {types}")

        print("\n🏁 Debug complete!")

    except Exception as e:
        print(f"❌ Error: {e}")
        print("\nℹ️  Note: This script needs ROS2 installed on Windows")
        print("Alternative: Run this directly in WSL2 instead")

    finally:
        try:
            rclpy.shutdown()
        except:
            pass

if __name__ == "__main__":
    main()