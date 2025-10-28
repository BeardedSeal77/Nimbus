#!/usr/bin/env python3
"""
Test direct connection to WSL2 Gazebo from Windows
This will try to connect to the WSL2 ROS2 system directly
"""

import os
import time
import subprocess
import tempfile

def create_fastrtps_config(wsl2_ip):
    """Create FastRTPS config to connect to WSL2"""
    config_content = f'''<?xml version="1.0" encoding="UTF-8"?>
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

    config_path = os.path.join(tempfile.gettempdir(), 'fastrtps_wsl2_debug.xml')
    with open(config_path, 'w') as f:
        f.write(config_content)
    return config_path

def test_ros2_connection(wsl2_ip):
    """Test ROS2 connection to WSL2"""
    print("=" * 60)
    print("TESTING ROS2 CONNECTION TO WSL2 GAZEBO")
    print("=" * 60)

    # Set up environment
    config_path = create_fastrtps_config(wsl2_ip)
    print(f"Created FastRTPS config: {config_path}")

    env = os.environ.copy()
    env['ROS_DOMAIN_ID'] = '0'
    env['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
    env['FASTRTPS_DEFAULT_PROFILES_FILE'] = config_path

    print(f"Target WSL2 IP: {wsl2_ip}")
    print("ROS2 Environment:")
    print(f"  ROS_DOMAIN_ID: {env.get('ROS_DOMAIN_ID')}")
    print(f"  RMW_IMPLEMENTATION: {env.get('RMW_IMPLEMENTATION')}")
    print()

    # Test 1: Try ros2 node list
    print("1. Testing ros2 node list...")
    try:
        result = subprocess.run(['ros2', 'node', 'list'],
                              capture_output=True, text=True, timeout=10, env=env)
        if result.returncode == 0 and result.stdout.strip():
            print("[OK] Found ROS2 nodes:")
            for line in result.stdout.strip().split('\n'):
                print(f"  - {line}")
        else:
            print("[FAIL] No ROS2 nodes found")
            if result.stderr:
                print(f"Error: {result.stderr}")
    except subprocess.TimeoutExpired:
        print("[FAIL] ros2 node list timed out")
    except Exception as e:
        print(f"[FAIL] Error running ros2 node list: {e}")

    print()

    # Test 2: Try ros2 topic list
    print("2. Testing ros2 topic list...")
    try:
        result = subprocess.run(['ros2', 'topic', 'list'],
                              capture_output=True, text=True, timeout=10, env=env)
        if result.returncode == 0 and result.stdout.strip():
            print("[OK] Found ROS2 topics:")
            topics = result.stdout.strip().split('\n')
            for topic in topics:
                print(f"  - {topic}")
                # Look for Gazebo-specific topics
                if 'tello' in topic or 'gazebo' in topic or 'gz' in topic:
                    print(f"    ^ GAZEBO TOPIC FOUND!")
        else:
            print("[FAIL] No ROS2 topics found")
            if result.stderr:
                print(f"Error: {result.stderr}")
    except subprocess.TimeoutExpired:
        print("[FAIL] ros2 topic list timed out")
    except Exception as e:
        print(f"[FAIL] Error running ros2 topic list: {e}")

    print()

    # Test 3: Try to echo a specific topic
    print("3. Testing ros2 topic echo /tello/status...")
    try:
        print("Listening for 5 seconds...")
        result = subprocess.run(['ros2', 'topic', 'echo', '/tello/status', '--once'],
                              capture_output=True, text=True, timeout=5, env=env)
        if result.returncode == 0 and result.stdout.strip():
            print("[OK] Received data from /tello/status:")
            print(f"  {result.stdout.strip()}")
        else:
            print("[FAIL] No data received from /tello/status")
            if result.stderr:
                print(f"Error: {result.stderr}")
    except subprocess.TimeoutExpired:
        print("[FAIL] ros2 topic echo timed out (normal if no publisher)")
    except Exception as e:
        print(f"[FAIL] Error running ros2 topic echo: {e}")

    print("\n" + "=" * 60)
    print("SUMMARY:")
    print("If you see topics above, ROS2 discovery is working!")
    print("If no topics, the FastRTPS static peer config may need adjustment")
    print("=" * 60)

def main():
    # Get WSL2 IP
    wsl2_ip = "172.29.75.106"  # From our network test

    print("Checking if ROS2 is installed on Windows...")
    try:
        result = subprocess.run(['ros2', '--version'],
                              capture_output=True, text=True, timeout=5)
        if result.returncode == 0:
            print(f"[OK] ROS2 found: {result.stdout.strip()}")
        else:
            print("[FAIL] ROS2 not found on Windows")
            print("You need ROS2 installed on Windows to use this test")
            return
    except Exception as e:
        print(f"[FAIL] ROS2 not available: {e}")
        print("You need ROS2 installed on Windows to use this test")
        return

    test_ros2_connection(wsl2_ip)

if __name__ == "__main__":
    main()