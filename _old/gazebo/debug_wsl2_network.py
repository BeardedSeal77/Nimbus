#!/usr/bin/env python3
"""
Debug WSL2 Network Connection
Test if we can reach WSL2 Gazebo from Windows
"""

import socket
import requests
import subprocess
import time

def test_wsl2_connection():
    print("=" * 60)
    print("WSL2 NETWORK CONNECTION DEBUGGER")
    print("=" * 60)

    # WSL2 IP from earlier
    wsl2_ip = "172.29.75.106"
    docker_ip = "192.168.8.102"

    print(f"Testing connections from Windows...")
    print(f"WSL2 Gazebo IP: {wsl2_ip}")
    print(f"Docker ROS2 IP: {docker_ip}")
    print()

    # Test 1: Ping WSL2
    print("1. Testing ping to WSL2...")
    try:
        result = subprocess.run(['ping', '-n', '1', wsl2_ip],
                              capture_output=True, text=True, timeout=5)
        if result.returncode == 0:
            print(f"[OK] Can ping WSL2 at {wsl2_ip}")
        else:
            print(f"[FAIL] Cannot ping WSL2 at {wsl2_ip}")
    except Exception as e:
        print(f"[FAIL] Ping test failed: {e}")

    # Test 2: Test DDS ports on WSL2
    print("\n2. Testing DDS ports on WSL2...")
    dds_ports = [7400, 7401, 7411]  # Common DDS ports

    for port in dds_ports:
        print(f"Testing {wsl2_ip}:{port}...")
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(2)
        try:
            result = sock.connect_ex((wsl2_ip, port))
            if result == 0:
                print(f"[OK] Port {port} is open on WSL2")
            else:
                print(f"[FAIL] Port {port} is closed/filtered on WSL2")
        except Exception as e:
            print(f"[FAIL] Error testing port {port}: {e}")
        finally:
            sock.close()

    # Test 3: Test Docker connection (we know this works)
    print(f"\n3. Testing Docker rosbridge connection...")
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(2)
        result = sock.connect_ex((docker_ip, 9090))
        if result == 0:
            print(f"[OK] Docker rosbridge at {docker_ip}:9090 is reachable")
        else:
            print(f"[FAIL] Docker rosbridge at {docker_ip}:9090 is not reachable")
        sock.close()
    except Exception as e:
        print(f"[FAIL] Error testing Docker: {e}")

    # Test 4: Check if WSL2 has any HTTP services
    print(f"\n4. Scanning for HTTP services on WSL2...")
    http_ports = [8080, 3000, 5000, 8000, 9090]

    for port in http_ports:
        try:
            response = requests.get(f"http://{wsl2_ip}:{port}", timeout=2)
            print(f"[OK] HTTP service found on port {port}: {response.status_code}")
        except requests.exceptions.ConnectionError:
            print(f"[FAIL] No HTTP service on port {port}")
        except Exception as e:
            print(f"[FAIL] Error testing HTTP port {port}: {e}")

    print("\n" + "=" * 60)
    print("ANALYSIS:")
    print("- If WSL2 ping works but DDS ports are closed:")
    print("  -> Gazebo ROS2 isn't exposing DDS properly")
    print("- If WSL2 ping fails:")
    print("  -> Network routing issue between Windows and WSL2")
    print("- If Docker works but WSL2 doesn't:")
    print("  -> Need to bridge WSL2 -> Docker instead of direct connection")
    print("=" * 60)

def check_wsl2_ip():
    """Try to get current WSL2 IP"""
    print("\nDetecting current WSL2 IP...")
    try:
        # Try to get WSL2 IP via WSL command
        result = subprocess.run(['wsl', 'hostname', '-I'],
                              capture_output=True, text=True, timeout=5)
        if result.returncode == 0:
            wsl2_ip = result.stdout.strip().split()[0]
            print(f"Current WSL2 IP: {wsl2_ip}")
            return wsl2_ip
    except Exception as e:
        print(f"[FAIL] Could not detect WSL2 IP: {e}")

    return "172.29.75.106"  # Fallback to known IP

if __name__ == "__main__":
    current_wsl2_ip = check_wsl2_ip()
    test_wsl2_connection()