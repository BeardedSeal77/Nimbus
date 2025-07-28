#!/usr/bin/env python3
"""
Test RTAB-Map integration with phone camera
Interactive test for phone webcam → RTAB-Map SLAM
"""

import sys
import os
import cv2
import time

# Add paths for imports - go up to nimbus-ai/ directory
nimbus_ai_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
sys.path.append(nimbus_ai_root)

from helpers.phone_camera import PhoneCamera, get_common_phone_camera_urls
from classes.rtabmap_node import RTABMapNode

def test_phone_camera_connection():
    """Test phone camera connection options"""
    print("=" * 60)
    print("PHONE CAMERA CONNECTION TEST")
    print("=" * 60)
    
    camera = PhoneCamera()
    
    print("\n1. Auto-detection test:")
    print("-" * 30)
    success = camera.auto_connect()
    if success:
        info = camera.get_camera_info()
        print(f"[OK] Auto-connected: {info['connection_type']}")
        print(f"     Resolution: {info['width']}x{info['height']}")
        print(f"     FPS: {info['fps']}")
        camera.disconnect()
        return True
    else:
        print("[FAIL] Auto-detection failed")
    
    print("\n2. Manual IP camera test:")
    print("-" * 30)
    print("Common phone camera apps:")
    print("- DroidCam: Use port 4747")
    print("- IP Webcam: Use port 8080")
    print("- Make sure phone and PC are on same network")
    
    # Get user input for IP
    ip_url = input("\nEnter phone camera IP URL (or press Enter to skip): ").strip()
    if ip_url:
        success = camera.connect_ip_camera(ip_url)
        if success:
            info = camera.get_camera_info()
            print(f"[OK] Connected to: {ip_url}")
            print(f"     Resolution: {info['width']}x{info['height']}")
            camera.disconnect()
            return True
        else:
            print(f"[FAIL] Could not connect to: {ip_url}")
    
    return False

def test_rtabmap_slam():
    """Test RTAB-Map SLAM initialization"""
    print("\n" + "=" * 60)
    print("RTAB-MAP SLAM TEST")
    print("=" * 60)
    
    # Test SLAM node creation
    slam = RTABMapNode()
    print(f"[OK] RTAB-Map node created")
    
    # Test initialization
    print("Testing SLAM initialization...")
    success = slam.initialize_slam()
    if success:
        print("[OK] RTAB-Map initialized successfully")
    else:
        print("[FAIL] RTAB-Map initialization failed")
    
    return success

def test_integrated_system():
    """Test phone camera + RTAB-Map integration"""
    print("\n" + "=" * 60)
    print("INTEGRATED SYSTEM TEST")
    print("=" * 60)
    
    # Connect camera
    camera = PhoneCamera()
    print("Connecting to phone camera...")
    
    # Try auto-connect first
    if not camera.auto_connect():
        ip_url = input("Auto-connect failed. Enter IP camera URL: ").strip()
        if not ip_url or not camera.connect_ip_camera(ip_url):
            print("[FAIL] Could not connect to camera")
            return False
    
    print("[OK] Camera connected")
    
    # Initialize SLAM
    slam = RTABMapNode()
    if not slam.initialize_slam():
        print("[FAIL] SLAM initialization failed")
        camera.disconnect()
        return False
    
    print("[OK] SLAM initialized")
    
    # Test frame processing
    print("\nTesting frame processing (5 frames)...")
    for i in range(5):
        ret, frame = camera.get_frame()
        if ret:
            pose = slam.process_frame(frame)
            if pose:
                print(f"Frame {i+1}: [OK] Pose estimated")
                print(f"  Position: x={pose['position']['x']:.3f}, y={pose['position']['y']:.3f}, z={pose['position']['z']:.3f}")
            else:
                print(f"Frame {i+1}: [FAIL] No pose estimate")
        else:
            print(f"Frame {i+1}: [FAIL] Could not capture frame")
        
        time.sleep(0.5)
    
    # Cleanup
    camera.disconnect()
    print("\n[OK] Integration test completed")
    return True

def interactive_slam_demo():
    """Interactive SLAM demonstration"""
    print("\n" + "=" * 60)
    print("INTERACTIVE SLAM DEMO")
    print("=" * 60)
    print("This will show live camera feed with SLAM processing")
    print("Press 'q' to quit, 'r' to reset SLAM")
    
    # Setup camera
    camera = PhoneCamera()
    if not camera.auto_connect():
        ip_url = input("Enter phone camera IP URL: ").strip()
        if not camera.connect_ip_camera(ip_url):
            print("Could not connect to camera")
            return
    
    # Setup SLAM
    slam = RTABMapNode()
    if not slam.initialize_slam():
        print("Could not initialize SLAM")
        camera.disconnect()
        return
    
    # Start streaming
    camera.start_streaming()
    print("\nStarting live demo... Press 'q' to quit")
    
    try:
        frame_count = 0
        while True:
            frame = camera.get_latest_frame()
            if frame is not None:
                frame_count += 1
                
                # Process with SLAM
                pose = slam.process_frame(frame)
                
                # Draw info on frame
                if pose:
                    cv2.putText(frame, f"Frame: {frame_count}", (10, 30), 
                              cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    cv2.putText(frame, f"SLAM: OK", (10, 70), 
                              cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    
                    pos = pose['position']
                    cv2.putText(frame, f"Pos: ({pos['x']:.2f}, {pos['y']:.2f}, {pos['z']:.2f})", 
                              (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                else:
                    cv2.putText(frame, f"Frame: {frame_count}", (10, 30), 
                              cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                    cv2.putText(frame, f"SLAM: FAIL", (10, 70), 
                              cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                
                # Show frame
                cv2.imshow('Nimbus SLAM Demo', frame)
                
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('r'):
                    slam.reset_slam()
                    print("SLAM reset")
                    
            time.sleep(0.03)  # ~30 FPS
            
    except KeyboardInterrupt:
        print("\nDemo interrupted")
    
    # Cleanup
    camera.stop_streaming()
    camera.disconnect()
    cv2.destroyAllWindows()
    print("Demo ended")

def main():
    """Main test menu"""
    print("NIMBUS RTAB-MAP + PHONE CAMERA TEST SUITE")
    print("=" * 60)
    
    while True:
        print("\nSelect test:")
        print("1. Test phone camera connection")
        print("2. Test RTAB-Map SLAM")
        print("3. Test integrated system")
        print("4. Interactive SLAM demo")
        print("5. Show common camera URLs")
        print("0. Exit")
        
        choice = input("\nEnter choice: ").strip()
        
        if choice == '1':
            test_phone_camera_connection()
        elif choice == '2':
            test_rtabmap_slam()
        elif choice == '3':
            test_integrated_system()
        elif choice == '4':
            interactive_slam_demo()
        elif choice == '5':
            print("\nCommon phone camera URLs:")
            for url in get_common_phone_camera_urls():
                print(f"  {url}")
        elif choice == '0':
            break
        else:
            print("Invalid choice")
    
    print("Exiting...")

if __name__ == "__main__":
    main()