#!/usr/bin/env python3
"""
Test script for object detection integration with AI.py
Tests the video pipeline with bounding box overlays
"""

import time
import sys
import os

# Add project root to path
project_root = os.path.dirname(os.path.abspath(__file__))
sys.path.append(project_root)

from AI import NimbusAISystem, GLOBAL_OBJECT, GLOBAL_INTENT, GLOBAL_STATE_ACTIVE

def test_object_detection():
    """Test the video pipeline with object detection"""
    print("🧪 Testing Object Detection Integration")
    print("=" * 50)
    
    # Set global variables for testing
    import AI
    AI.GLOBAL_OBJECT = "chair"  # Test with chair detection
    AI.GLOBAL_INTENT = "go"
    AI.GLOBAL_STATE_ACTIVE = True
    
    print(f"Target Object: {AI.GLOBAL_OBJECT}")
    print(f"Intent: {AI.GLOBAL_INTENT}")
    print(f"State Active: {AI.GLOBAL_STATE_ACTIVE}")
    print()
    
    # Initialize AI system
    ai_system = NimbusAISystem()
    
    try:
        print("🚀 Starting AI system...")
        print("📹 Make sure to:")
        print("  1. Have ROS2 container running (docker-compose up -d ros2-central)")
        print("  2. Have camera publisher running")
        print("  3. Point camera at a chair to test detection")
        print()
        print("🎮 Controls: 'q' = quit, 's' = stats")
        print()
        
        ai_system.run()
        
    except KeyboardInterrupt:
        print("\n🛑 Test interrupted by user")
    except Exception as e:
        print(f"\n❌ Test failed: {e}")
    finally:
        ai_system.stop()
        print("✅ Test completed")

if __name__ == "__main__":
    test_object_detection()