#!/bin/bash

# Nimbus Integration Test Script
# Tests ROS2 communication between Docker and WSL2

echo "=== Nimbus Integration Test ==="
echo ""

# Check ROS2 environment
echo "1. Checking ROS2 environment..."
source /opt/ros/humble/setup.bash
source ~/nimbus_ws/install/setup.bash

if [ -z "$ROS_DOMAIN_ID" ]; then
    export ROS_DOMAIN_ID=0
    echo "   Set ROS_DOMAIN_ID=0"
else
    echo "   ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
fi

# Test ROS2 discovery
echo ""
echo "2. Testing ROS2 node discovery..."
timeout 5 ros2 node list > /tmp/nodes.txt 2>/dev/null
if [ -s /tmp/nodes.txt ]; then
    echo "   ✓ Found ROS2 nodes:"
    cat /tmp/nodes.txt | sed 's/^/     /'
else
    echo "   ✗ No ROS2 nodes found - Docker containers may not be running"
fi

# Test topic discovery
echo ""
echo "3. Testing ROS2 topic discovery..."
timeout 5 ros2 topic list > /tmp/topics.txt 2>/dev/null
if [ -s /tmp/topics.txt ]; then
    echo "   ✓ Found ROS2 topics:"
    cat /tmp/topics.txt | sed 's/^/     /'
else
    echo "   ✗ No ROS2 topics found"
fi

# Test specific Docker services
echo ""
echo "4. Testing Docker service connectivity..."

# Check ROS WebSocket bridge
if curl -s http://localhost:9090 > /dev/null 2>&1; then
    echo "   ✓ ROS WebSocket bridge accessible on port 9090"
else
    echo "   ✗ ROS WebSocket bridge not accessible on port 9090"
fi

# Check Ollama service
if curl -s http://localhost:11434 > /dev/null 2>&1; then
    echo "   ✓ Ollama service accessible on port 11434"
else
    echo "   ✗ Ollama service not accessible on port 11434"
fi

# Test Gazebo readiness
echo ""
echo "5. Testing Gazebo simulation readiness..."
if pgrep -f gazebo > /dev/null; then
    echo "   ✓ Gazebo process is running"

    # Test if Gazebo topics are available
    if timeout 3 ros2 topic list | grep -q "/clock"; then
        echo "   ✓ Gazebo simulation topics detected"
    else
        echo "   ! Gazebo running but simulation topics not yet available"
    fi
else
    echo "   ✗ Gazebo is not running"
    echo "     Run: gazebo &"
fi

# Test bridge functionality
echo ""
echo "6. Testing ROS2-Gazebo bridge..."
if pgrep -f "nimbus_bridge" > /dev/null; then
    echo "   ✓ Nimbus bridge is running"
else
    echo "   ✗ Nimbus bridge is not running"
    echo "     Run: ros2 launch nimbus_bridge nimbus_bridge.launch.py &"
fi

echo ""
echo "=== Integration Test Complete ==="
echo ""

# Summary recommendations
echo "Quick start commands:"
echo "  Start Gazebo: gazebo &"
echo "  Start bridge: ros2 launch nimbus_bridge nimbus_bridge.launch.py &"
echo "  Test topics:  ros2 topic echo /clock"
echo ""