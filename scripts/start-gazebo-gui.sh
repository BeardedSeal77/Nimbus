#!/bin/bash

# Nimbus Gazebo GUI Startup Script
# This script automates the X11 forwarding and container startup

echo "=== Nimbus Gazebo GUI Startup ==="

# Check if running on WSL/Linux
if [[ "$OSTYPE" == "linux-gnu"* ]] || [[ "$OSTYPE" == "msys" ]] || [[ -n "$WSL_DISTRO_NAME" ]]; then
    echo "Setting up X11 forwarding..."

    # Enable X11 forwarding for Docker containers
    xhost +local:docker

    # Set DISPLAY variable if not set
    if [ -z "$DISPLAY" ]; then
        export DISPLAY=:0
        echo "Set DISPLAY to :0"
    else
        echo "Using existing DISPLAY: $DISPLAY"
    fi

    # Stop old VNC container if running
    echo "Stopping old VNC container..."
    docker-compose -f docker-compose.dev.yml stop gazebo-gui 2>/dev/null || true

    # Start the new ROS2 Humble + Gazebo container
    echo "Starting ROS2 Humble + Gazebo container..."
    docker-compose -f docker-compose.dev.yml up -d ros2-humble-gazebo

    # Wait for container to be ready
    echo "Waiting for container to be ready..."
    sleep 5

    echo ""
    echo "=== Container Started Successfully ==="
    echo ""
    echo "To access Gazebo GUI:"
    echo "1. Run: docker exec -it ros2_humble_gazebo_harmonic bash"
    echo "2. Inside container: ign gazebo"
    echo ""
    echo "Or run this one-liner:"
    echo "docker exec -it ros2_humble_gazebo_harmonic bash -c 'source /opt/ros/humble/setup.bash && ign gazebo'"
    echo ""

else
    echo "ERROR: This script requires Linux/WSL environment for X11 forwarding"
    echo "Please see GAZEBO_SETUP.md for Windows-specific instructions"
    exit 1
fi