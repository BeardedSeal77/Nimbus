#!/bin/bash

# Nimbus ROS2 Package Installation Script
# Run this script inside the ros2-central container to install all required packages

set -e  # Exit on any error

echo "🚀 Starting Nimbus ROS2 package installation..."

# Update package lists
echo "📦 Updating package lists..."
apt update

# Core ROS2 packages
echo "🔧 Installing core ROS2 packages..."
apt install -y \
    ros-humble-rclcpp \
    ros-humble-rclpy \
    ros-humble-rclcpp-components \
    ros-humble-rclcpp-lifecycle

# Message types
echo "📨 Installing message packages..."
apt install -y \
    ros-humble-std-msgs \
    ros-humble-geometry-msgs \
    ros-humble-sensor-msgs \
    ros-humble-nav-msgs \
    ros-humble-visualization-msgs \
    ros-humble-trajectory-msgs \
    ros-humble-diagnostic-msgs

# SLAM and Navigation
echo "🗺️ Installing SLAM and Navigation packages..."
apt install -y \
    ros-humble-rtabmap-ros \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-nav2-msgs \
    ros-humble-slam-toolbox

# Transform and coordinate frames
echo "🔄 Installing transform packages..."
apt install -y \
    ros-humble-tf2 \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-tf2-sensor-msgs \
    ros-humble-robot-state-publisher

# Image and point cloud processing
echo "🖼️ Installing image processing packages..."
apt install -y \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-image-geometry \
    ros-humble-pcl-ros \
    ros-humble-pcl-conversions \
    ros-humble-pcl-msgs \
    ros-humble-depthimage-to-laserscan

# Communication and bridge
echo "🌐 Installing communication packages..."
apt install -y \
    ros-humble-rosbridge-server \
    ros-humble-rosbridge-suite \
    ros-humble-rosapi

# Sensors
echo "📡 Installing sensor packages..."
apt install -y \
    ros-humble-realsense2-camera \
    ros-humble-velodyne \
    ros-humble-imu-filter-madgwick

# Teleop and control
echo "🎮 Installing control packages..."
apt install -y \
    ros-humble-teleop-twist-joy \
    ros-humble-teleop-twist-keyboard \
    ros-humble-joy

# Visualization
echo "👁️ Installing visualization packages..."
apt install -y \
    ros-humble-rviz2 \
    ros-humble-rviz-common \
    ros-humble-interactive-markers

# Utilities
echo "🔧 Installing utility packages..."
apt install -y \
    ros-humble-resource-retriever \
    ros-humble-pluginlib \
    ros-humble-filters \
    ros-humble-angles \
    ros-humble-laser-geometry \
    ros-humble-kdl-parser

# Additional useful packages
echo "➕ Installing additional packages..."
apt install -y \
    ros-humble-example-interfaces \
    ros-humble-lifecycle \
    ros-humble-composition \
    ros-humble-demo-nodes-cpp \
    ros-humble-demo-nodes-py

# Clean up
echo "🧹 Cleaning up..."
apt autoremove -y
apt autoclean

echo "✅ All packages installed successfully!"
echo "🔄 You may need to restart the container for all changes to take effect."
echo ""
echo "To verify installation, run:"
echo "  source /opt/ros/humble/setup.bash"
echo "  ros2 pkg list | wc -l"