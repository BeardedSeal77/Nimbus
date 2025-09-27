#!/bin/bash

# Nimbus WSL2 Setup Script - Ubuntu 24.04 ONLY
# Installs ROS2 Iron + Gazebo for Ubuntu 24.04

set -e

echo "=== Nimbus WSL2 Setup - Ubuntu 24.04 Only ==="
echo ""

# Check Ubuntu version
UBUNTU_VERSION=$(lsb_release -rs)
if [[ "$UBUNTU_VERSION" != "24.04" ]]; then
    echo "ERROR: This script is only for Ubuntu 24.04. You have $UBUNTU_VERSION"
    exit 1
fi

echo "✓ Ubuntu 24.04 confirmed"
echo ""

# Auto-detect project directory
echo "Auto-detecting Nimbus project directory..."
PROJECT_DIR=""
for path in /mnt/c/Users/*/OneDrive/Documents/Work/Modules/Year*/PRJ/Nimbus; do
    if [ -d "$path" ] && [ -f "$path/docker-compose.dev.yml" ]; then
        PROJECT_DIR="$path"
        echo "   ✓ Found Nimbus project at: $PROJECT_DIR"
        break
    fi
done

if [ -z "$PROJECT_DIR" ]; then
    echo "   ✗ Could not find Nimbus project directory"
    read -p "Enter full path to your Nimbus project: " PROJECT_DIR
fi

echo "Using project directory: $PROJECT_DIR"
echo ""

# Update system
echo "Updating system..."
sudo apt update && sudo apt upgrade -y

# Install dependencies
echo "Installing dependencies..."
sudo apt install -y curl gnupg lsb-release software-properties-common wget ca-certificates

# Set locale
echo "Setting up locale..."
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Enable Universe repository
echo "Enabling Universe repository..."
sudo apt install -y software-properties-common
sudo add-apt-repository universe

# Add ROS2 Jazzy repository (Ubuntu 24.04 official)
echo "Adding ROS2 Jazzy repository..."
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Add Gazebo repository
echo "Adding Gazebo repository..."
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable noble main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Update package lists
echo "Updating package lists..."
sudo apt update

# Install minimal ROS2 for bridge only
echo "Installing minimal ROS2 (just for bridge)..."
sudo apt install -y \
    ros-jazzy-ros-core \
    ros-jazzy-geometry-msgs \
    ros-jazzy-sensor-msgs \
    ros-jazzy-nav-msgs \
    python3-colcon-common-extensions \
    python3-rosdep

# Install Gazebo Harmonic (Ubuntu 24.04)
echo "Installing Gazebo Harmonic..."
sudo apt install -y gz-harmonic

# Initialize rosdep
echo "Initializing rosdep..."
sudo rosdep init || true
rosdep update

# Setup ROS2 environment
echo "Setting up ROS2 environment..."
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

# Create workspace
echo "Creating ROS2 workspace..."
mkdir -p ~/nimbus_ws/src

# Setup Gazebo environment
echo "Setting up Gazebo environment..."
echo "export GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models:$GAZEBO_MODEL_PATH" >> ~/.bashrc
echo "export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11:$GAZEBO_RESOURCE_PATH" >> ~/.bashrc

# Create simple bridge package
echo "Creating ROS2-Gazebo bridge package..."
mkdir -p ~/nimbus_ws/src/nimbus_bridge

cat << 'EOF' > ~/nimbus_ws/src/nimbus_bridge/package.xml
<?xml version="1.0"?>
<package format="3">
  <name>nimbus_bridge</name>
  <version>1.0.0</version>
  <description>Nimbus ROS2-Gazebo bridge package</description>
  <maintainer email="nimbus@example.com">Nimbus Team</maintainer>
  <license>MIT</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
EOF

cat << 'EOF' > ~/nimbus_ws/src/nimbus_bridge/CMakeLists.txt
cmake_minimum_required(VERSION 3.8)
project(nimbus_bridge)
find_package(ament_cmake REQUIRED)
ament_package()
EOF

# Build workspace
echo "Building ROS2 workspace..."
cd ~/nimbus_ws
source /opt/ros/jazzy/setup.bash
colcon build

# Add workspace to bashrc
echo "source ~/nimbus_ws/install/setup.bash" >> ~/.bashrc

# Create startup script
echo "Creating startup script..."

cat << 'EOF' > ~/start-nimbus-gazebo.sh
#!/bin/bash
# Start Gazebo simulation for Nimbus project

echo "Starting Nimbus Gazebo Simulation..."
echo "Make sure Docker containers are running first!"
echo ""

# Set ROS domain
export ROS_DOMAIN_ID=0

# Source ROS2
source /opt/ros/jazzy/setup.bash
source ~/nimbus_ws/install/setup.bash

# Start Gazebo Harmonic
echo "Starting Gazebo Harmonic..."
gz sim &
GAZEBO_PID=$!

echo ""
echo "=== Nimbus Gazebo Started ==="
echo "Gazebo PID: $GAZEBO_PID"
echo ""
echo "To stop: pkill gazebo"
echo ""

# Wait for user input
read -p "Press Enter to stop Gazebo..."
kill $GAZEBO_PID 2>/dev/null || true
pkill -f "gz sim" 2>/dev/null || true
echo "Stopped."
EOF

chmod +x ~/start-nimbus-gazebo.sh

echo ""
echo "=== Installation Complete! ==="
echo ""
echo "Next steps:"
echo "1. Restart terminal (or run: source ~/.bashrc)"
echo "2. Start Docker: docker-compose -f docker-compose.dev.yml up -d"
echo "3. Start Gazebo: ~/start-nimbus-gazebo.sh"
echo ""