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

# Install ROS2 Jazzy (Ubuntu 24.04 official + Gazebo Harmonic support)
echo "Installing ROS2 Jazzy (Ubuntu 24.04 official + Gazebo Harmonic support)..."
sudo apt install -y \
    ros-jazzy-ros-core \
    ros-jazzy-geometry-msgs \
    ros-jazzy-sensor-msgs \
    ros-jazzy-nav-msgs \
    ros-jazzy-std-msgs \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-rmw-cyclonedds-cpp \
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

cat << 'STARTUP_EOF' > ~/start-nimbus-gazebo.sh
#!/bin/bash
# Start Gazebo simulation for Nimbus project

echo "Starting Nimbus Gazebo Simulation..."
echo "Make sure Docker containers are running first!"
echo ""

# Set ROS2 environment for FastRTPS connection to Docker
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/fastrtps_profile.xml

# Create FastRTPS profile to connect directly to Docker
cat > /tmp/fastrtps_profile.xml << 'PROFILE_EOF'
<?xml version="1.0" encoding="UTF-8"?>
<profiles>
  <participant profile_name="default_participant" is_default_profile="true">
    <rtps>
      <builtin>
        <discovery_config>
          <static_edp_xml_config>
            <participant>
              <name>DockerROS2</name>
              <unicast_locator address="192.168.8.102" port="7400"/>
            </participant>
          </static_edp_xml_config>
        </discovery_config>
      </builtin>
    </rtps>
  </participant>
</profiles>
PROFILE_EOF

# Source ROS2
source /opt/ros/jazzy/setup.bash
source ~/nimbus_ws/install/setup.bash

# Copy world files
echo "Setting up world files..."
mkdir -p ~/.gazebo/models
mkdir -p ~/.simulation-gazebo/worlds

PROJECT_PATH="/mnt/c/Users/edcul/OneDrive/Documents/Work/Modules/Year 3/PRJ/Nimbus"
if [ -f "$PROJECT_PATH/gazebo/gazebo-worlds/world.sdf" ]; then
    cp "$PROJECT_PATH/gazebo/gazebo-worlds/world.sdf" ~/.simulation-gazebo/worlds/
    cp "$PROJECT_PATH/gazebo/gazebo-worlds/world.dae" ~/.simulation-gazebo/worlds/
    echo "World files copied successfully"
else
    echo "World files not found, will start with empty world"
fi

# Start ROS2-Gazebo bridge
echo "Starting ROS2-Gazebo bridge..."
ros2 run ros_gz_bridge parameter_bridge \
    /tello/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist \
    /tello/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry \
    /tello/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image \
    /tello/status@std_msgs/msg/String@gz.msgs.StringMsg &
BRIDGE_PID=$!

# Start status publisher
python3 -c "
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class StatusPublisher(Node):
    def __init__(self):
        super().__init__('tello_status_publisher')
        self.publisher = self.create_publisher(String, '/tello/status', 10)
        self.timer = self.create_timer(2.0, self.publish_status)

    def publish_status(self):
        msg = String()
        msg.data = 'waiting'
        self.publisher.publish(msg)

def main():
    rclpy.init()
    publisher = StatusPublisher()
    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
" &
STATUS_PID=$!

sleep 2

# Start Gazebo
echo "Starting Gazebo Harmonic..."
if [ -f ~/.simulation-gazebo/worlds/world.sdf ]; then
    gz sim ~/.simulation-gazebo/worlds/world.sdf &
else
    gz sim &
fi
GAZEBO_PID=$!

echo ""
echo "=== Nimbus Gazebo Started ==="
echo "Gazebo PID: $GAZEBO_PID"
echo ""

# Wait for user input
read -p "Press Enter to stop Gazebo..."
kill $GAZEBO_PID 2>/dev/null || true
kill $BRIDGE_PID 2>/dev/null || true
kill $STATUS_PID 2>/dev/null || true
pkill -f "gz sim" 2>/dev/null || true
pkill -f "parameter_bridge" 2>/dev/null || true
pkill -f "tello_status_publisher" 2>/dev/null || true
echo "Stopped."
STARTUP_EOF

chmod +x ~/start-nimbus-gazebo.sh

echo ""
echo "=== Installation Complete! ==="
echo ""
echo "Next steps:"
echo "1. Restart terminal (or run: source ~/.bashrc)"
echo "2. Start Docker: docker-compose -f docker-compose.dev.yml up -d"
echo "3. Start Gazebo: ~/start-nimbus-gazebo.sh"
echo ""