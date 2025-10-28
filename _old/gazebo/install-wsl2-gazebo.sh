#!/bin/bash

# Nimbus WSL2 Setup Script
# Installs ROS2 Humble + Gazebo 11 + ROS2-Gazebo bridge in WSL2

set -e

echo "=== Nimbus WSL2 ROS2 + Gazebo Installation ==="
echo "This script will install ROS2 Humble and Gazebo 11 in your WSL2 environment"
echo ""

# Auto-detect project directory
echo "Auto-detecting Nimbus project directory..."

# Common patterns for Windows drives in WSL2
POSSIBLE_PATHS=(
    "/mnt/c/Users/*/OneDrive/Documents/Work/Modules/Year*/PRJ/Nimbus"
    "/mnt/c/Users/*/Documents/Work/Modules/Year*/PRJ/Nimbus"
    "/mnt/c/Users/*/Desktop/Nimbus"
    "/mnt/c/Users/*/Downloads/Nimbus"
    "/mnt/d/*/Nimbus"
    "/mnt/e/*/Nimbus"
)

PROJECT_DIR=""
for pattern in "${POSSIBLE_PATHS[@]}"; do
    for path in $pattern; do
        if [ -d "$path" ] && [ -f "$path/docker-compose.dev.yml" ]; then
            PROJECT_DIR="$path"
            echo "   ✓ Found Nimbus project at: $PROJECT_DIR"
            break 2
        fi
    done
done

if [ -z "$PROJECT_DIR" ]; then
    echo "   ✗ Could not auto-detect Nimbus project directory"
    echo ""
    read -p "Enter full path to your Nimbus project directory: " PROJECT_DIR
    if [ ! -d "$PROJECT_DIR" ] || [ ! -f "$PROJECT_DIR/docker-compose.dev.yml" ]; then
        echo "ERROR: Invalid project directory. Must contain docker-compose.dev.yml"
        exit 1
    fi
fi

echo "Using project directory: $PROJECT_DIR"
echo ""

# Update system
echo "Updating system packages..."
sudo apt update && sudo apt upgrade -y

# Install required dependencies
echo "Installing dependencies..."
sudo apt install -y \
    curl \
    gnupg \
    lsb-release \
    software-properties-common \
    wget \
    ca-certificates

# Check Ubuntu version and add appropriate ROS2 repository
UBUNTU_VERSION=$(lsb_release -rs)
echo "Detected Ubuntu version: $UBUNTU_VERSION"

if [[ "$UBUNTU_VERSION" == "24.04" ]]; then
    echo "Ubuntu 24.04 detected - installing ROS2 Iron (compatible with Noble)"
    echo "Adding ROS2 Iron repository..."
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    ROS_DISTRO="iron"
else
    echo "Adding ROS2 Humble repository..."
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    ROS_DISTRO="humble"
fi

# Add Gazebo repository
echo "Adding Gazebo repository..."
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Update package lists
echo "Updating package lists..."
sudo apt update

# Install ROS2
echo "Installing ROS2 $ROS_DISTRO..."
sudo apt install -y \
    ros-$ROS_DISTRO-desktop \
    ros-$ROS_DISTRO-ros-gz \
    ros-$ROS_DISTRO-ros-gz-bridge \
    ros-$ROS_DISTRO-ros-gz-sim \
    ros-$ROS_DISTRO-gazebo-ros-pkgs \
    python3-colcon-common-extensions \
    python3-rosdep

# Install Gazebo 11
echo "Installing Gazebo 11..."
sudo apt install -y gazebo

# Initialize rosdep
echo "Initializing rosdep..."
sudo rosdep init || true
rosdep update

# Setup ROS2 environment
echo "Setting up ROS2 environment..."
echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc

# Create workspace
echo "Creating ROS2 workspace..."
mkdir -p ~/nimbus_ws/src

# Setup Gazebo environment
echo "Setting up Gazebo environment..."
echo "export GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models:$GAZEBO_MODEL_PATH" >> ~/.bashrc
echo "export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11:$GAZEBO_RESOURCE_PATH" >> ~/.bashrc

# Create Gazebo bridge launch file
echo "Creating ROS2-Gazebo bridge configuration..."
mkdir -p ~/nimbus_ws/src/nimbus_bridge/launch

cat << 'EOF' > ~/nimbus_ws/src/nimbus_bridge/package.xml
<?xml version="1.0"?>
<package format="3">
  <name>nimbus_bridge</name>
  <version>1.0.0</version>
  <description>Nimbus ROS2-Gazebo bridge package</description>
  <maintainer email="nimbus@example.com">Nimbus Team</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>ros_gz_bridge</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>nav_msgs</depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
EOF

cat << 'EOF' > ~/nimbus_ws/src/nimbus_bridge/CMakeLists.txt
cmake_minimum_required(VERSION 3.8)
project(nimbus_bridge)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
EOF

cat << 'EOF' > ~/nimbus_ws/src/nimbus_bridge/launch/nimbus_bridge.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Camera bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/camera/image@sensor_msgs/msg/Image@gz.msgs.Image'],
            output='screen'
        ),

        # Velocity command bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'],
            output='screen'
        ),

        # Odometry bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry'],
            output='screen'
        ),

        # Transform bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V'],
            output='screen'
        )
    ])
EOF

# Build workspace
echo "Building ROS2 workspace..."
cd ~/nimbus_ws
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build

# Add workspace to bashrc
echo "source ~/nimbus_ws/install/setup.bash" >> ~/.bashrc

# Create startup scripts
echo "Creating startup scripts..."

cat << EOF > ~/start-nimbus-gazebo.sh
#!/bin/bash
# Start Gazebo simulation for Nimbus project

echo "Starting Nimbus Gazebo Simulation..."
echo "Make sure Docker containers are running first!"
echo ""

# Set ROS domain
export ROS_DOMAIN_ID=0

# Source ROS2
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/nimbus_ws/install/setup.bash

# Start Gazebo in background
echo "Starting Gazebo..."
gazebo --verbose &
GAZEBO_PID=$!

# Wait for Gazebo to start
sleep 5

# Start ROS2 bridge
echo "Starting ROS2-Gazebo bridge..."
ros2 launch nimbus_bridge nimbus_bridge.launch.py &
BRIDGE_PID=$!

echo ""
echo "=== Nimbus Gazebo Started ==="
echo "Gazebo PID: $GAZEBO_PID"
echo "Bridge PID: $BRIDGE_PID"
echo ""
echo "To stop: pkill gazebo && pkill -f ros2"
echo ""

# Wait for user input to keep script running
read -p "Press Enter to stop Gazebo and bridge..."

echo "Stopping processes..."
kill $GAZEBO_PID $BRIDGE_PID 2>/dev/null || true
pkill gazebo 2>/dev/null || true
pkill -f ros2 2>/dev/null || true

echo "Stopped."
EOF

chmod +x ~/start-nimbus-gazebo.sh

echo ""
echo "=== Installation Complete! ==="
echo ""
echo "Project detected at: $PROJECT_DIR"
echo ""
echo "Next steps:"
echo "1. Restart your terminal (or run: source ~/.bashrc)"
echo "2. Start Docker containers from Windows in project directory"
echo "3. Start Gazebo: ~/start-nimbus-gazebo.sh"
echo ""
echo "Test with: ros2 topic list (should see topics from both Docker and Gazebo)"
echo ""