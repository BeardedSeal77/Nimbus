# Nimbus Setup Guide - WSL2 + Docker Architecture

## Overview
This setup splits Nimbus into two optimized environments:
- **Docker**: Backend services (ROS2, Ollama, nimbus-ai web interface)
- **WSL2**: Native Gazebo simulation with GUI forwarding to Windows

## Prerequisites
- Windows 10/11 with WSL2 enabled
- **Ubuntu 24.04** installed in WSL2 (REQUIRED)
- Docker Desktop installed and running
- NVIDIA drivers for WSL2 (if using GPU acceleration)

## Step-by-Step Setup

### Step 1: Copy Installation Script Using File Explorer
1. **Open File Explorer** in Windows
2. **Navigate to**: `Ubuntu\home\%USERNAME%`
3. **Open another File Explorer window** and navigate to your Nimbus project folder
4. **Copy** `scripts\install-wsl2-gazebo-ubuntu24.sh` from your Nimbus project
5. **Paste** it into the Ubuntu home folder

### Step 2: Run Installation from Ubuntu Terminal
Open Ubuntu terminal and run:

```bash
sudo apt update && sudo apt install dos2unix -y
dos2unix ~/install-wsl2-gazebo-ubuntu24.sh
chmod +x ~/install-wsl2-gazebo-ubuntu24.sh
./install-wsl2-gazebo-ubuntu24.sh
```

**What this installs:**
- ROS2 Iron (Ubuntu 24.04 compatible)
- Gazebo (classic)
- Basic ROS2-Gazebo bridge
- Nimbus workspace and configuration
- Auto-detects your project directory

**Installation time:** ~10-15 minutes

### 2. Start Everything
From Windows (in project directory):
```batch
scripts\start-nimbus.bat
```

Or manually:
1. **Start Docker backend**: `docker-compose -f docker-compose.dev.yml up -d`
2. **Start WSL2 Gazebo**: In WSL2 run `~/start-nimbus-gazebo.sh`

## Architecture Details

### Docker Services
- **ros2-central**: ROS2 master + WebSocket bridge (ports 11311, 9090)
- **ollama**: LLM service (port 11434)

### WSL2 Components
- **Gazebo**: Native simulation with Windows GUI
- **ROS2 bridge**: Connects Gazebo to Docker ROS2
- **Topics**: /camera/image, /cmd_vel, /odom, /tf

### Communication Flow
```
nimbus-ai (Python) → WebSocket (9090) → Docker ROS2 → WSL2 Bridge → Gazebo
```

## Usage

### Starting the System
1. Run `scripts\start-nimbus.bat` from Windows
2. Gazebo window appears on Windows desktop
3. Access nimbus-ai web interface as usual
4. ROS2 topics flow between all components

### Checking Status
```bash
# In WSL2 - should see both Docker and WSL2 topics
ros2 topic list

# Should show topics from both environments:
# /camera/image (from Gazebo)
# /cmd_vel (bidirectional)
# /rosout (from Docker)
```

### Stopping
- **Close Gazebo window** or Ctrl+C in WSL2 terminal
- **Stop Docker**: `docker-compose -f docker-compose.dev.yml down`

## Troubleshooting

### Gazebo doesn't appear
```bash
# Check X11 forwarding in WSL2
echo $DISPLAY  # Should show something like :0

# Test GUI forwarding
xclock  # Should show a clock window
```

### ROS2 communication issues
```bash
# Check ROS domain matches
echo $ROS_DOMAIN_ID  # Should be 0 in both WSL2 and Docker

# Check Docker network connectivity from WSL2
ping host.docker.internal
```

### Bridge not connecting
```bash
# In WSL2, check if Docker ROS2 is visible
ros2 node list  # Should see Docker nodes

# Restart bridge if needed
pkill -f ros2
ros2 launch nimbus_bridge nimbus_bridge.launch.py
```

## File Structure
```
scripts/
├── install-wsl2-gazebo-ubuntu24.sh  # WSL2 setup script (Ubuntu 24.04 only)
├── start-nimbus.bat                 # Windows unified startup
└── install-wsl2-gazebo.sh           # Old script (archived)

WSL2 (~/ directory):
├── nimbus_ws/                       # ROS2 Iron workspace
├── start-nimbus-gazebo.sh           # WSL2 startup script
└── .bashrc                          # Auto-configured with ROS2 Iron
```

## Benefits
- **Native Gazebo performance** with GPU acceleration
- **Real Windows GUI** - no VNC/browser complexity
- **Your program unchanged** - still uses same ROS2 WebSocket
- **Clean separation** - simulation vs backend services
- **Easy maintenance** - each component optimized for its purpose