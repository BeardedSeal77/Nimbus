# Nimbus Project - Complete Installation Guide

## Overview
This guide covers the complete setup for the Nimbus drone AI system, including ROS2 containers, video pipeline, and AI processing components.

## Prerequisites
- Docker Desktop with GPU support
- Python 3.8+
- Git
- Network connection for downloading models and dependencies

## 1. Repository Setup

```bash
# Clone the repository
git clone <repository-url>
cd Nimbus

# Verify directory structure
ls -la
# Should show: docker-compose.yml, docker-compose.dev.yml, nimbus-ai/, etc.
```

## 2. Docker Environment Setup

### Option A: Production Environment (All Containers)
```bash
# Create external ollama volume for model persistence
docker volume create ollama

# Start all services
docker-compose up -d

# Verify all containers are running
docker-compose ps
```

### Option B: Development Environment (Recommended)
```bash
# Create external ollama volume
docker volume create ollama

# Start only external dependencies (ROS2 + Ollama)
docker-compose -f docker-compose.dev.yml up -d

# Verify containers
docker-compose -f docker-compose.dev.yml ps
```

## 3. ROS2 Container Configuration

### Verify ROS2 Setup
```bash
# Check ROS2 container status
docker exec ros2-central bash -c "source /opt/ros/humble/setup.bash && ros2 topic list"

# Should show topics like:
# /client_count
# /connected_clients  
# /parameter_events
# /rosout
```

### Verify ROSBridge WebSocket
```bash
# Test rosbridge connection
curl http://localhost:9090

# Should return: Can "Upgrade" only to "WebSocket".
```

### Manual ROS2 Commands (if needed)
```bash
# Enter ROS2 container
docker exec -it ros2-central bash

# Source ROS2 environment
source /opt/ros/humble/setup.bash

# List available packages
ros2 pkg list | grep rtabmap

# Check RTAB-Map installation
ros2 run rtabmap_ros rtabmap --help
```

## 4. Ollama Setup

### Verify Ollama Container
```bash
# Check ollama status
docker exec ollama ollama list

# Should show installed models (llama3:latest should be available)
```

### Install Models (if needed)
```bash
# Install llama3 model
docker exec ollama ollama pull llama3:latest

# Verify installation
docker exec ollama ollama list
```

## 5. Python Environment Setup

### Install Required Dependencies
```bash
# Core video pipeline dependencies
pip install websocket-client opencv-python numpy

# Additional helpers
pip install requests flask

# Optional: Full AI processing dependencies
pip install transformers torch ultralytics faster-whisper
```

### Verify Python Setup
```bash
cd nimbus-ai

# Test camera diagnostic
python camera_diagnostic.py

# Test AI imports (if dependencies installed)
python -c "from helpers.phone_camera import PhoneCamera; print('Camera helper OK')"
python -c "from helpers.display import ProcessedDisplay; print('Display helper OK')"
```

## 6. Camera Setup

### Camo Studio Configuration
1. **Install Camo Studio** on your phone
2. **Connect phone to same WiFi** as your computer  
3. **Start Camo Studio app** to establish connection
4. **Note the camera setup** but **close the app** before running pipeline

### Camera Testing
```bash
cd nimbus-ai

# Run camera diagnostic
python camera_diagnostic.py

# Should show:
# ✅ Camera 0: Opens successfully
# ✅ Camera 0: Can read frames (1280x720)
```

**Important**: If you see camera errors, make sure Camo Studio app is completely closed.

## 7. Video Pipeline Testing

### Start Complete System
```bash
cd nimbus-ai

# Method 1: Coordinated startup (recommended)
python run_video_system.py

# Method 2: Manual startup (for debugging)
# Terminal 1: python scripts/simple_camera_publisher.py  
# Terminal 2: python simple_ai_subscriber.py
```

### Expected Output
```
🚀 Starting Nimbus Video System...
Pipeline: Camo Studio → ROS2 → AI.py → Display

✅ Camera Publisher: Camo Studio → ROS2
✅ AI Subscriber: ROS2 → AI.py → Display  
✅ ROS2 Bridge: WebSocket communication working
🎥 You should see video feed in a window
```

## 8. Verification Checklist

### ✅ Docker Containers
- [ ] `ros2-central` container running
- [ ] `nimbus-ollama` container running  
- [ ] Port 9090 accessible (ROSBridge)
- [ ] Port 11434 accessible (Ollama)

### ✅ ROS2 System
- [ ] ROSBridge WebSocket responding
- [ ] RTAB-Map packages available
- [ ] ROS2 topics listing works

### ✅ Camera System  
- [ ] Camera 0 detected and accessible
- [ ] Camo Studio app closed
- [ ] Camera diagnostic passes

### ✅ Video Pipeline
- [ ] Camera publisher connects to ROS2
- [ ] AI subscriber receives frames
- [ ] Video display window appears
- [ ] Processing overlays visible

## 9. Troubleshooting

### Docker Issues
```bash
# Restart containers
docker-compose down
docker-compose up -d

# Check container logs
docker logs ros2-central
docker logs nimbus-ollama

# Remove and recreate (nuclear option)
docker-compose down -v
docker volume rm ollama
docker volume create ollama
docker-compose up -d
```

### ROS2 Issues
```bash
# Restart ROS2 container specifically
docker-compose restart ros2-central

# Check if rosbridge is running
docker exec ros2-central bash -c "ps aux | grep rosbridge"

# Manually start rosbridge (if needed)
docker exec -d ros2-central bash -c "source /opt/ros/humble/setup.bash && ros2 run rosbridge_server rosbridge_websocket --ros-args -p port:=9090"
```

### Camera Issues
- **Error -1072875772**: Close Camo Studio app completely
- **Camera not found**: Check USB connection or try different camera index
- **Permission denied**: Try running as administrator (Windows)

### Video Pipeline Issues
- **No video window**: Check if both publisher and subscriber are running
- **WebSocket errors**: Verify ROS2 container and rosbridge are running
- **Black screen**: Camera might be in use by another application

## 10. Development Workflow

### Daily Development Setup
```bash
# 1. Start containers
docker-compose -f docker-compose.dev.yml up -d

# 2. Verify systems
curl http://localhost:9090  # ROSBridge
curl http://localhost:11434/api/tags  # Ollama

# 3. Test camera
cd nimbus-ai
python camera_diagnostic.py

# 4. Run video system
python run_video_system.py
```

### Adding New AI Dependencies
```bash
# Install new packages
pip install <new-package>

# Update requirements.txt
pip freeze > requirements.txt

# Test integration
python -c "import <new-package>; print('OK')"
```

## 11. Production Deployment

### Full System Startup
```bash
# Start all containers
docker-compose up -d

# Verify all services
docker-compose ps

# Check service health
curl http://localhost:5000  # Web UI
curl http://localhost:9090  # ROSBridge  
curl http://localhost:11434/api/tags  # Ollama
```

### Monitoring
```bash
# Check container resource usage
docker stats

# View logs
docker-compose logs -f ros2-central
docker-compose logs -f nimbus-ollama
```

## Next Steps

Once the installation is complete:

1. **Test the video pipeline** with your camera
2. **Install AI dependencies** for full processing
3. **Implement individual AI nodes** as needed
4. **Connect to actual drone hardware** when ready

The system is now ready for development and testing! 🚀