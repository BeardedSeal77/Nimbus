# Nimbus Multi-Container Setup Guide

## Overview
The Nimbus project consists of 6 containerized services:
- **nimbus-ai**: AI processing (internal, ROS2 communication only)
- **nimbus-mr**: Mixed Reality services
- **nimbus-drone**: Drone/Robotics control
- **ros2-central**: ROS2 + RTAB-Map SLAM system
- **nimbus-webui**: Web interface
- **ollama**: Large Language Model service

## File Structure
```
Nimbus/
├── nimbus-ai/          # AI Processing (internal only)
├── nimbus-mr/          # Mixed Reality
├── nimbus-robotics/    # Drone/Robotics
├── nimbus-webui/       # Web Interface
├── ros2-config/        # ROS2 configuration
├── docker-compose.yml  # Production deployment
└── docker-compose.dev.yml # Development (Ollama + ROS2 only)
```

## Prerequisites

### 1. Docker & Docker Compose
```bash
# Install Docker
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh

# Install Docker Compose
sudo curl -L "https://github.com/docker/compose/releases/latest/download/docker-compose-$(uname -s)-$(uname -m)" -o /usr/local/bin/docker-compose
sudo chmod +x /usr/local/bin/docker-compose
```

### 2. NVIDIA Docker Support (for Ollama GPU acceleration)
```bash
# Install NVIDIA Container Toolkit
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list

sudo apt-get update && sudo apt-get install -y nvidia-docker2
sudo systemctl restart docker
```

### 3. System Requirements
- **RAM**: Minimum 16GB (32GB recommended for full stack)
- **Storage**: 30GB free space
- **GPU**: NVIDIA GPU recommended for Ollama
- **OS**: Linux (Ubuntu 20.04+) or Windows with WSL2

## Installation Methods

### Option 1: Full Production Setup
Deploy all 6 containers for complete system:

```bash
# Clone repository
git clone <repository-url>
cd Nimbus

# Ensure Ollama volume exists (if you have existing models)
docker volume create ollama

# Build and start all services
docker-compose build
docker-compose up -d

# Check all containers are running
docker-compose ps
```

### Option 2: Development Setup
Only run external dependencies (Ollama + ROS2):

```bash
# Start only external dependencies
docker-compose -f docker-compose.dev.yml up -d

# Verify containers
docker-compose -f docker-compose.dev.yml ps

# Work locally in nimbus-ai/ directory
cd nimbus-ai
python -c "from AI import get_ai_system; print('AI system ready')"
```

## Service Configuration

### Ollama Setup
If starting fresh (no existing models):
```bash
# Enter Ollama container
docker exec -it nimbus-ollama bash

# Download llama3 model
ollama pull llama3:latest

# Verify model
ollama list
```

**Note**: If you already have Ollama models, they'll be automatically available via the external `ollama` volume.

### ROS2 + RTAB-Map Setup
The ROS2 container includes RTAB-Map and is ready to use:
```bash
# Allow X11 forwarding (Linux)
xhost +local:docker

# Test ROS2 installation
docker exec -it ros2-central bash -c "source /opt/ros/humble/setup.bash && ros2 pkg list | grep rtabmap"

# Launch RTAB-Map (when ready)
docker exec -it ros2-central bash -c "source /opt/ros/humble/setup.bash && ros2 launch rtabmap_ros rtabmap.launch.py"
```

## Network Architecture
All services communicate via the `nimbus-network` bridge:

### Production URLs (internal):
- **nimbus-ai**: Internal only (no external ports)
- **nimbus-mr**: `http://nimbus-mr:5003`
- **nimbus-drone**: `http://nimbus-drone:5004`
- **ros2-central**: `http://ros2-central:11311`
- **ollama**: `http://ollama:11434`

### External Access:
- **Web UI**: `http://localhost:5000`
- **Mixed Reality**: `http://localhost:5003`
- **Drone Service**: `http://localhost:5004`
- **ROS2 Bridge**: `http://localhost:9090`
- **ROS2 Master**: `http://localhost:11311`
- **Ollama API**: `http://localhost:11434`

## Verification

### Test All Services
```bash
# Web UI
curl http://localhost:5000

# Mixed Reality
curl http://localhost:5003/mr/status

# Drone Service
curl http://localhost:5004/drone/status

# Ollama
curl http://localhost:11434/api/tags

# ROS2 (from within container)
docker exec -it ros2-central bash -c "source /opt/ros/humble/setup.bash && ros2 node list"
```

### Test AI Components (Development)
```bash
cd nimbus-ai
python classes/Tests/test_intent_object_node.py
python classes/Tests/interactive_intent_test.py
```

## Development Workflow

### Local Development (Recommended)
```bash
# 1. Start external dependencies only
docker-compose -f docker-compose.dev.yml up -d

# 2. Install local dependencies
cd nimbus-ai
pip install -r requirements.txt

# 3. Work on your code locally
python -c "from classes.intent_object_node import intent_object_node; print(intent_object_node('go to the chair'))"

# 4. Test against containerized services
python test-dev-setup.py
```

### Container Development
```bash
# Rebuild specific service
docker-compose build nimbus-ai

# Restart specific service
docker-compose restart nimbus-ai

# View service logs
docker-compose logs -f nimbus-ai
```

## Troubleshooting

### Common Issues

**1. ROS2 container exits immediately**
- Fixed: Container now includes keep-alive command
- Check logs: `docker logs ros2-central`

**2. Ollama models not found**
- Ensure external `ollama` volume exists: `docker volume ls | grep ollama`
- Your existing models should be automatically available

**3. GPU not detected in Ollama**
```bash
# Test GPU access
docker run --rm --gpus all nvidia/cuda:11.0-base nvidia-smi
```

**4. Port conflicts**
```bash
# Check port usage
netstat -tulpn | grep :11434
# Stop conflicting services if needed
```

**5. Permission issues (Linux)**
```bash
# Add user to docker group
sudo usermod -aG docker $USER
newgrp docker
```

### Service Health Checks
```bash
# Development setup
curl http://localhost:11434/api/tags  # Ollama
docker exec -it ros2-central bash -c "source /opt/ros/humble/setup.bash && ros2 --help"  # ROS2

# Production setup
docker-compose ps  # All containers
curl http://localhost:5000  # Web UI
```

### Clean Restart
```bash
# Development
docker-compose -f docker-compose.dev.yml down
docker-compose -f docker-compose.dev.yml up -d

# Production
docker-compose down
docker-compose build --no-cache
docker-compose up -d
```

## Architecture Notes

### AI Container (nimbus-ai)
- **Internal only**: No external HTTP API
- **Communication**: Direct integration with ROS2
- **Purpose**: Core AI processing (STT, intent extraction, object detection, etc.)
- **Development**: Work locally in `nimbus-ai/` directory

### Volume Management
- **ollama**: External volume for model persistence
- **ROS2 configs**: Mounted from `./ros2-config/`

### Security
- AI container has no external exposure
- All communication flows through ROS2 or other service APIs
- GPU access restricted to Ollama container

## Next Steps

### For Development:
1. Start with `docker-compose -f docker-compose.dev.yml up -d`
2. Work locally in `nimbus-ai/`
3. Use containerized Ollama + ROS2 as dependencies

### For Production:
1. Use `docker-compose up -d` for full deployment
2. Configure your specific hardware interfaces
3. Customize ROS2 launch parameters

The system is modular and ready for both development and production use!