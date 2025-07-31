# Nimbus Development System Installation Guide

Complete guide to setting up the Nimbus development environment from scratch on Windows.

## Prerequisites

- Windows 10/11 (64-bit)
- Administrator access to your machine
- Stable internet connection

## Step 1: Install WSL (Windows Subsystem for Linux)

### 1.1 Enable WSL
Open PowerShell as Administrator and run:
```powershell
wsl --install
```

This will:
- Enable WSL and Virtual Machine Platform features
- Install Ubuntu (default distribution)
- Download and install the Linux kernel update

### 1.2 Restart Your Computer
After installation completes, restart your machine.

### 1.3 Set Up Ubuntu
1. Launch Ubuntu from Start Menu
2. Create a new user account when prompted:
   ```bash
   # Enter new UNIX username: your_username
   # New password: [enter secure password]
   # Retype new password: [confirm password]
   ```

### 1.4 Update Ubuntu
```bash
sudo apt update && sudo apt upgrade -y
```

### 1.5 Install Essential Tools
```bash
# Install curl, git, and other essentials
sudo apt install -y curl git build-essential

# Install Python and pip
sudo apt install -y python3 python3-pip
```

## Step 2: Install Docker Desktop

### 2.1 Download Docker Desktop
1. Go to [Docker Desktop for Windows](https://docs.docker.com/desktop/install/windows-install/)
2. Download Docker Desktop Installer
3. Run the installer with Administrator privileges

### 2.2 Configure Docker Desktop
1. **Launch Docker Desktop** from Start Menu
2. **Accept the service agreement**
3. **Complete the tutorial** (optional but recommended)
4. **Configure WSL Integration**:
   - Go to Settings → Resources → WSL Integration
   - Enable integration with your Ubuntu distribution
   - Click "Apply & Restart"

### 2.3 Verify Docker Installation
Open Ubuntu terminal and verify:
```bash
# Check Docker is accessible from WSL
docker --version
docker-compose --version

# Test Docker with hello-world
docker run hello-world
```

## Step 3: Set Up Nimbus Development Environment

### 3.1 Clone the Repository
```bash
# Navigate to your preferred directory
cd ~
mkdir -p workspace
cd workspace

# Clone the Nimbus repository
git clone [YOUR_NIMBUS_REPO_URL] Nimbus
cd Nimbus
```

### 3.2 Start Development Containers
```bash
# Start only Ollama + ROS2 containers
docker-compose -f docker-compose.dev.yml up -d

# Verify both containers are running
docker-compose -f docker-compose.dev.yml ps
```

Expected output:
```
     Name                   Command               State                    Ports                  
------------------------------------------------------------------------------------------------
nimbus-ollama    /bin/ollama serve                Up      0.0.0.0:11434->11434/tcp               
ros2-central     bash -c source /opt/ros/hu ...   Up      0.0.0.0:11311->11311/tcp, 0.0.0.0:9090->9090/tcp
```

### 3.3 Verify Ollama Models
Your existing models are automatically available:
```bash
# Check available models
docker exec -it nimbus-ollama ollama list

# Should show llama3:latest and other models
# If no models, download them:
docker exec -it nimbus-ollama ollama pull llama3:latest
```

### 3.4 Set Up Local Python Environment
```bash
# Navigate to the nimbus-ai directory
cd nimbus-ai

# Install Python dependencies
pip3 install -r requirements.txt

# Test the installation
python3 -c "from AI import get_ai_system; print('AI system imported successfully')"
```

## Step 4: Verify Complete Installation

### 4.1 Test Ollama Connection
```bash
# Test Ollama API from host
curl -X POST http://localhost:11434/api/generate \
  -H "Content-Type: application/json" \
  -d '{"model": "llama3:latest", "prompt": "Hello", "stream": false}'
```

### 4.2 Test Python Integration
```bash
cd nimbus-ai

# Test intent extraction
python3 classes/Tests/interactive_intent_test.py

# Test AI system health
python3 -c "from AI import get_ai_system; ai = get_ai_system(); print(ai.health_check())"
```

### 4.3 Test ROS2 Container
```bash
# Enter ROS2 container
docker exec -it ros2-central bash

# Inside container, test ROS2
source /opt/ros/humble/setup.bash
ros2 pkg list | grep rtabmap

# Exit container
exit
```

## Step 5: Development Workflow

### Daily Startup Routine
```bash
# 1. Open Ubuntu terminal
# 2. Navigate to project
cd ~/workspace/Nimbus

# 3. Start development containers
docker-compose -f docker-compose.dev.yml up -d

# 4. Work in nimbus-ai directory
cd nimbus-ai

# 5. Your development environment is ready!
```

### Stopping the Environment
```bash
# Stop containers when done
docker-compose -f docker-compose.dev.yml down
```

## Troubleshooting

### Common Issues

**WSL Installation Issues:**
- Ensure virtualization is enabled in BIOS
- Run `wsl --status` to check WSL version
- Update WSL: `wsl --update`

**Docker Desktop Issues:**
- Restart Docker Desktop service
- Check WSL integration is enabled
- Verify Docker daemon is running: `docker info`

**Container Issues:**
```bash
# Check container logs
docker logs nimbus-ollama
docker logs ros2-central

# Restart specific container
docker-compose -f docker-compose.dev.yml restart ollama
```

**Port Conflicts:**
```bash
# Check what's using ports
netstat -tulpn | grep :11434  # Ollama
netstat -tulpn | grep :11311  # ROS2
```

**Python Import Issues:**
```bash
# Ensure you're in the correct directory
cd nimbus-ai
python3 -c "import sys; print(sys.path)"
```

### Reset Environment
```bash
# Complete reset if needed
docker-compose -f docker-compose.dev.yml down
docker volume rm ollama
docker volume create ollama
docker-compose -f docker-compose.dev.yml up -d
```

## Next Steps

1. **Read the Development Guide**: Check `_DOCS/DEVELOPMENT.md` for detailed development workflows
2. **Start Coding**: Begin working on AI nodes in `nimbus-ai/classes/`
3. **Run Tests**: Use files in `nimbus-ai/classes/Tests/` to verify your changes
4. **Explore Integration**: Test communication between local Python and containerized services

## Useful Commands Reference

```bash
# Container Management
docker-compose -f docker-compose.dev.yml up -d     # Start containers
docker-compose -f docker-compose.dev.yml down      # Stop containers
docker-compose -f docker-compose.dev.yml ps        # Check status
docker-compose -f docker-compose.dev.yml logs      # View logs

# Development
cd nimbus-ai                                        # Work directory
python3 classes/Tests/interactive_intent_test.py   # Test intent extraction
pip3 install -r requirements.txt                   # Update dependencies

# Docker Utilities
docker exec -it nimbus-ollama bash                 # Enter Ollama container
docker exec -it ros2-central bash                  # Enter ROS2 container
docker volume ls                                    # List volumes
docker network ls                                   # List networks
```

Your Nimbus development environment is now ready! 🚀