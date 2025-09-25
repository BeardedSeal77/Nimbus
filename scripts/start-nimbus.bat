@echo off
REM Nimbus Unified Startup Script
REM Starts Docker backend + WSL2 Gazebo simulation

echo === Nimbus Unified Startup ===
echo.

REM Check if Docker is running
echo Checking Docker status...
docker version >nul 2>&1
if %errorlevel% neq 0 (
    echo ERROR: Docker is not running or not installed
    echo Please start Docker Desktop and try again
    pause
    exit /b 1
)

REM Check if WSL2 is available
echo Checking WSL2 status...
wsl --status >nul 2>&1
if %errorlevel% neq 0 (
    echo ERROR: WSL2 is not available
    echo Please install WSL2 and Ubuntu distribution
    pause
    exit /b 1
)

echo.
echo === Starting Docker Backend ===
echo Starting ROS2 central container and Ollama...
docker-compose -f docker-compose.dev.yml up -d

if %errorlevel% neq 0 (
    echo ERROR: Failed to start Docker containers
    pause
    exit /b 1
)

echo.
echo === Docker containers started ===
echo ROS2 WebSocket Bridge: http://localhost:9090
echo Ollama API: http://localhost:11434
echo.

REM Wait for containers to be ready
echo Waiting for containers to initialize...
timeout /t 5 /nobreak >nul

echo.
echo === Starting WSL2 Gazebo ===
echo Launching Gazebo simulation in WSL2...
echo.

REM Start Gazebo in WSL2
wsl bash -c "cd /mnt/c/Users/edcul/OneDrive/Documents/Work/Modules/Year\ 3/PRJ/Nimbus && ~/start-nimbus-gazebo.sh"

echo.
echo === Nimbus System Started ===
echo.
echo Services running:
echo - Docker ROS2 central: ros2-central container
echo - Ollama LLM: nimbus-ollama container
echo - Gazebo simulation: WSL2 native
echo - ROS2 bridge: WSL2 connecting to Docker
echo.
echo To stop everything:
echo 1. Close this window to stop Gazebo
echo 2. Run: docker-compose -f docker-compose.dev.yml down
echo.
echo Press Ctrl+C to stop Gazebo and bridges...
pause