#!/bin/bash
# Start Nimbus Central Communication Hub

echo "=========================================="
echo "Starting Nimbus Central Hub"
echo "=========================================="

# Check if virtual environment exists
if [ ! -d "venv" ]; then
    echo "Creating virtual environment..."
    python3 -m venv venv
fi

# Activate virtual environment
source venv/bin/activate

# Install PyTorch with CUDA support first
echo "Installing PyTorch with CUDA 12.9 support..."
pip3 install torch torchvision --index-url https://download.pytorch.org/whl/cu129

# Install other dependencies
echo "Installing other dependencies..."
pip install -r requirements.txt

# Start the hub
echo ""
echo "Starting Nimbus Hub..."
python hub.py