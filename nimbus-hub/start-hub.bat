@echo off
REM Start Nimbus Central Communication Hub

echo ==========================================
echo Starting Nimbus Central Hub
echo ==========================================

REM Check if virtual environment exists
if not exist "venv" (
    echo Creating virtual environment...
    python -m venv venv
)

REM Activate virtual environment
call venv\Scripts\activate.bat

REM Install PyTorch with CUDA support first
echo Installing PyTorch with CUDA 12.9 support...
pip3 install torch torchvision --index-url https://download.pytorch.org/whl/cu129

REM Install other dependencies
echo Installing other dependencies...
pip install -r requirements.txt

REM Start the hub
echo.
echo Starting Nimbus Hub...
python hub.py

pause