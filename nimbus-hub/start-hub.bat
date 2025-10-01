@echo off
setlocal EnableDelayedExpansion

REM Disable Ctrl+C prompt
if "%1"=="CHILD" goto :CHILD

REM Start as child process to disable Ctrl+C prompt
cmd /c ""%~f0" CHILD %*"
exit /b

:CHILD
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
echo Web Interface will open at: http://localhost:5000
echo.

REM Open web browser after 5 seconds (gives server time to start)
start "" cmd /c "timeout /t 5 /nobreak > nul && start http://localhost:5000"

REM Start the hub
python hub.py