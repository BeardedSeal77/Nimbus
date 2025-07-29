@echo off
echo ========================================
echo NIMBUS VIDEO PIPELINE STARTUP
echo ========================================
echo Starting optimized multi-threaded video pipeline...
echo.

cd /d "%~dp0"
python start_video_pipeline.py

pause