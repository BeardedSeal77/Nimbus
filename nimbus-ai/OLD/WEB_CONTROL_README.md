# 🌐 Nimbus AI Web Control System

A Flask-based web interface to control the Nimbus AI depth detection system.

## 🚀 Quick Start

### 1. Start the AI System
First, run the main AI system in one terminal:
```bash
python AI.py
```

### 2. Start the Web Control Server
In a **separate terminal**, run:
```bash
python start_web_control.py
```

### 3. Open the Web Interface
Open your browser and go to:
```
http://localhost:5000
```

## 🎯 How to Trigger Depth Detection

### Method 1: Web Interface (Recommended)
1. Open http://localhost:5000 in your browser
2. Click the **"🎯 Trigger Depth Detection"** button
3. The system will start collecting frames when objects are detected
4. Distance will be calculated after 5 frames are collected
5. Depth detection automatically stops after completion

### Method 2: API Endpoints
You can also control the system programmatically:

```bash
# Trigger depth detection
curl -X POST http://localhost:5000/api/trigger_depth

# Stop depth detection
curl -X POST http://localhost:5000/api/stop_depth

# Check status
curl http://localhost:5000/api/status
```

## 🔧 Web Interface Features

### 📏 Depth Detection Control
- **Trigger Depth Detection**: Sets `GLOBAL_GET_DIST = 1` to start depth collection
- **Stop Depth Detection**: Sets `GLOBAL_GET_DIST = 0` to stop depth collection
- **Real-time Status**: Shows current depth detection state and target distance

### 🔍 Object Detection Control
- **Set Target Object**: Change what object to detect (Person, Chair, Bottle, etc.)
- **Reset System**: Clear all parameters and stop processing

### ⚙️ Advanced Settings
- **Frame Collection Interval**: How often to collect frames (default: every 5th detection)
- **Frames per Batch**: How many frames to collect for SfM processing (default: 5)

### 📊 System Status
- Real-time monitoring of all system flags and parameters
- Processing status indicators
- Current target distance display

## 🔄 Depth Detection Workflow

1. **Trigger**: Click "Trigger Depth Detection" or call `/api/trigger_depth`
2. **Object Detection**: System detects target objects every 100ms
3. **Frame Collection**: Every 5th successful detection, a frame is collected
4. **Batch Processing**: After 5 frames are collected, SfM processing begins
5. **Distance Calculation**: System calculates distance using Structure from Motion
6. **Auto-Stop**: `GLOBAL_GET_DIST` automatically set to 0 after completion
7. **Result Display**: Distance shown in web interface and AI display

## 📱 API Reference

### GET /api/status
Returns current system status:
```json
{
  "global_object": "chair",
  "global_get_dist": 1,
  "global_target_distance": 2.45,
  "global_intent": "",
  "object_detection_busy": false,
  "depth_processing_busy": false,
  "depth_frame_interval": 5,
  "depth_frame_count": 5
}
```

### POST /api/trigger_depth
Starts depth detection:
```json
{
  "success": true,
  "message": "Depth detection triggered",
  "global_get_dist": 1
}
```

### POST /api/stop_depth
Stops depth detection:
```json
{
  "success": true,
  "message": "Depth detection stopped", 
  "global_get_dist": 0
}
```

### POST /api/set_target_object
Set target object:
```json
{
  "object": "Person"
}
```

### POST /api/set_depth_params
Update depth parameters:
```json
{
  "frame_interval": 5,
  "frame_count": 5
}
```

## 🛠️ Technical Details

### Architecture
- **Flask Server**: Runs independently from AI.py
- **Shared Memory**: Accesses AI.py global variables via module import
- **Thread Safety**: Uses threading locks for safe variable access
- **Real-time Updates**: Status refreshes every 2 seconds

### Global Variables Controlled
- `GLOBAL_GET_DIST`: Enable/disable depth detection (0 or 1)
- `GLOBAL_OBJECT`: Target object for detection
- `GLOBAL_TARGET_DISTANCE`: Calculated distance result
- `DEPTH_FRAME_INTERVAL`: Frame collection frequency
- `DEPTH_FRAME_COUNT`: Frames per SfM batch

### Safety Features
- Thread-safe global variable access
- Error handling for all API endpoints
- Automatic status refresh
- Visual indicators for system state

## 🚨 Important Notes

1. **Run Order**: Start AI.py FIRST, then start the web control server
2. **Separate Processes**: The web server runs independently from AI.py
3. **Auto-Disable**: Depth detection automatically stops after completion
4. **Thread Safety**: All global variable access is protected by locks
5. **Real-time**: Changes take effect immediately in the AI system

## 🔍 Troubleshooting

### Web server won't start
- Check if port 5000 is already in use
- Ensure Flask is installed: `pip install flask`

### Can't control AI system
- Make sure AI.py is running first
- Verify both scripts are in the same directory

### Status not updating
- Check browser console for JavaScript errors
- Refresh the page manually

## 🎮 Usage Examples

### Basic Depth Detection
1. Start AI.py
2. Start web control server
3. Open http://localhost:5000
4. Set target object to "Person"
5. Click "Trigger Depth Detection"
6. Move in front of camera
7. Wait for distance calculation (5 detections × 5 frames = ~2.5 seconds)
8. View result in web interface

### Advanced Configuration
1. Change "Frame Collection Interval" to 3 (faster collection)
2. Change "Frames per Batch" to 7 (more accurate SfM)
3. Click "Update Depth Parameters"
4. Trigger depth detection
5. Monitor progress in real-time