# Nimbus Unified Architecture

## Overview

The Nimbus system has been consolidated into a single-entry-point architecture where **nimbus-hub** is the main Flask server that spawns **nimbus-ai** as a parallel worker process.

## Architecture Diagram

```
┌─────────────────────────────────────────────────────────────┐
│                      NIMBUS HUB                             │
│                 (Flask Server - Port 5000)                   │
│                                                             │
│  ┌─────────────────┐  ┌──────────────────┐                │
│  │ Web Interface   │  │ Message Broker   │                │
│  │ - control.html  │  │ - pub/sub topics │                │
│  │ - telemetry     │  │ - SSE streaming  │                │
│  │ - video feed    │  │ - drone state    │                │
│  └─────────────────┘  └──────────────────┘                │
│                                                             │
│  ┌───────────────────────────────────────────────────┐    │
│  │        Shared State (multiprocessing.Manager)      │    │
│  │  - cancel, mode, target_object                     │    │
│  │  - ai_results, video_frame                         │    │
│  │  - last_heartbeat, processing_fps                  │    │
│  └───────────────────────────────────────────────────┘    │
│                           ↕                                 │
│  ┌───────────────────────────────────────────────────┐    │
│  │     AI Worker Process (spawned as child)           │    │
│  │     nimbus-ai/app.py::run_ai_worker()             │    │
│  │     - YOLO object detection                        │    │
│  │     - Depth processing                             │    │
│  │     - Video streaming                              │    │
│  │     - Display service                              │    │
│  └───────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────────┘
                           ↕
                    HTTP @ 100Hz
                           ↕
                 ┌─────────────────┐
                 │ Webots Simulator│
                 └─────────────────┘
```

## Single Startup

**Just run ONE script:**

```bash
cd nimbus-hub
start-hub.bat    # Windows
./start-hub.sh   # Linux/Mac
```

This will:
1. Create virtual environment
2. Install PyTorch with CUDA 12.9
3. Install all dependencies (YOLO, transformers, Flask, etc.)
4. Start unified Flask server on port 5000
5. Spawn AI worker process automatically

## What Changed

### Before (Multiple Processes)
```
Terminal 1: python nimbus-hub/hub.py      (port 8000)
Terminal 2: python nimbus-ai/app.py       (port 5000)
Terminal 3: Webots
```

### After (Single Entry Point)
```
Terminal 1: python nimbus-hub/hub.py      (port 5000, spawns AI automatically)
Terminal 2: Webots
```

## Files Moved

### From nimbus-ai → nimbus-hub

1. **templates/control.html** - Web interface
2. **routes/web_routes.py** - Web routes blueprint
3. **services/drone_state_service.py** - Telemetry service

### Refactored

1. **nimbus-hub/hub.py**
   - Now main Flask server (was just message broker)
   - Spawns AI worker via multiprocessing
   - Manages shared state
   - Hosts web interface
   - Includes heartbeat monitoring

2. **nimbus-ai/app.py**
   - Changed from Flask server to worker function
   - `run_ai_worker(shared_state)` - main entry point
   - Runs in separate process
   - Communicates via shared memory

## Communication Flow

### Hub ↔ AI Worker
- **Type:** `multiprocessing.Manager().dict()`
- **Bidirectional:** Both can read/write
- **Real-time:** Direct memory access, no HTTP overhead

**Shared State Structure:**
```python
shared_state = {
    # Control signals (Hub → AI)
    'cancel': False,              # Cancel button override
    'mode': 'MANUAL',             # Flight mode
    'target_object': 'car',       # AI detection target
    'ai_running': True,           # Worker control flag

    # Results (AI → Hub)
    'ai_results': {},             # Latest detections
    'video_frame': bytes,         # JPEG frame for streaming
    'processing_fps': 30.0,       # AI performance
    'detection_count': 5,         # Objects detected

    # Monitoring (AI → Hub)
    'last_heartbeat': timestamp,  # Runaway prevention
}
```

### Webots → Hub
- **Type:** HTTP POST @ 100Hz
- **Endpoint:** `POST /drone/state`
- **Data:** Complete telemetry (position, velocity, orientation, etc.)

### Hub → Web Interface
- **Telemetry:** SSE stream via `/stream/drone/state`
- **Video:** MJPEG via `/video_feed`
- **Controls:** HTTP POST to `/api/*`

## Key Endpoints

### Message Broker (Existing)
- `POST /publish` - Publish to any topic
- `GET /get/<topic>` - Get latest from topic
- `GET /stream/<topic>` - SSE stream
- `POST /drone/state` - Update drone telemetry

### AI Control (New)
- `POST /api/cancel` - Cancel AI processing
- `POST /api/set_target_object` - Set detection target
- `GET /api/ai_status` - Get AI worker stats

### Web Interface
- `GET /` - Main control interface
- `GET /video_feed` - MJPEG video stream
- `GET /health` - System health check

## Runaway Prevention

### Heartbeat Monitoring
```python
# AI worker updates every 100ms
shared_state['last_heartbeat'] = time.time()

# Hub monitors every 5 seconds
heartbeat_age = time.time() - shared_state['last_heartbeat']
if heartbeat_age > 10:
    logger.warning("AI worker heartbeat stale")
if heartbeat_age > 30:
    logger.error("AI worker appears hung")
```

### Cancel Override
```javascript
// Web interface cancel button
fetch('/api/cancel', {method: 'POST'})
  .then(() => console.log('AI processing cancelled'));

// AI worker checks every loop iteration
if shared_state.get('cancel', False):
    cleanup_and_pause()
    shared_state['cancel'] = False
```

## Performance Metrics

| Component | Update Rate | Latency | Protocol |
|-----------|-------------|---------|----------|
| Webots → Hub | 100Hz | ~5ms | HTTP POST |
| AI Processing | 30 FPS | ~33ms | Internal |
| Video Stream | 30 FPS | ~50ms | MJPEG |
| Telemetry Display | 100Hz | ~10ms | SSE |
| Hub ↔ AI Worker | 100Hz | <1ms | Shared memory |

## Development Workflow

### Running the System

1. **Start Hub (once)**
   ```bash
   cd nimbus-hub
   start-hub.bat
   ```

2. **Start Webots**
   - Load `sim-webot/mavic/worlds/mavic_2_pro.wbt`
   - Press Play ▶️

3. **Access Web Interface**
   - Open browser: http://localhost:5000

### Modifying AI Code

AI code in `nimbus-ai/services/` is hot-reloadable:
1. Edit files in nimbus-ai
2. Stop hub (Ctrl+C)
3. Restart hub
4. Changes take effect immediately

### Debugging

**Hub logs:**
```
INFO - NIMBUS CENTRAL HUB
INFO - Shared state initialized
INFO - AI worker process started (PID: 12345)
INFO - Heartbeat monitor started
```

**AI Worker logs:**
```
INFO - [AI WORKER] - NIMBUS AI WORKER PROCESS STARTING
INFO - [AI WORKER] - Services imported successfully
INFO - [AI WORKER] - All services started successfully
INFO - [AI WORKER] - Entering main processing loop...
```

**Check AI status:**
```bash
curl http://localhost:5000/api/ai_status
```

## Troubleshooting

### Hub won't start
**Error:** `Failed to start AI worker`

**Fix:** Check that nimbus-ai/app.py exists and has `run_ai_worker(shared_state)` function

### AI worker stuck
**Symptom:** Heartbeat warnings in logs

**Fix:** Hub automatically detects via heartbeat monitoring. Restart hub to respawn worker.

### Video not streaming
**Check:**
1. Webots camera publishing to http://localhost:8080/video
2. AI worker video_stream_service running
3. Frames being written to shared_state['video_frame']
4. Access http://localhost:5000/video_feed directly

### Port already in use
**Error:** `Address already in use: 5000`

**Fix:**
```bash
# Windows
netstat -ano | findstr :5000
taskkill /PID <pid> /F

# Linux/Mac
lsof -ti:5000 | xargs kill -9
```

## Migration Notes

### Old Code (nimbus-ai standalone)
```python
from flask import Flask
app = Flask(__name__)
# ... Flask routes ...
app.run(port=5000)
```

### New Code (nimbus-ai as worker)
```python
def run_ai_worker(shared_state):
    # ... worker loop ...
    while shared_state.get('ai_running', True):
        shared_state['last_heartbeat'] = time.time()
        # ... process frames ...
```

## Future Enhancements

1. **Auto-restart AI worker** if heartbeat fails
2. **Multiple AI workers** for parallel processing
3. **GPU affinity control** for worker processes
4. **Dynamic worker scaling** based on load
5. **Process pool** for heavy computations
6. **Watchdog service** for automatic recovery

## Summary

✅ Single startup script
✅ Unified port (5000)
✅ Multiprocessing for parallel AI
✅ Shared memory IPC (fast)
✅ Heartbeat monitoring
✅ Cancel override support
✅ Live video streaming
✅ All dependencies in one venv