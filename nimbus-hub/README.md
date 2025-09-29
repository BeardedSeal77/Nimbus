# Nimbus Central Communication Hub

Fast, lightweight message broker for real-time communication between all Nimbus components.

## Architecture

```
Webots (simulation)  ──> Hub (port 8000) ──> nimbus-ai (web interface)
                              ↓
                         nimbus-robotics (PID control)
```

## Features

- **Fast**: HTTP-based pub/sub, 100Hz telemetry updates (10ms interval)
- **Simple**: No Docker/ROS2 overhead
- **Real-time**: Server-Sent Events (SSE) for live streaming
- **Flexible**: Topic-based architecture, easy to add new components

## Quick Start

### Windows
```bash
start-hub.bat
```

### Linux/Mac
```bash
chmod +x start-hub.sh
./start-hub.sh
```

Or manually:
```bash
pip install -r requirements.txt
python hub.py
```

## API Endpoints

### Core Pub/Sub

**POST /publish**
Publish data to any topic
```json
{
  "topic": "drone/state",
  "message": {"position": {"x": 1.0, "y": 2.0, "z": 3.0}}
}
```

**GET /get/<topic>**
Get latest data from a topic
```json
{
  "status": "ok",
  "topic": "drone/state",
  "message": {...},
  "timestamp": 1234567890.123
}
```

**GET /topics**
List all active topics

**GET /stream/<topic>**
Server-Sent Events stream for real-time updates

### Drone-Specific

**POST /drone/state**
Shortcut for publishing complete drone state

**GET /drone/state**
Shortcut for getting drone state

**POST /control/command**
Shortcut for publishing control commands

### Monitoring

**GET /health**
Health check and connection info

## Usage Example

### Publishing (from Webots)
```python
import requests

state = {
    'position': {'x': 1.0, 'y': 2.0, 'z': 3.0},
    'velocity': {'x': 0.1, 'y': 0.0, 'z': 0.0},
    # ... other fields
}

requests.post('http://localhost:8000/drone/state', json=state, timeout=0.005)
```

### Subscribing via SSE (from nimbus-ai)
```python
import requests

response = requests.get('http://localhost:8000/stream/drone/state', stream=True)

for line in response.iter_lines(decode_unicode=True):
    if line and line.startswith('data: '):
        data = json.loads(line[6:])
        print(f"Received: {data}")
```

## Performance

- **Update rate**: 100Hz (10ms intervals)
- **Latency**: <5ms for HTTP POST
- **No blocking**: 5ms timeout prevents control loop delays

## Integration Status

- ✅ Webots → Hub (100Hz telemetry)
- ✅ nimbus-ai → Hub (SSE subscription)
- ⏳ nimbus-robotics → Hub (pending)
- ⏳ ROS2 compatibility mode (30Hz legacy support)