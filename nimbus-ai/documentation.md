# Nimbus AI Container Documentation

## Overview
The Nimbus AI container is a parallel processing system designed for real-time drone control operations. It features **dual-pipeline architecture** combining live video streaming with periodic AI processing and activation-based voice command processing. The system provides responsive visual feedback while maintaining computational efficiency.

## System Architecture
The AI container employs a **parallel dual-pipeline architecture**:

### Video Pipeline (Continuous)
- **Live Video Stream**: Continuous video feed from ROS2 at full framerate
- **Periodic AI Processing**: Frame extraction every 200ms (configurable) for AI analysis
- **Overlay System**: Real-time overlay of bounding boxes and AI results on live video
- **Visual Feedback**: Immediate visual response with 200ms refresh rate for AI overlays

### Audio Pipeline (Event-Driven)
- **Activation Detection**: Listens for activation phrases ("ok drone", configurable list)
- **Voice Capture**: Records audio snippet after activation until user pause
- **Intent Processing**: Sends transcript to Ollama for structured output extraction
- **Global State Management**: Sets persistent global variables (intent, object) for flight duration

This architecture enables:
- **Real-time Responsiveness**: Live video with minimal latency
- **Computational Efficiency**: AI processing only on selected frames
- **Natural Interaction**: Voice activation system similar to smart assistants
- **Persistent State**: Command context maintained throughout flight

## Configuration Variables

### Global Processing Parameters
```python
# Frame processing interval (default: 200ms)
FRAME_PROCESSING_INTERVAL_MS = 200

# Voice activation phrases (expandable list)
ACTIVATION_PHRASES = ["ok drone", "hey nimbus", "drone activate"]

# Global state variables (persistent during flight)
GLOBAL_INTENT = ""      # Current flight intent: "go", "stop", "cancel", "home", "land"
GLOBAL_OBJECT = ""      # Current target object: "chair", "table", etc.
GLOBAL_STATE_ACTIVE = False  # Whether global state is set and active
```

## AI Processing Nodes

### 1. Object Detection Node (`object_detect_node.py`)
**Purpose**: Processes individual frames for object detection and bounding box generation
- **Technology**: YOLOv8 (ultralytics)
- **Trigger**: Called every 200ms on extracted video frames
- **Input**: Intent, target object, single video frame
- **Output**: Bounding box coordinates (x, y, width, height)
- **Use Case**: Creates overlay data for live video feed

```python
# Current signature:
def object_detect_node(intent: str, target_object: str, video_frame: np.ndarray) -> dict:
    # Processes single frame with YOLO
    # Returns: {'x': x1, 'y': y1, 'width': w, 'height': h} or {}
```

### 2. Speech-to-Text Node (`stt_node.py`) 
**Purpose**: Converts voice snippets to text after activation phrase detection
- **Technology**: Whisper (OpenAI/HuggingFace)
- **Trigger**: Only after activation phrase detected
- **Input**: Audio snippet (from activation to user pause)
- **Output**: Text transcript
- **Use Case**: Voice command transcription for intent extraction

```python
# Current signature:
def stt_node(audio_data: np.ndarray, sample_rate: int = 44100) -> str:
    # Processes audio snippet with Whisper
    # Returns: "go to the chair" (example transcript)
```

### 3. Intent & Object Extraction Node (`intent_object_node.py`)
**Purpose**: Extracts structured commands from voice transcripts using LLM
- **Technology**: Ollama (llama3:latest) with structured prompting
- **Trigger**: Called after successful voice transcription
- **Input**: Text transcript from STT
- **Output**: Structured JSON with intent and object
- **Use Case**: Converts natural language to actionable drone commands

**Supported Intents**: go, stop, cancel, home, land

```python
# Current signature:
def intent_object_node(transcript: str) -> Dict[str, str]:
    # Uses Ollama with structured system prompt
    # Returns: {"intent": "go", "object": "chair"}
```

### 4. Depth Estimation Node (`depth_node.py`)
**Purpose**: Calculates initial 3D distance to target object when command is first given
- **Technology**: OpenCV, RTAB-Map integration
- **Trigger**: Once when new intent/object command is processed (not during tracking)
- **Input**: Bounding box, video frame, camera pose
- **Output**: Object distance and 3D position
- **Use Case**: Provides initial target distance for navigation planning

```python
# Expected signature:
def depth_node(intent, bounding_box, video_frame, camera_pose):
    # Calculate initial 3D position and distance to target
    return {"distance": float, "position": {"x": float, "y": float, "z": float}}
```

### 5. RTAB-Map SLAM Node (`rtabmap_node.py`) - Background Process
**Purpose**: Continuous localization and mapping (runs independently)
- **Technology**: RTAB-Map
- **Trigger**: Continuous background processing
- **Input**: Video stream, depth camera data
- **Output**: Camera pose (position and orientation in 3D space)
- **Use Case**: Maintains spatial awareness for depth calculations

### 6. Survey Node (`survey_node.py`) - Future Enhancement
**Purpose**: Comprehensive environment scanning (special command mode)
- **Technology**: Coordinates multiple detection cycles
- **Trigger**: When global intent is set to "survey"
- **Input**: Video stream, depth camera, camera pose
- **Output**: Scene objects with distances (array of object poses)
- **Use Case**: Creates complete 3D map of environment objects

## Data Flow Pipeline

### Parallel Processing Architecture

#### Video Pipeline (Continuous - Live Feed with Overlays)
```
Live Video Stream (ROS2) → AI.py → Display with Overlays
    ↓ (every 200ms)
Extract Frame → object_detect_node() → Bounding Box → Overlay on Live Video
    ↑ (uses current GLOBAL_INTENT & GLOBAL_OBJECT)
```

#### Audio Pipeline (Event-Driven - Voice Commands)
```
Microphone → Activation Detection ("ok drone") → Voice Recording → stt_node() → Transcript
    ↓
intent_object_node() → {"intent": "go", "object": "chair"} → Update Global Variables
    ↓ (when new command issued)
GLOBAL_INTENT = "go", GLOBAL_OBJECT = "chair" → Triggers initial depth calculation
    ↓
object_detect_node() → Bounding Box → depth_node() → Target Distance → Navigation Start
```

### Command Processing Flow:
1. **Voice Activation**: "ok drone" detected → Start recording
2. **Voice Capture**: Record until user pause → Audio snippet
3. **Transcription**: `stt_node(audio_snippet)` → "go to the chair"
4. **Intent Extraction**: `intent_object_node(transcript)` → {"intent": "go", "object": "chair"}
5. **Global State Update**: Set GLOBAL_INTENT="go", GLOBAL_OBJECT="chair"
6. **Initial Detection**: `object_detect_node()` finds chair → Bounding box
7. **Distance Calculation**: `depth_node()` → Target distance (one-time)
8. **Navigation Start**: Send target distance to drone navigation
9. **Continuous Tracking**: object_detect_node() every 200ms for visual feedback

### Background Processes:
- **RTAB-Map SLAM**: Continuous camera pose tracking
- **Live Video**: Uninterrupted feed with 200ms overlay refresh
- **Global State**: Persistent intent/object variables during flight

## ROS2 Integration Strategy

### Service-Based Architecture
Each AI node can be wrapped as a ROS2 service:

```python
# Example ROS2 service wrapper
class IntentObjectNodeService(Node):
    def __init__(self):
        super().__init__('intent_object_node_service')
        self.srv = self.create_service(
            IntentObjectExtraction, 
            'extract_intent_object', 
            self.intent_object_callback
        )
    
    def intent_object_callback(self, request, response):
        result = intent_object_node(request.transcript)
        response.intent = result["intent"]
        response.object = result["object"]
        return response
```

### Action-Based Architecture (Recommended)
For long-running processes like survey operations:

```python
# Example ROS2 action server
class SurveyNodeAction(Node):
    def __init__(self):
        super().__init__('survey_node_action')
        self._action_server = ActionServer(
            self,
            SurveyEnvironment,
            'survey_environment',
            self.survey_callback
        )
```

## Container Configuration

### Dependencies (requirements.txt)
Based on the technology stack:
```txt
flask==3.0.1
gunicorn==21.2.0
faster-whisper
transformers
opencv-python
ultralytics
requests
numpy
torch
torchvision
ollama-python
websocket-client  # For ROS2 bridge communication
cv2  # OpenCV for camera handling
```

### ROS2 Container Setup
The system uses `introlab3it/rtabmap_ros:humble` container with the following configuration:

#### Required ROS2 Packages:
- **RTAB-Map**: Already included in base image
- **ROS Bridge Suite**: `ros-humble-rosbridge-suite` (auto-installed on startup)

#### Container Ports:
- **11311**: ROS Master
- **9090**: ROS Bridge WebSocket (for Python ↔ ROS2 communication)

#### Startup Command:
```bash
source /opt/ros/humble/setup.bash && 
apt update && 
apt install -y ros-humble-rosbridge-suite && 
ros2 run rosbridge_server rosbridge_websocket --ros-args -p port:=9090 & 
tail -f /dev/null
```

### Docker Integration
The AI container communicates with ROS2 container through:
- **WebSocket ROS Bridge** (port 9090) - Primary method for video pipeline
- **HTTP APIs** (Flask implementation for other services)
- **ROS2 Topics**:
  - `/camera/image_raw` - Camera feed from publisher to AI
  - `/nimbus/ai/slam_pose` - SLAM results from AI
  - `/nimbus/ai/object_detection` - Object detection results
  - `/nimbus/ai/processing_status` - AI processing status

## Video Pipeline Architecture

### Camera Feed Integration
The system supports camera input through a modular pipeline:

#### Camera Publisher (`scripts/simple_camera_publisher.py`)
- Captures video from Camo Studio (or other cameras)
- Publishes to ROS2 topic `/camera/image_raw`
- Uses WebSocket connection to ROS2 bridge

#### AI Subscriber (`simple_ai_subscriber.py`) 
- Subscribes to camera feed from ROS2
- Processes frames through AI.py
- Displays results with overlays

#### File Structure:
```
nimbus-ai/
├── AI.py                           # Central AI coordinator
├── helpers/
│   ├── phone_camera.py            # Camera interface
│   └── display.py                 # Display overlays
├── classes/                       # AI processing nodes
└── scripts/                       # Application entry points
    ├── simple_camera_publisher.py # Camera → ROS2
    ├── simple_ai_subscriber.py   # ROS2 → AI → Display
    └── run_video_system.py       # Coordinated startup
```

### Camera Setup Requirements:
1. **Camo Studio** or compatible camera app
2. **Camera must be free** (not used by other applications)
3. **WebSocket client**: `pip install websocket-client`

### Running the Video System:
```bash
# Start ROS2 container
docker-compose up -d ros2-central

# Install Python dependencies
pip install websocket-client opencv-python

# Run complete video pipeline
cd nimbus-ai
python run_video_system.py
```

## Quick Start Installation Guide

### 1. Docker Setup
```bash
# Production (all containers)
docker-compose up -d

# Development (only ROS2 + Ollama)  
docker-compose -f docker-compose.dev.yml up -d
```

### 2. ROS2 Container Verification
```bash
# Check if rosbridge is running
curl http://localhost:9090
# Should return: Can "Upgrade" only to "WebSocket".

# Check ROS2 topics
docker exec ros2-central bash -c "source /opt/ros/humble/setup.bash && ros2 topic list"
```

### 3. Python Environment Setup
```bash
# Required packages for video pipeline
pip install websocket-client opencv-python numpy

# Optional: AI processing dependencies  
pip install transformers torch ultralytics faster-whisper
```

### 4. Camera Setup
1. Install Camo Studio app on phone
2. Connect phone to same network as PC
3. **Close Camo Studio app completely** before running pipeline
4. Run camera diagnostic: `python camera_diagnostic.py`

### 5. Run Video System
```bash
cd nimbus-ai
python run_video_system.py
```

### Troubleshooting
- **Camera Error -1072875772**: Camo Studio app is still running - close it completely
- **WebSocket Connection Failed**: ROS2 container not running or rosbridge not started
- **No Video Display**: Check if both publisher and subscriber processes are running

## Current Implementation Status

### ✅ Completed
- Basic Flask web server structure
- Docker containerization setup with ROS2 integration
- Modular file organization with proper directory structure
- **ROS2 Bridge WebSocket communication** 
- **Camera feed pipeline** (Camo Studio → ROS2 → AI)
- **Video display system** with processing overlays
- **Phone camera helper** with multiple connection methods
- **RTAB-Map node architecture** (modular design ready)
- **Intent object extraction** with Ollama integration (limited to 5 intents)
- **Coordinated startup scripts** for complete system

### 🚧 In Progress  
- Individual node implementations (AI processing logic)
- Full AI model integration (requires dependencies)
- ROS2 service wrappers for production deployment

### 📋 TODO
- Install AI dependencies (transformers, whisper, ultralytics)
- Implement actual AI processing in each node
- Add comprehensive error handling and logging
- Performance optimization for video processing
- ROS2 message type definitions
- Integration testing with drone hardware
- Production deployment configuration

### 🎥 Video Pipeline Status
- ✅ **Camera Publisher**: Captures and publishes to ROS2
- ✅ **AI Subscriber**: Receives from ROS2 and displays
- ✅ **ROS2 Bridge**: WebSocket communication working
- ✅ **Display System**: Overlays and processing status
- ⚠️ **Camera Access**: Requires Camo Studio app to be closed
- 📋 **AI Processing**: Awaiting model dependencies

## Development Recommendations

1. **Implement Nodes Incrementally**: Start with `stt_node` and `intent_object_node` as they're the foundation
2. **Add Comprehensive Logging**: Each node should log inputs, outputs, and processing time
3. **Create Unit Tests**: Test each node independently with mock data
4. **Add Configuration Management**: Use environment variables or config files for model paths
5. **Implement Health Checks**: Each node should have a health endpoint for monitoring
6. **Add Performance Metrics**: Monitor processing time and resource usage
7. **Error Recovery**: Implement graceful degradation when nodes fail

## Example Usage

```python
# Complete pipeline example
audio = capture_microphone()
transcript = stt_node(audio)
intent_object_result = intent_object_node(transcript)
intent = intent_object_result["intent"]
target_object = intent_object_result["object"]

video_frame = capture_camera()
bounding_box = object_detect_node(intent, target_object, video_frame)
camera_pose = rtabmap_node(video_frame, depth_data)
object_pose = depth_node(intent, bounding_box, video_frame, camera_pose)

# Send object_pose to ROS2 navigation stack
navigate_to_pose(object_pose)
```

This modular architecture provides a solid foundation for building an intelligent drone control system that can understand natural language commands and execute complex navigation tasks.