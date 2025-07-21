# Nimbus AI Container Documentation

## Overview
The Nimbus AI container is a modular Python layer designed for drone control operations. It consists of independent processing nodes that can be orchestrated by a ROS2 container to enable natural language-based drone navigation and environmental understanding.

## Architecture
The AI container follows a **node-based modular architecture** where each Python file represents a discrete processing unit with defined inputs and outputs. This design enables:
- **Modularity**: Each node can be developed, tested, and deployed independently
- **Scalability**: Nodes can be distributed across different containers or processes
- **Maintainability**: Clear separation of concerns with defined interfaces
- **ROS2 Integration**: Easy integration as ROS2 services or action servers

## AI Processing Nodes

### 1. Speech-to-Text Node (`stt_node.py`)
**Purpose**: Converts audio input to text transcript
- **Technology**: Whisper (HuggingFace model)
- **Input**: Microphone audio stream
- **Output**: Text transcript
- **Use Case**: First step in the voice command pipeline

```python
# Expected signature:
def stt_node(audio_data):
    # Process audio with Whisper model
    return transcript
```

### 2. Intent & Object Extraction Node (`intent_object_node.py`)
**Purpose**: Extracts both user intent and target objects from natural language in a single pass
- **Technology**: Ollama with structured system prompt
- **Input**: Text transcript
- **Output**: JSON response containing intent and object
- **Use Case**: Efficiently parses commands like "go to the chair" into structured data

**System Prompt**:
```
you are an AI interlayer for a drone. you need to extract entities from a user input.
your response should be in JSON format.
response:
{
"intent": (IN "Go", "Cancel", "Home")
"Object": (like "table", "chair", "stairs")
}
```

```python
# Expected signature:
def intent_object_node(transcript):
    # Extract both intent and object using LLM with structured prompt
    return {
        "intent": "Go",
        "object": "chair"
    }
```

### 3. Object Detection Node (`object_detect_node.py`)
**Purpose**: Visual object detection in camera feed
- **Technology**: YOLO "latest" (likely YOLOv8)
- **Input**: Intent, object name, video stream
- **Output**: Bounding box coordinates
- **Use Case**: Locates target objects visually in the environment

```python
# Expected signature:
def object_detect_node(intent, object_name, video_frame):
    # Run YOLO detection
    return bounding_box
```

### 4. Depth Estimation Node (`depth_node.py`)
**Purpose**: Calculates 3D position of detected objects
- **Technology**: OpenCV, RTAB-Map
- **Input**: Intent, bounding box, video stream, camera pose
- **Output**: Object pose (3D position and orientation)
- **Use Case**: Provides spatial coordinates for navigation

```python
# Expected signature:
def depth_node(intent, bounding_box, video_frame, camera_pose):
    # Calculate 3D position using depth data
    return object_pose
```

### 5. RTAB-Map SLAM Node (`rtabmap_node.py`)
**Purpose**: Simultaneous Localization and Mapping
- **Technology**: RTAB-Map
- **Input**: SLAM video, depth camera data
- **Output**: Camera pose (position and orientation in 3D space)
- **Use Case**: Maintains spatial awareness and camera tracking

```python
# Expected signature:
def rtabmap_node(video_stream, depth_data):
    # Perform SLAM operations
    return camera_pose
```

### 6. Survey Node (`survey_node.py`)
**Purpose**: Comprehensive environment scanning
- **Technology**: Loops YOLO, Depth node, RTAB-Map node
- **Input**: Intent="survey", video stream, depth camera, camera pose
- **Output**: Scene objects (array of object poses)
- **Use Case**: Creates a complete 3D map of all objects in the environment

```python
# Expected signature:
def survey_node(video_stream, depth_data, camera_pose):
    # Orchestrate multiple detection cycles
    return scene_objects_array
```

## Data Flow Pipeline

### Standard Command Processing Flow:
1. **Audio Input** → `stt_node()` → **Transcript**
2. **Transcript** → `intent_object_node()` → **Intent + Object (JSON)**
3. **Intent + Object + Video** → `object_detect_node()` → **Bounding Box**
4. **Intent + Bounding Box + Video + Camera Pose** → `depth_node()` → **Object Pose**
5. **Video + Depth Data** → `rtabmap_node()` → **Camera Pose** (continuous)

### Survey Mode Flow:
1. **"Survey" Intent** → `survey_node()` 
2. **Survey Node** internally loops:
   - `object_detect_node()` for multiple objects
   - `depth_node()` for each detection
   - `rtabmap_node()` for continuous pose updates
3. **Output**: Complete scene understanding

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
```

### Docker Integration
The AI container runs independently and communicates with the ROS2 container through:
- **HTTP APIs** (current Flask implementation)
- **ROS2 Services/Actions** (recommended for production)
- **Message Queues** (Redis/RabbitMQ for high-throughput scenarios)

## Current Implementation Status

### ✅ Completed
- Basic Flask web server structure
- Docker containerization setup
- Modular file organization
- Inter-service communication framework

### 🚧 In Progress
- Individual node implementations (currently placeholder functions)
- AI model integration
- ROS2 service wrappers

### 📋 TODO
- Implement actual AI processing in each node
- Add error handling and logging
- Performance optimization
- ROS2 message type definitions
- Integration testing with drone hardware

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