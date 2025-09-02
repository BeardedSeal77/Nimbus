# ROS2 Publishers and Subscribers Documentation

```
                    ╔═══════════════════════════════════╗
                    ║            ROS2 CORE              ║
                    ║                                   ║
                    ║  Host: localhost (default)       ║
                    ║        ros2-central (docker)     ║
                    ║  Port: 9090 (rosbridge)          ║
                    ║  WebSocket: ws://localhost:9090   ║
                    ║  Type: rosbridge websocket        ║
                    ╚══════════════╤════════════════════╝
                                   │
            ┌──────────────────────┼──────────────────────┐
            │                      │                      │
            ▼                      ▼                      ▼
    ╔═══════════════╗     ╔═══════════════╗     ╔═══════════════╗
    ║      AI       ║     ║   ROBOTICS    ║     ║      MR       ║
    ║               ║     ║               ║     ║               ║
    ║ SUBSCRIBERS:  ║     ║ PUBLISHERS:   ║     ║ PUBLISHERS:   ║
    ║ • /camera/    ║     ║ • /camera/    ║     ║ • /microphone/║
    ║   image_raw   ║     ║   image_raw   ║     ║   audio_raw   ║
    ║ • /microphone/║     ║               ║     ║               ║
    ║   audio_raw   ║     ║ SUBSCRIBERS:  ║     ║ SUBSCRIBERS:  ║
    ║               ║     ║ • /drone/     ║     ║ • /ai/        ║
    ║ PUBLISHERS:   ║     ║   commands    ║     ║   processed_  ║
    ║ • /ai/        ║     ║               ║     ║   video       ║
    ║   processed_  ║     ║               ║     ║               ║
    ║   video       ║     ║               ║     ║               ║
    ║ • /drone/     ║     ║               ║     ║               ║
    ║   commands    ║     ║               ║     ║               ║
    ╚═══════════════╝     ╚═══════════════╝     ╚═══════════════╝
```

<div style="page-break-before: always;"></div>

## Detailed Topic Information

### AI Block (nimbus-ai)
**Subscribers:**
- **Topic:** `/camera/image_raw`
  - **Type:** `sensor_msgs/Image`
  - **Encoding:** JPEG
  - **Purpose:** Raw video feed from camera for AI processing
- **Topic:** `/microphone/audio_raw`
  - **Type:** `audio_msgs/AudioData`
  - **Purpose:** Raw audio input from microphone for speech processing

**Publishers:**
- **Topic:** `/ai/processed_video`
  - **Type:** `sensor_msgs/Image`
  - **Encoding:** JPEG
  - **Purpose:** AI-processed video with object detection overlays
- **Topic:** `/drone/commands`
  - **Type:** `geometry_msgs/Twist`
  - **Purpose:** Movement commands for drone navigation

---

### Robotics Block (nimbus-robotics)
**Publishers:**
- **Topic:** `/camera/image_raw`
  - **Type:** `sensor_msgs/Image`
  - **Encoding:** JPEG
  - **Purpose:** Raw video feed from drone/robot camera

**Subscribers:**
- **Topic:** `/drone/commands`
  - **Type:** `geometry_msgs/Twist`
  - **Purpose:** Receive navigation and control commands from AI

---

### MR Block (nimbus-mr)
**Publishers:**
- **Topic:** `/microphone/audio_raw`
  - **Type:** `audio_msgs/AudioData`
  - **Purpose:** Audio input from mixed reality interface microphone

**Subscribers:**
- **Topic:** `/ai/processed_video`
  - **Type:** `sensor_msgs/Image`
  - **Encoding:** JPEG
  - **Purpose:** Receive AI-processed video for MR visualization

<div style="page-break-before: always;"></div>

## Implementation Example

### AI Block Subscription to `/camera/image_raw` (Reference Implementation)

```python
def _setup_subscriptions(self):
    """Subscribe to ROS2 camera topic"""
    # Define subscription message for rosbridge
    subscribe_msg = {
        "op": "subscribe",              # rosbridge operation type
        "topic": "/camera/image_raw",   # Topic name to subscribe to
        "type": "sensor_msgs/Image"     # ROS2 message type
    }
    # Send subscription request via WebSocket
    self._send_ros_message(subscribe_msg)
    logger.info("Subscribed to /camera/image_raw")

def _on_ws_message(self, ws, message):
    """Handle WebSocket messages from ROS2"""
    try:
        # Parse incoming JSON message from rosbridge
        msg = json.loads(message)
        
        # Check if this is our camera topic
        if msg.get('topic') == '/camera/image_raw':
            self._handle_camera_frame(msg)
        
    except json.JSONDecodeError:
        logger.debug("Invalid JSON message")
    except Exception as e:
        logger.error(f"Error handling ROS2 message: {e}")

def _handle_camera_frame(self, ros_msg):
    """Handle incoming camera frame from ROS2"""
    try:
        # Increment frame counter for statistics
        self.frames_received += 1
        
        # Extract message data (contains the actual image)
        msg_data = ros_msg.get('msg', {})
        
        # Decode the ROS image to OpenCV format
        frame = self._decode_ros_image(msg_data)
        
        if frame is not None:
            # Process the frame through AI pipeline
            from services.ai_service import add_frame_for_processing
            add_frame_for_processing(frame)
            
            # Update display with new frame
            from services.display_service import update_current_frame
            update_current_frame(frame)
            
    except Exception as e:
        logger.error(f"Error handling camera frame: {e}")

def _decode_ros_image(self, image_msg):
    """Decode ROS Image message to OpenCV frame"""
    try:
        # Extract image properties from ROS message
        width = image_msg.get('width', 0)
        height = image_msg.get('height', 0)
        encoding = image_msg.get('encoding', '')  # e.g., 'jpeg'
        data = image_msg.get('data', '')          # base64 encoded image
        
        if not data:
            return None
        
        # Decode base64 image data
        image_data = base64.b64decode(data)
        
        if encoding == 'jpeg':
            # Convert to numpy array and decode JPEG
            nparr = np.frombuffer(image_data, np.uint8)
            frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            return frame
        else:
            logger.debug(f"Unsupported encoding: {encoding}")
            return None
            
    except Exception as e:
        logger.error(f"Error decoding ROS image: {e}")
        return None
```

<div style="page-break-before: always;"></div>

### Robotics Block Publisher to `/camera/image_raw` (Reference Implementation)

```python
def _advertise_topic(self):
    """Advertise the camera topic to ROS2"""
    # Define advertisement message for rosbridge
    advertise_msg = {
        "op": "advertise",                # rosbridge operation type
        "topic": "/camera/image_raw",     # Topic name to publish to
        "type": "sensor_msgs/Image"       # ROS2 message type
    }
    # Send advertisement request via WebSocket
    self._send_ros_message(advertise_msg)
    logger.info("Camera topic advertised: /camera/image_raw")

def publish_frame(self, frame: np.ndarray) -> bool:
    """Publish a single frame to ROS2"""
    if not self.is_publishing or not self.ws_connected:
        return False
    
    try:
        # Encode frame as JPEG for transmission
        encode_param = [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
        _, encoded_frame = cv2.imencode('.jpg', frame, encode_param)
        frame_data = base64.b64encode(encoded_frame).decode('utf-8')
        
        # Get frame properties
        height, width = frame.shape[:2]
        timestamp = time.time()
        
        # Create ROS2 Image message (sensor_msgs/Image format)
        ros_image_msg = {
            "op": "publish",                # rosbridge publish operation
            "topic": "/camera/image_raw",   # Topic to publish to
            "msg": {
                "header": {
                    "stamp": {
                        "sec": int(timestamp),
                        "nanosec": int((timestamp % 1) * 1e9)
                    },
                    "frame_id": "camera_frame"
                },
                "height": height,            # Image height in pixels
                "width": width,              # Image width in pixels  
                "encoding": "jpeg",          # Image encoding format
                "is_bigendian": 0,          # Byte order
                "step": len(encoded_frame),  # Bytes per row
                "data": frame_data           # Base64 encoded image data
            }
        }
        
        # Send to ROS2 via WebSocket
        self._send_ros_message(ros_image_msg)
        
        # Update statistics
        self.frames_published += 1
        self.last_publish_time = timestamp
        
        return True
        
    except Exception as e:
        self.publish_errors += 1
        return False

def _send_ros_message(self, message: dict):
    """Send message to ROS2 via websocket"""
    if self.ws_connected and self.ws:
        try:
            # Send JSON message via WebSocket
            self.ws.send(json.dumps(message))
        except Exception as e:
            self.publish_errors += 1
```

<div style="page-break-before: always;"></div>

### Key Implementation Notes for Other Teams:

**For Subscribers:**
1. **Subscription Setup:** Use rosbridge `subscribe` operation with topic name and message type
2. **Message Filtering:** Check `msg.get('topic')` to handle specific topics
3. **Data Extraction:** Access actual data via `ros_msg.get('msg', {})`
4. **Error Handling:** Wrap all operations in try-catch blocks
5. **Base64 Decoding:** Image data comes as base64, needs decoding before use

**For Publishers:**
1. **Advertisement:** Use rosbridge `advertise` operation to announce your topic
2. **Message Format:** Create proper ROS2 message structure with `op`, `topic`, and `msg` fields
3. **Data Encoding:** Encode binary data (images) as base64 for JSON transmission
4. **Timestamps:** Include proper ROS2 header with timestamp for synchronization
5. **Rate Limiting:** Control publishing rate to prevent overwhelming subscribers

## Connection Status Monitoring

The system tracks:
- Connection status (`ws_connected`)
- Frames received counter
- FPS calculation (logged every 30 frames)
- Service uptime
- Reconnection logic with 5-second retry intervals
