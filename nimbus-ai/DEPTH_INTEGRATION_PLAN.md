# Depth Detection Integration Plan

## 🎯 Objective
Integrate depth estimation into the existing object detection pipeline using sequential branching with frame-based collection.

## 🏗️ Architecture Decision

### Sequential Branching (Option 1 - Selected)
```
AI Processing (100ms) → Object Detection → [IF object found AND GLOBAL_GET_DIST=1] → Depth Collection → Combined Result
```

### Class Structure Decision
**Separate Classes (Recommended)**
- Keep `ObjectDetector` class focused on detection only
- Create separate `DepthEstimator` class in `depth_node.py`
- AI.py orchestrates both classes sequentially

**Why not overload ObjectDetector:**
- ✅ Single Responsibility Principle
- ✅ Easier testing and debugging
- ✅ Can enable/disable depth independently
- ✅ Cleaner code organization

## 📋 Implementation Plan

### 1. Global Variables (AI.py)
```python
# Existing
GLOBAL_GET_DIST = 1                    # Enable/disable depth (set to 0 after completion)
GLOBAL_OBJECT = "Person"               # Target object
GLOBAL_TARGET_DISTANCE = 0.0           # Result storage

# New Variables  
DEPTH_FRAME_INTERVAL = 5               # Every 5th successful detection
DEPTH_FRAME_COUNT = 5                  # Collect 5 frames for SfM

# Processing Flags (Critical for preventing overlaps)
OBJECT_DETECTION_BUSY = False          # Prevent object detection overlaps
DEPTH_PROCESSING_BUSY = False          # Prevent depth processing overlaps
```

### 2. Processing Flags
```python
# In AI.py class
self.depth_frame_buffer = []           # Store frames for depth processing
self.detection_counter = 0             # Count successful detections
self.depth_estimator = None            # DepthEstimator instance
```

### 3. Files to Edit/Create

#### A. Edit: `AI.py`
**Changes:**
- Add depth estimator initialization
- Modify `_ai_processing_loop()` to include depth branching
- Add frame collection logic
- Add processing flags

**New Flow:**
```python
def _ai_processing_loop(self):
    while self.ai_thread_running:
        if time_for_processing():
            frame = get_frame()
            
            # Step 1: Object Detection (with safety flag)
            if not OBJECT_DETECTION_BUSY:
                OBJECT_DETECTION_BUSY = True
                try:
                    detection_result = self.object_detector.detect(GLOBAL_OBJECT, frame)
                finally:
                    OBJECT_DETECTION_BUSY = False
                
                if detection_result:  # Object found
                    self.detection_counter += 1
                    
                    # Step 2: Depth Collection (only when GLOBAL_GET_DIST=1)
                    if GLOBAL_GET_DIST == 1 and not DEPTH_PROCESSING_BUSY:
                        # Reuse SAME detection result (no duplicate object detection)
                        # Only increment depth counter when depth is actually needed
                        if self.depth_estimator.should_collect_frame():
                            depth_result = self.depth_estimator.collect_frame(frame, detection_result)
                            if depth_result:  # Batch processing complete (5 frames collected)
                                GLOBAL_TARGET_DISTANCE = depth_result['distance']
                                GLOBAL_GET_DIST = 0  # Turn off depth detection
                                return combine_results(detection_result, depth_result)
                    elif GLOBAL_GET_DIST == 1 and DEPTH_PROCESSING_BUSY:
                        # Skip depth if busy - continue with object detection only
                        logger.debug("Depth processing busy, skipping depth collection")
                    # If GLOBAL_GET_DIST == 0, depth collection is completely skipped
                
                return detection_result
            else:
                # Skip this cycle if object detection is busy - NO WAITING
                logger.debug("Object detection busy, skipping frame")
                # Continue to next 100ms cycle, don't block
                pass
```

#### B. Edit: `classes/depth_node.py`
**Create new class:**
```python
class DepthEstimator:
    def __init__(self):
        self.frame_buffer = []
        self.depth_collection_counter = 0  # Only counts when GLOBAL_GET_DIST=1
        
    def should_collect_frame(self):
        """Check if this depth-enabled detection should trigger frame collection"""
        self.depth_collection_counter += 1
        return self.depth_collection_counter % DEPTH_FRAME_INTERVAL == 0
        
    def collect_frame(self, frame, detection_result):
        """Collect frames for multi-frame depth estimation (batch processing)"""
        if DEPTH_PROCESSING_BUSY:
            return None  # Skip if busy - NO WAITING
            
        # Add frame + detection data to buffer
        frame_data = {
            'frame': frame.copy(),
            'bounding_box': detection_result,
            'timestamp': datetime.now(),
            'frame_id': len(self.frame_buffer) + 1
        }
        self.frame_buffer.append(frame_data)
        
        # Process when we have enough frames
        if len(self.frame_buffer) >= DEPTH_FRAME_COUNT:
            return self.process_depth_batch()
        return None
        
    def process_depth_batch(self):
        """Process all collected frames using SfM (with flag protection)"""
        if DEPTH_PROCESSING_BUSY:
            return None  # Skip if busy
            
        DEPTH_PROCESSING_BUSY = True
        try:
            # Use ALL 5 frames for SfM accuracy
            point_cloud = self._calculate_sfm_from_frames(self.frame_buffer)
            
            # Extract distance from LATEST frame's bounding box
            latest_frame = self.frame_buffer[-1]
            distance = self._extract_depth_at_bbox(point_cloud, latest_frame['bounding_box'])
            
            result = {
                'distance': distance,
                'reference_frame': 'latest',
                'confidence': self._calculate_confidence(),
                'method': 'sfm_multiframe'
            }
            
            # Clear buffer for next batch
            self.frame_buffer.clear()
            return result
            
        finally:
            DEPTH_PROCESSING_BUSY = False
        
    def estimate_distance_object_size(self, detection, frame):
        """Fallback: single frame object size method"""
```

#### C. Edit: `helpers/display.py`
**Changes:**
- Display depth collection status
- Show distance when available
- Update overlay with depth info

### 4. Frame vs Time Decision: **FRAMES** (Selected)

**Chosen: Frame-based counting**
- `DEPTH_FRAME_INTERVAL = 5` (every 5th detection)
- `DEPTH_FRAME_COUNT = 5` (collect 5 frames)

**Why Frames over Time:**
- ✅ Predictable behavior regardless of video FPS
- ✅ Simple counter logic
- ✅ Consistent collection pattern
- ✅ Easier to debug and test

## 🔄 Complete Workflow

### 1. Object Detection Phase
```
Every 100ms:
├─ Get frame from queue
├─ Run object detection
├─ If object found: detection_counter++
└─ Proceed to depth phase (if enabled)
```

### 2. Depth Collection Phase  
```
If object detected AND GLOBAL_GET_DIST=1 AND not DEPTH_PROCESSING_BUSY:
├─ Increment depth_collection_counter (separate from detection_counter)
├─ Check if depth_collection_counter % DEPTH_FRAME_INTERVAL == 0 (every 5th depth-enabled detection)
├─ If yes: Add (frame + detection_result) to depth_frame_buffer
├─ If buffer has DEPTH_FRAME_COUNT frames (5 total): Process all frames in batch
└─ Set DEPTH_PROCESSING_BUSY = True during processing
```

### 3. Depth Processing Phase
```
Process 5 collected frames (batch processing):
├─ Extract features from all 5 frames for SfM accuracy
├─ Calculate camera movement between frames
├─ Triangulate 3D point cloud from multiple viewpoints
├─ Extract distance at LATEST frame's bounding box position
├─ Store result in GLOBAL_TARGET_DISTANCE
├─ Set GLOBAL_GET_DIST = 0 (turn off depth detection)
└─ Set DEPTH_PROCESSING_BUSY = False
```

### 4. Result Integration
```
Return combined result:
├─ Object detection data (bounding box, confidence)
├─ Depth data (distance, method used)
└─ Status flags for display
```

## 🛡️ Safety Mechanisms

### Processing Flags
```python
OBJECT_DETECTION_BUSY = False          # Prevent parallel object detection
DEPTH_PROCESSING_BUSY = False          # Prevent parallel depth processing
is_collecting_frames = False           # Track collection state
```

### Error Handling
- If object detection fails: reset `OBJECT_DETECTION_BUSY` flag in finally block
- If depth estimation fails: reset `DEPTH_PROCESSING_BUSY` flag and continue with object detection only
- If SfM takes too long: timeout and use object size fallback
- Clear buffers on errors to prevent memory buildup

### Skipping Strategy (No Waiting/Blocking)
- **Object detection busy**: Skip frame, continue to next 100ms cycle
- **Depth processing busy**: Skip depth collection, continue with object detection only
- **Frame collection**: Accumulate 5 frames, then batch process all together
- **Distance calculation**: Use latest frame's bounding box for current position

### Exception Safety
- All processing flags use `try/finally` blocks to ensure reset
- Prevents deadlock situations where flags never get cleared
- Graceful degradation if any component fails
- **No duplicate work**: Depth reuses existing object detection results

### Auto-disable
- Set `GLOBAL_GET_DIST = 0` after successful depth calculation
- Prevents continuous depth processing once distance is known

## 📊 Expected Performance

### Timing
- Object detection: ~50ms (unchanged)
- Frame collection: 5 detections × 100ms intervals = 500ms to collect frames
- Depth processing: ~200ms (SfM batch processing) or ~5ms (object size fallback)
- Total depth cycle: ~700ms for high accuracy
- **Distance reference**: Latest frame's object position (most current)

### Frame Collection Timeline (Only when GLOBAL_GET_DIST=1)
```
Depth-enabled detection 1:  Skip (1/5)
Depth-enabled detection 2:  Skip (2/5)
Depth-enabled detection 3:  Skip (3/5)
Depth-enabled detection 4:  Skip (4/5)
Depth-enabled detection 5:  Collect frame #1 → Buffer: [1]
Depth-enabled detection 10: Collect frame #2 → Buffer: [1,2] 
Depth-enabled detection 15: Collect frame #3 → Buffer: [1,2,3]
Depth-enabled detection 20: Collect frame #4 → Buffer: [1,2,3,4]
Depth-enabled detection 25: Collect frame #5 → Buffer: [1,2,3,4,5] → PROCESS BATCH → Return distance from frame #5
```

**Key Point**: Depth collection counter only increments when `GLOBAL_GET_DIST = 1`. If depth is disabled, no counting or collection happens.

### Memory
- Frame buffer: 5 frames × ~2MB = ~10MB temporary storage
- Clear buffer after each depth calculation

## 🧪 Testing Strategy

### 1. Unit Testing
- Test `DepthEstimator` class independently
- Test frame collection logic
- Test processing flags

### 2. Integration Testing  
- Test with webcam (hand movement)
- Test flag transitions
- Test error scenarios

### 3. Performance Testing
- Measure processing times
- Test memory usage
- Verify no processing overlaps

## 📝 Implementation Order

1. **Create `DepthEstimator` class** in `depth_node.py`
2. **Add global variables** to `AI.py`
3. **Modify AI processing loop** for sequential branching
4. **Add frame collection logic**
5. **Integrate depth processing**
6. **Update display system** to show depth status
7. **Test and debug** complete pipeline

---

This plan provides a clean, safe, and maintainable approach to depth estimation integration while preserving the existing object detection performance.