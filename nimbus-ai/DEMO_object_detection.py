"""
================================================================================
NIMBUS AI - OBJECT DETECTION COMPONENT DEMO
================================================================================

This module demonstrates the Object Detection segment of the Nimbus AI system.

SYSTEM OVERVIEW:
    The Object Detection component is part of the Video Pipeline in the AI Process.
    It processes video frames to identify objects in real-time for drone navigation.

PIPELINE FLOW (from flowchart):
    Raw Video -> Frame Capture (every 100ms) -> Object Detection ->
    Bounding Box Output -> Central Hub

KEY TECHNOLOGIES:
    - YOLOv11n (Ultralytics): State-of-the-art real-time object detection
    - CUDA/GPU Acceleration: Optimized for RTX 4070 Super
    - FP16 Inference: Half-precision for maximum speed

OUTPUT:
    - Bounding box coordinates (x, y, width, height)
    - Sent to Central Hub for integration with depth detection and robotics
================================================================================
"""

import cv2
import numpy as np
from ultralytics import YOLO


class ObjectDetector:
    """
    Object Detection Node for Nimbus AI System

    Purpose:
        Detect specific objects in video frames and return their bounding box
        coordinates for downstream processing by the robotics and MR branches.

    Integration:
        - Input: Raw video frames from drone camera (via Central Hub)
        - Output: Bounding box coordinates to Central Hub
        - Works asynchronously with Depth Detection node
    """

    def __init__(self):
        """
        Initialize the YOLO object detection model with GPU optimization

        Model Selection:
            - yolo11n.pt: Nano model for maximum speed (chosen for real-time)
            - Alternative: yolo11s/m/l for higher accuracy at cost of speed

        GPU Optimization:
            - Automatically detects CUDA-capable GPU
            - Falls back to CPU if GPU unavailable
            - Uses FP16 (half-precision) for 2x inference speed
        """
        import torch

        # Load YOLOv11 nano model (smallest, fastest variant)
        self.model = YOLO("yolo11n.pt")

        # Configure device (GPU preferred for real-time performance)
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'

        if self.device == 'cuda':
            print(f"GPU detected: {torch.cuda.get_device_name(0)}")
            print(f"CUDA version: {torch.version.cuda}")
        else:
            print("WARNING: Running on CPU - performance will be degraded")

        # Store last results for visualization
        self.last_results = None

        print(f"YOLO initialized on {self.device} with FP16 acceleration")
        print("Ready for real-time object detection")



# -------------------------------------------------------------------------------------
# Core detection method - finds target object in image and returns coordinates

# This is the main entry point used by the Central Hub to process frames.

# Parameters:
#     target_object (str): Object to find (e.g., "chair", "person", "cup")
#                        - User provides this via voice command
#                        - Comes from LLM classification in Audio Pipeline

#     image_source: Video frame to analyze
#                  - Can be: file path, URL, PIL image, or numpy array
#                  - In production: numpy array from video stream

# Returns:
#     dict: Bounding box coordinates
#           Format: {'x': x, 'y': y, 'width': w, 'height': h}
#           Empty dict {} if object not found

# Integration Point:
#     These coordinates are sent to:
#     1. Central Hub -> Global Variables Storage
#     2. Depth Detection Node (for distance calculation)
#     3. MR Branch (for Unity overlay visualization)
#     4. Robotics Branch (for navigation toward object)

    def detect(self, target_object: str, image_source) -> dict:
        try:
            # Run YOLO inference on the frame
            # Configuration:
            #   - device: GPU (cuda) or CPU
            #   - half=True: FP16 precision for 2x speed boost on GPU
            #   - imgsz=640: Input resolution (640x640 pixels)
            #   - verbose=False: Suppress console output
            self.last_results = self.model.predict(
                image_source,
                device=self.device,
                verbose=False,
                half=True,      # FP16 inference for speed
                imgsz=640       # Standard YOLO input size
            )

            # Parse detection results
            for result in self.last_results:
                boxes = result.boxes      # All detected bounding boxes
                names = result.names      # Class names (e.g., "chair", "person")

                if boxes is not None:
                    # Iterate through all detected objects
                    for box in boxes:
                        # Get class ID and name
                        cls_id = int(box.cls[0])
                        label = names[cls_id].lower()

                        # Check if this is our target object
                        # Flexible matching: "chair" matches "dining chair", etc.
                        if target_object.lower() in label or label in target_object.lower():

                            # Extract bounding box coordinates
                            # xyxy format: [x1, y1, x2, y2] (top-left, bottom-right)
                            xyxy = box.xyxy[0].cpu().numpy()
                            x1, y1, x2, y2 = map(int, xyxy)

                            # Convert to width/height format for easier use
                            # This format is more intuitive for robotics calculations
                            return {
                                'x': x1,                # Left edge
                                'y': y1,                # Top edge
                                'width': x2 - x1,       # Box width
                                'height': y2 - y1       # Box height
                            }

            # Target object not found in frame
            return {}

        except Exception as e:
            # Propagate errors to Central Hub for handling
            raise RuntimeError(f"Object detection failed: {e}")


    def show_results(self):
        """
        Visualization helper - displays last detection results

        Used for:
            - Development/debugging
            - Demo presentations (like this one!)
            - Not used in production (Unity MR handles visualization)

        Shows:
            - Bounding boxes around all detected objects
            - Class labels and confidence scores
            - Color-coded by object type
        """
        if self.last_results is None:
            print("No results to show. Call detect() first.")
            return

        # Display using YOLO's built-in visualization
        for result in self.last_results:
            result.show()


# =============================================================================
# EXAMPLE USAGE - Simulates Central Hub calling Object Detection
# =============================================================================

def demo_object_detection():
    """
    Demonstration of how the Central Hub interacts with Object Detection

    In the actual system:
        1. Central Hub receives video stream from drone
        2. Captures frame every 100ms (from flowchart specification)
        3. Calls detector.detect(target_object, frame)
        4. Receives bounding box coordinates
        5. Stores coordinates in Global Variables
        6. Passes to Depth Detection and other branches
    """

    print("="*60)
    print("NIMBUS AI - OBJECT DETECTION DEMO")
    print("="*60)

    # Step 1: Initialize detector (done once at startup)
    detector = ObjectDetector()

    # Step 2: Simulate receiving target object from Audio Pipeline
    # In real system: comes from LLM after processing "go to the chair"
    target_object = "chair"
    print(f"\nTarget object from user command: '{target_object}'")

    # Step 3: Simulate video frame from drone
    # In real system: this is a frame from the live video stream
    image_source = "path/to/test/image.jpg"  # Replace with actual image
    print(f"Processing video frame...")

    # Step 4: Run detection (this happens asynchronously in real system)
    bounding_box = detector.detect(target_object, image_source)

    # Step 5: Output results (sent to Central Hub)
    if bounding_box:
        print(f"\nObject '{target_object}' FOUND!")
        print(f"Bounding Box Coordinates:")
        print(f"  - X: {bounding_box['x']}")
        print(f"  - Y: {bounding_box['y']}")
        print(f"  - Width: {bounding_box['width']}")
        print(f"  - Height: {bounding_box['height']}")
        print(f"\nThese coordinates are now sent to:")
        print(f"  1. Central Hub -> Global Variables")
        print(f"  2. Depth Detection (to calculate distance)")
        print(f"  3. MR Branch (to draw overlay in Unity)")
        print(f"  4. Robotics Branch (to navigate toward object)")
    else:
        print(f"\nObject '{target_object}' NOT FOUND in frame")
        print(f"System will continue processing next frame...")

    # Step 6: Visualize results (demo purposes only)
    print(f"\nDisplaying detection results...")
    detector.show_results()


if __name__ == "__main__":
    demo_object_detection()
