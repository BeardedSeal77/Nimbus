from ultralytics import YOLO
import cv2
import numpy as np
import threading

# Load YOLOv8 model
model = YOLO("yolov8n.pt") 

"""
# Global lock to ensure thread-safe model prediction
model_lock = threading.Lock()

# Frame counter for skip logic
_frame_counter = 0
_frame_skip_interval = 5  # Process every 5th frame
"""

def object_detect_node(intent: str, target_object: str, video_frame: np.ndarray) -> dict:
    """
    Detects the specified object in the given frame using YOLOv8.

    Parameters:
        intent (str): The intent (e.g. "Go") – not used for now but available
        target_object (str): Name of the object to detect (e.g. "chair")
        video_frame (np.ndarray): BGR image from OpenCV

    Returns:
        dict: Bounding box in format {'x': x, 'y': y, 'width': w, 'height': h}
              or {} if not found
    """

    """
    global _frame_counter
    _frame_counter += 1

    # Skip frames to improve performance
    if _frame_counter % _frame_skip_interval != 0:
        return {}
    """

    try:

        """
        # Downscale for faster inference
        resized = cv2.resize(video_frame, (640, 480))

        # Thread-safe inference
        with model_lock:
            results = model.predict(resized, verbose=False)
        """

        # Run detection
        results = model.predict(video_frame, verbose=False)

        for r in results:
            boxes = r.boxes
            names = r.names  

            for box in boxes:
                cls_id = int(box.cls[0])
                label = names[cls_id].lower()
                
                if target_object.lower() in label or label in target_object.lower():
                    xyxy = box.xyxy[0].cpu().numpy()
                    x1, y1, x2, y2 = map(int, xyxy)

                    """
                    # Scale bounding box back to original frame size
                    h_ratio = video_frame.shape[0] / resized.shape[0]
                    w_ratio = video_frame.shape[1] / resized.shape[1]

                    return {
                        'x': int(x1 * w_ratio),
                        'y': int(y1 * h_ratio),
                        'width': int((x2 - x1) * w_ratio),
                        'height': int((y2 - y1) * h_ratio)
                    }
                    """

                    return {
                        'x': x1,
                        'y': y1,
                        'width': x2 - x1,
                        'height': y2 - y1
                        #'confidence': float(box.conf[0])
                    }

        # No matching object
        return {}

    except Exception as e:
        raise RuntimeError(f"Object detection failed: {e}")
